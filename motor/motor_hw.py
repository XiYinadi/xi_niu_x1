#!/usr/bin/env python3
"""Motor hardware abstraction — dual-channel H-bridge + GMR quadrature encoders.

Provides these layers:
  * DualSoftPwm  — single-thread software PWM for two motor channels (no hw PWM
                   pins on this header; 1 kHz, duty-fraction API).
  * SoftServoPwm — single-channel software PWM for a hobby servo (50 Hz frame,
                   pulse-width in µs). Separate class, not DualSoftPwm at 50 Hz:
                   the motors need 1 kHz and the servo needs 50 Hz at once.
  * QuadDecoder  — GMR A/B quadrature decoder via libgpiod edge events (4x).
  * GpioMotorHW  — real backend using libgpiod v1.x (/dev/gpiochipN).
  * FakeMotorHW  — simulated backend for headless unit tests.

The ROS2 node (motor_driver.py) only talks to the MotorHW interface, so it can
run against FakeMotorHW without any hardware attached.
"""

import math
import os
import sys
import threading
import time

# AidLux: ROS2 env overrides PYTHONPATH, libgpiod bindings live in system site-packages
for site_pkg in [
    "/usr/local/lib/python3.10/dist-packages",
    "/usr/lib/python3/dist-packages",
]:
    if site_pkg not in sys.path and os.path.isdir(site_pkg):
        sys.path.insert(0, site_pkg)

try:
    import gpiod
    _GPIO_AVAILABLE = True
except ImportError:
    _GPIO_AVAILABLE = False


# --------------------------------------------------------------------------
#  Software PWM (single thread, two channels, sleep-then-spin timing)
# --------------------------------------------------------------------------
class DualSoftPwm:
    """Two software PWM channels driven by one daemon thread at ``freq`` Hz.

    set_duty() is thread-safe. Direction pins are separate (see GpioMotorHW).

    Two invariants this class exists to uphold — both were violated once and
    both violations produced bugs that were expensive to find:

    * **No spurious pulse on an idle channel.** When both duties are ~0 the
      thread parks with both lines low; when only one is non-zero, the other
      line is never raised at all. The original implementation raised *both*
      lines every period and dropped each at its own duty, so a duty=0 channel
      received a few-µs HIGH pulse each 1 ms (~0.5 % duty) — enough to make an
      "idle" motor buzz, and enough to poison single-wheel isolation tests.
    * **Never spin away the whole PWM period.** A pure busy-wait for the entire
      period holds the GIL and starves every other thread in this process (the
      200 Hz control loop dropped to 1–2 Hz). See ``_busy_until`` below for the
      measurements and for why the coarse-sleep + short-spin split is correct.
    """

    def __init__(self, chip, line_l, line_r, freq=1000.0, logger=None):
        self._chip = chip
        self._l = self._chip.get_line(line_l)
        self._r = self._chip.get_line(line_r)
        self._l.request(consumer='motor_pwm_l', type=gpiod.LINE_REQ_DIR_OUT)
        self._r.request(consumer='motor_pwm_r', type=gpiod.LINE_REQ_DIR_OUT)
        self._l.set_value(0)
        self._r.set_value(0)
        self.freq = float(freq)
        self._duty_l = 0.0
        self._duty_r = 0.0
        self._lock = threading.Lock()
        self._run = threading.Event()
        self._stop = threading.Event()
        self._thread = None
        self._logger = logger

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._run.set()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        self._run.clear()
        self._stop.set()
        self._l.set_value(0)
        self._r.set_value(0)

    def set_duty(self, d_l, d_r):
        with self._lock:
            self._duty_l = min(max(float(d_l), 0.0), 1.0)
            self._duty_r = min(max(float(d_r), 0.0), 1.0)

    def set_direction(self, dir_l, dir_r):
        # NOTE: direction pins are owned by GpioMotorHW, not here.
        raise NotImplementedError("use GpioMotorHW.set_direction()")

    def _loop(self):
        period = 1.0 / self.freq
        while not self._stop.is_set():
            with self._lock:
                hl = self._duty_l * period
                hr = self._duty_r * period
            # Park with both lines low when nothing is being driven.
            if hl <= 0.0 and hr <= 0.0:
                self._l.set_value(0)
                self._r.set_value(0)
                time.sleep(period)
                continue
            t0 = time.monotonic()
            # 只拉高真正要驱动的通道。duty=0 的那一路必须**全程保持低电平**。
            #
            # 原实现是无条件把两条线都拉高、再按各自 duty 落下。于是当一路
            # duty=0、另一路非 0 时,duty=0 那条线每周期都会吃到一个宽度约等于
            # 一次 set_value 调用开销(几 µs)的高脉冲。1kHz 下那是 ~0.5% 的
            # 占空比 —— 足以让"本该静止"的电机嗡嗡响或轻微蠕动,直接污染
            # 单轮隔离测试的判读(你会分不清某个轮子动是被驱动的还是被毛刺带的)。
            if hl > 0.0:
                self._l.set_value(1)
            if hr > 0.0:
                self._r.set_value(1)
            if hl < hr:
                self._busy_until(t0 + hl)
                if hl > 0.0:
                    self._l.set_value(0)
                self._busy_until(t0 + hr)
                self._r.set_value(0)
            elif hr < hl:
                self._busy_until(t0 + hr)
                if hr > 0.0:
                    self._r.set_value(0)
                self._busy_until(t0 + hl)
                self._l.set_value(0)
            else:
                # 走到这里必有 hl == hr > 0:两者同为 0 已在上面 park 分支拦掉。
                self._busy_until(t0 + hl)
                self._l.set_value(0)
                self._r.set_value(0)

            # Both outputs must remain LOW for the rest of this PWM period.
            # Without this wait, equal 10% commands repeat after only 0.1ms,
            # producing nearly continuous HIGH instead of 1kHz / 10% duty.
            self._busy_until(t0 + period)

    # 忙等的精度余量:超过它就先 sleep,只有最后这段才自旋。
    # 取值权衡:内核 sleep 的过冲通常在 50~100µs,余量必须远大于它才能把
    # 过冲吸收掉、仍准时醒来;但余量越大自旋越久,占着 GIL 不放的时间越长。
    COARSE_SLEEP = 0.0003       # 300µs

    @staticmethod
    def _busy_until(target):
        """等到 target,但**不要把整个等待都自旋掉**。

        2026-09-15 实测(必须保留这段历史,否则很容易"优化"回去):
        原实现是 `while time.monotonic() < target: pass`,对**整个 PWM 周期**
        自旋。duty≠0 时该线程死占 GIL(top -H 显示 93~100% CPU),把同进程的
        主循环饿到 **1~2Hz**(stderr 状态行 `轮询 2Hz/目标200Hz`)。后果是
        duty 半秒才更新一次,且 UDP 每 0.5s 才 drain 一次导致内核接收缓冲溢出
        —— 实测注入 60 包/s、pwm_proc 只收到 24 包/s,丢掉的都是真实摇杆值。
        症状是"车一顿一顿、和摇杆不同步"。

        `sys.setswitchinterval(0.001)` 救不回来:自旋线程在被交棒后会立刻把
        GIL 抓回去,主循环 sleep 醒来抢不过它。

        修法依据一个观察:**精度只对短的边沿区间重要,长的等待不需要自旋**。
        所以先 sleep 到"目标前 300µs"再自旋收尾 —— 短脉冲(低 duty 的高电平)
        因为本身就短,走不到 sleep 分支,精度不受影响;长等待则把 GIL 交出去。
        """
        remaining = target - time.monotonic()
        if remaining > DualSoftPwm.COARSE_SLEEP:
            time.sleep(remaining - DualSoftPwm.COARSE_SLEEP)
        while time.monotonic() < target:
            pass


# --------------------------------------------------------------------------
#  Servo PWM (one channel, 50 Hz frame, pulse width in µs)
# --------------------------------------------------------------------------
class SoftServoPwm:
    """One software-PWM channel for a hobby servo: 50 Hz frame, µs pulse.

    Why this is not ``DualSoftPwm(freq=50)``: that class drives *two* channels
    off *one* period and expresses its output as a duty **fraction** of that
    period. The motors need 1 kHz and the servo needs 50 Hz, so one instance
    cannot serve both. (``DualSoftPwm._loop`` also reads ``period`` once at
    entry, so mutating its ``freq`` at runtime silently does nothing.)

    So this class signals in µs instead. SG90-class: ~500–2400 µs full travel,
    ~1500 µs centre.

    Two invariants, inherited from DualSoftPwm and both load-bearing:

    * **No pulse at all when not driving.** ``park()`` leaves the line low
      forever. A servo fed a continuous pulse actively holds against its
      linkage and draws current; with no pulses it goes limp and draws ~0.
      That matters here because the servo's 5 V comes off the board's shared
      header rail (pin 2/4), which already feeds the L298N logic and the
      camera — see the 5 V warning in motor_wiring.md.
    * **Never spin away the whole frame.** Reuses ``DualSoftPwm._busy_until``
      verbatim — do not write a new spin loop here. Measured GIL cost is
      ~2 × 300 µs per 20 ms frame ≈ 3 %, versus the motor thread's ~30 %.
      Still: after adding any output this way, confirm the status line's
      ``轮询`` field is still 199–200 Hz. That field is the only guard rail.
    """

    def __init__(self, chip_name, line, freq=50.0, center_us=1500.0,
                 range_us=500.0, min_us=500.0, max_us=2500.0,
                 invert=False, deadband=0.0, hysteresis_us=0.0, logger=None):
        if not _GPIO_AVAILABLE:
            raise RuntimeError(
                'gpiod (libgpiod v1.x python bindings) not importable. '
                'Install with: sudo apt install python3-gpiod')
        self._logger = logger
        self.freq = float(freq)
        self._center = float(center_us)
        self._range = float(range_us)
        self._lo = float(min_us)
        self._hi = float(max_us)
        self._invert = bool(invert)
        self._deadband = abs(float(deadband))
        # 脉宽域滞回(2026-10-19,「按住转向时抖」):输入 a 的微小抖动会让舵机每帧
        # 都重新纠偏。_hyst 之内的变化直接不更新 —— 这是**指令量化**,不是斜率
        # 限幅:正常打方向每步远超它(不增加跟随延迟),park→驱动时
        # (_pulse is None)无条件接受第一帧。
        self._hyst = abs(float(hysteresis_us))
        self._stat_lock = threading.Lock()
        self._widths = []          # 最近若干帧**实测**高电平时长(秒)
        self._cmds = []            # 对应指令脉宽(µs)
        self._stat_max = 250       # ≈5 秒(50Hz)

        # 脉冲必须短于帧周期。把 servo_freq 往上调错会让 _busy_until 立即返回、
        # 循环空转成远短于 20ms 的高电平 —— 舵机收到的就不是脉冲串而是**持续高
        # 电平**,会堵转发热直到烧掉。两行守卫换一个舵机,值。
        period_us = 1e6 / self.freq
        if self._hi >= period_us:
            raise ValueError(
                'servo_max_us=%.0f 必须小于帧周期 %.0fµs (servo_freq=%.1fHz)。'
                '把 freq 调高会让舵机收到持续高电平而烧毁。'
                % (self._hi, period_us, self.freq))
        if self._lo < 0.0 or self._hi <= self._lo:
            raise ValueError('servo_min_us/max_us 非法: %.0f / %.0f'
                             % (self._lo, self._hi))

        # 自己再开一个 chip 句柄 + 申请自己的 line。同进程开两次 chip 没问题 ——
        # 独占的是 line 不是 chip,而这里要的是一条和电机完全不同的 line。
        self._chip = gpiod.Chip(chip_name)
        self._line = self._chip.get_line(int(line))
        self._line.request(consumer='servo_steer', type=gpiod.LINE_REQ_DIR_OUT)
        # 和 DualSoftPwm 一样:申请完立刻压到低电平,堵住「申请到第一帧之间」的窗口。
        self._line.set_value(0)

        self._pulse = None          # None = park(不发脉冲)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = None

    def start(self):
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def stop(self):
        with self._lock:
            self._pulse = None
        self._stop.set()
        try:
            self._line.set_value(0)
        except Exception:
            pass

    # ---------------------------------------------------------------- command
    def set_command(self, x):
        """归一化转向指令 x∈[-1,1] → 脉宽。**invert 在这里生效。**

        中位死区(servo_deadband)也在这里生效:目的是掐掉摇杆略偏中位时舵机的
        嗡嗡声和白吃的电流,**不是**在指令层加斜率限幅 —— 那个明确不做,因为舵机
        自己的控制器就限速,指令层再加限幅只会增加延迟、并不减少电流。
        """
        x = max(-1.0, min(1.0, float(x)))
        if self._deadband > 0.0 and abs(x) < self._deadband:
            x = 0.0
        if self._invert:
            x = -x
        self.set_pulse_us(self._center + x * self._range)

    def set_pulse_us(self, us):
        """直接给脉宽(µs),仍然过电气硬限位。"""
        us = min(self._hi, max(self._lo, float(us)))
        with self._lock:
            cur = self._pulse
            # cur is None = park:必须**无条件**接受,否则恢复驱动时的第一帧会被
            # 滞回吞掉(舵机晚 20ms 才动)。
            if cur is not None and self._hyst > 0.0 and abs(us - cur) < self._hyst:
                return
            self._pulse = us

    def park(self):
        """不发任何脉冲 → 舵机松掉、几乎不吃电。"""
        with self._lock:
            self._pulse = None

    @property
    def parked(self):
        return self._pulse is None

    @property
    def pulse_us(self):
        return self._pulse

    def _record(self, t_rise, t_fall, us):
        w = t_fall - t_rise
        with self._stat_lock:
            self._widths.append(w)
            self._cmds.append(us)
            if len(self._widths) > self._stat_max:
                del self._widths[0]
                del self._cmds[0]

    def stats(self):
        """最近约 5 秒的**实测**高电平时长(µs)。只给状态行看,不参与控制。"""
        with self._stat_lock:
            if not self._widths:
                return None
            w = sorted(x * 1e6 for x in self._widths)
            n = len(w)
            return {
                'n': n,
                'cmd': self._cmds[-1],
                'min': w[0],
                'p50': w[n // 2],
                'p95': w[min(n - 1, int(n * 0.95))],
                'max': w[-1],
                'mean': sum(w) / n,
            }

    # ---------------------------------------------------------------- loop
    def _loop(self):
        period = 1.0 / self.freq
        while not self._stop.is_set():
            with self._lock:
                us = self._pulse
            if us is None:
                # 不驱动时**一个脉冲都不发**。这和 DualSoftPwm 的「空闲通道」是
                # 同一类不变量:那边是不许有毛刺脉冲,这边是不许有脉冲。
                self._line.set_value(0)
                time.sleep(period)
                continue
            t0 = time.monotonic()
            self._line.set_value(1)
            # 2026-10-19: 脉宽起点必须取**实际拉高之后**的时刻。原实现取在
            # set_value(1) 之前,而这个 syscall 会被电机 PWM 线程的 GIL 自旋
            # 推迟(最多 ~300µs)⇒ 每帧高电平比指令短 0~300µs,舵机「按住转向」
            # 时会来回找角度。这是抖动的来源之一。
            t_rise = time.monotonic()
            # 高电平只有 1~2.5ms,整段自旋(不 sleep):_busy_until 的 sleep 分支
            # 一旦过冲就是几十~上千 µs,而这段时间线还拉着高 —— 舵机看到的就是
            # 一个明显偏长的脉冲。自旋的代价是这段时间占着 GIL,所以上面把
            # sys.setswitchinterval 调到了 0.2ms,让它能按时让出。
            _t_end = t_rise + us / 1e6
            while time.monotonic() < _t_end:
                pass
            # 时间戳取在拉低**之前**:拉低之后再取,一旦线程在两次调用之间被
            # 抢占,记录到的是「调度延迟」而不是高电平时长 —— 会把 1.7ms 的脉冲
            # 记成 15ms,看起来像舵机自己在发疯(2026-10-19 第一次测量就撞上)。
            t_fall = time.monotonic()
            self._line.set_value(0)
            self._record(t_rise, t_fall, us)
            # 帧尾这段要等 ~18.5ms,远大于 COARSE_SLEEP,所以几乎全在 sleep、只有
            # 最后 300µs 自旋。用 _busy_until 而不是裸 sleep 是为了和 DualSoftPwm
            # 共用同一套时序写法;代价是那 300µs 的 GIL,而换来的帧对齐精度远超舵机
            # 能用的(它本身容差 ±10%)。**已知偏差**:帧锚点每轮重采样(与
            # DualSoftPwm._loop 同形),所以实际是 ~49.9Hz 而非 50.0Hz —— 0.2%,
            # 比舵机容差小 50 倍,不值得为它引入和现有类不同的写法。
            DualSoftPwm._busy_until(t0 + period)


# --------------------------------------------------------------------------
#  Quadrature decoder (libgpiod edge events, 4x)
# --------------------------------------------------------------------------
# 本机 libgpiod 绑定的两个反直觉之处(2026-09-15 实测,勿按 v1 文档写):
#
#   1. `event_wait(sec, nsec)` 的 sec 必须是**整数**。传 0.2 会抛
#      `TypeError: 'float' object cannot be interpreted as an integer`。
#   2. 事件**不从 LineBulk 读**。`bulk.event_wait()` 返回的是「有待处理事件的
#      Line 对象列表」;单条 line 上还有 `line.event_wait(sec, nsec)` 返回 bool,
#      以及 `line.event_read()` 读一个事件。LineEvent 的属性是 `.type`
#      (1=上升沿 2=下降沿),不是 `event_type()`/`line_offset()`。
#
# 最要命的是:`line.event_read()` 在**队列为空时会阻塞**。所以正确姿势是
# 先用非阻塞的 `line.event_wait(0, 0)` 探路,再 `event_read()`,绝不能
# `while True: event_read()` 读到 None —— 那是个死循环。
DRAIN_PER_LINE = 512        # 单次唤醒每线上限,防高速时饿死主循环


def drain_events(bulk, handler):
    """把 bulk 上所有待处理事件取干净,逐个交给 handler(offset, is_rising)。

    非阻塞。返回处理的事件数。
    """
    ready = bulk.event_wait(0, 0)
    if not ready:
        return 0
    n = 0
    for ln in ready:
        off = ln.offset()
        for _ in range(DRAIN_PER_LINE):
            if not ln.event_wait(0, 0):
                break
            ev = ln.event_read()
            handler(off, ev.type == gpiod.LineEvent.RISING_EDGE)
            n += 1
    return n


def quad_step(prev_state, a, b):
    """One 4x-quadrature step. Returns (delta, new_state).

    prev_state/new_state are 2-bit (A<<1|B). Tested directly without gpiod.
    """
    new_state = (int(a) << 1) | int(b)
    delta = QuadDecoder.QUAD[(prev_state << 2) | new_state]
    return delta, new_state


class QuadDecoder:
    """Count A/B quadrature edges of one GMR encoder via libgpiod.

    A dedicated daemon thread waits on the LineBulk and drains the event queue
    so no edges are lost while Python is busy. 4x decoder state machine.
    """

    # 4-bit lookup: index = (prev_state << 2) | new_state
    QUAD = [0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0]

    def __init__(self, chip, a_off, b_off, name='enc', logger=None):

        self.a_off = int(a_off)
        self.b_off = int(b_off)
        self._chip = chip
        self.bulk = self._chip.get_lines([self.a_off, self.b_off])
        # pull-up: unwired encoder lines idle high instead of floating → no
        # spurious edges/counts before the encoders are connected+powered.
        self.bulk.request(consumer=f'motor_{name}',
                          type=gpiod.LINE_REQ_EV_BOTH_EDGES,
                          flags=gpiod.LINE_REQ_FLAG_BIAS_PULL_UP)
        self._cur = [0, 0]          # current A, B levels
        self._prev_state = 0
        self.count = 0
        self.name = name
        self._stop = threading.Event()
        self._dead = threading.Event()
        self._error = None
        self._logger = logger
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        while not self._stop.is_set():
            try:
                # 阻塞等 0.2s 拿「有事件的 line 列表」;期间无事件就空转一圈。
                ready = self.bulk.event_wait(0, 200000000)
                if not ready:
                    continue
                drain_events(self.bulk, self._on_edge)
            except Exception as exc:
                # 曾经这里是裸 `break` + 吞异常,导致 event_wait 签名不匹配时
                # 解码线程启动即静默退出,计数器永远为 0 —— 现场表现为「线接对了
                # 但 /odom 不前进」,极难定位。错误必须吼出来。
                self._die(exc)
                return

    def _on_edge(self, off, is_rising):
        """4x 正交解码的单步状态转移。"""
        bit = 1 if is_rising else 0
        if off == self.a_off:
            self._cur[0] = bit
        else:
            self._cur[1] = bit
        delta, self._prev_state = quad_step(
            self._prev_state, self._cur[0], self._cur[1])
        self.count += delta

    def read(self):
        return self.count

    def _die(self, exc):
        """记录解码线程的致命错误,并通过 logger 吼出来。"""
        self._error = exc
        self._dead.set()
        msg = ('encoder %s decoder thread died: %s: %s — counts will stay '
               'frozen at %d' % (self.name, type(exc).__name__, exc, self.count))
        if self._logger is not None:
            self._logger.error(msg)
        else:
            print('[motor_hw] ERROR ' + msg, file=sys.stderr)

    @property
    def failed(self):
        """解码线程是否已死(死了则 count 不可信)。"""
        return self._error is not None

    def stop(self):
        self._stop.set()
        try:
            self.bulk.release()
        except Exception:
            pass


# --------------------------------------------------------------------------
#  Backends
# --------------------------------------------------------------------------
class MotorHW:
    """Abstract hardware interface consumed by the ROS2 node."""

    def start(self): ...
    def set_duty(self, d_l, d_r): ...
    def set_direction(self, dir_l, dir_r): ...
    def read_encoders(self): ...
    def stop(self): ...


class GpioMotorHW(MotorHW):
    """Real backend: libgpiod character device + software PWM + quadrature.

    Driver board interface (L298N-style): each motor has a PWM pin for speed
    plus a complementary INA/INB pair for direction — INA=1/INB=0 drives one
    way, INA=0/INB=1 the other. set_direction(True,True) parks the pair as
    INA=1/INB=0 (no speed while duty=0).
    """

    def __init__(self, chip_name, pwm_l, ina_l, inb_l, pwm_r, ina_r, inb_r,
                 enc_l_a, enc_l_b, enc_r_a, enc_r_b, pwm_freq=1000.0,
                 enable_encoders=True, logger=None):
        if not _GPIO_AVAILABLE:
            raise RuntimeError(
                'gpiod (libgpiod v1.x python bindings) not importable. '
                'Install with: sudo apt install python3-gpiod')
        self._logger = logger
        # NOTE: this build exposes `gpiod.Chip`, not the classic v1 `gpiod.chip()`
        self.chip = gpiod.Chip(chip_name)
        self._ina_l = self.chip.get_line(ina_l)
        self._inb_l = self.chip.get_line(inb_l)
        self._ina_r = self.chip.get_line(ina_r)
        self._inb_r = self.chip.get_line(inb_r)
        for consumer, ln in (('motor_ina_l', self._ina_l), ('motor_inb_l', self._inb_l),
                             ('motor_ina_r', self._ina_r), ('motor_inb_r', self._inb_r)):
            ln.request(consumer=consumer, type=gpiod.LINE_REQ_DIR_OUT)
            ln.set_value(0)
        self.pwm = DualSoftPwm(self.chip, pwm_l, pwm_r, pwm_freq, logger)
        self.enable_encoders = bool(enable_encoders)
        if self.enable_encoders:
            self.enc_l = QuadDecoder(self.chip, enc_l_a, enc_l_b, 'L', logger)
            self.enc_r = QuadDecoder(self.chip, enc_r_a, enc_r_b, 'R', logger)
        else:
            # 编码器未接线时不申请那 4 条 line:悬空 + pull-up 只会产生噪声
            # 边沿,白占 GPIO 且毫无信息量。
            self.enc_l = self.enc_r = None

    def start(self):
        self.pwm.start()

    def set_duty(self, d_l, d_r):
        self.pwm.set_duty(d_l, d_r)

    def set_direction(self, dir_l, dir_r):
        """Complementary INA/INB: True → INA=1,INB=0 ; False → INA=0,INB=1."""
        self._ina_l.set_value(1 if dir_l else 0)
        self._inb_l.set_value(0 if dir_l else 1)
        self._ina_r.set_value(1 if dir_r else 0)
        self._inb_r.set_value(0 if dir_r else 1)

    def read_encoders(self):
        if not self.enable_encoders:
            return 0, 0
        return self.enc_l.read(), self.enc_r.read()

    def stop(self):
        self.pwm.stop()
        try:
            self._ina_l.set_value(0)
            self._inb_l.set_value(0)
            self._ina_r.set_value(0)
            self._inb_r.set_value(0)
            if self.enable_encoders:
                self.enc_l.stop()
                self.enc_r.stop()
        except Exception:
            pass


class FakeMotorHW(MotorHW):
    """Simulated backend: duty→wheel speed with saturation + light noise.

    Encoder counters integrate the wheel speed over wall time, so the PID and
    odometry code paths run exactly as on real hardware.
    """

    def __init__(self, max_linear_speed=0.3, counts_per_meter=11752.0,
                 wheel_base=0.2, flip_l=False, flip_r=False, feedback=True,
                 logger=None):
        self._v_max = max_linear_speed
        self._cpm = counts_per_meter
        self._wheel_base = wheel_base
        self._flip_l = bool(flip_l)
        self._flip_r = bool(flip_r)
        self._feedback = bool(feedback)
        self._logger = logger
        self._duty_l = 0.0
        self._duty_r = 0.0
        self._dir_l = True
        self._dir_r = True
        self._counts = [0, 0]
        self._last_t = time.monotonic()
        self._lock = threading.Lock()

    def start(self):
        self._last_t = time.monotonic()

    def set_duty(self, d_l, d_r):
        with self._lock:
            self._duty_l = min(max(float(d_l), 0.0), 1.0)
            self._duty_r = min(max(float(d_r), 0.0), 1.0)

    def set_direction(self, dir_l, dir_r):
        with self._lock:
            self._dir_l = bool(dir_l)
            self._dir_r = bool(dir_r)

    def read_encoders(self):
        if not self._feedback:
            # 模拟「编码器没接线」:计数恒为 0。控制环里等价于 err ≡ 目标值,
            # 正是修复前「摇杆松手电机不停」的成因,所以 mock 必须能表达这个
            # 状态。注意不能用「把 counts_per_meter 调大」来代替 —— 那个参数
            # 同时喂给本类(_cpm)和驱动,分子分母一起缩放,怎么改都抵消。
            return (0, 0)
        now = time.monotonic()
        with self._lock:
            dt = now - self._last_t
            self._last_t = now
            # dir_* 是方向**引脚**电平,flip_* 表示电机接线反了 —— 实际转向是
            # 两者异或。少了这一层,flip_left:=true 的闭环会看到左编码器读数为
            # 负,PID 于是把 duty 一路顶到 0.83 去"纠正"一个纯方向问题(实测踩过),
            # 而真机上接线反只影响转向、不影响编码器自身符号。
            phys_l = self._dir_l != self._flip_l
            phys_r = self._dir_r != self._flip_r
            v_l = (self._duty_l if phys_l else -self._duty_l) * self._v_max
            v_r = (self._duty_r if phys_r else -self._duty_r) * self._v_max
            self._counts[0] += int(v_l * self._cpm * dt)
            self._counts[1] += int(v_r * self._cpm * dt)
            return tuple(self._counts)

    def stop(self):
        self.set_duty(0.0, 0.0)
        self.set_direction(True, True)
