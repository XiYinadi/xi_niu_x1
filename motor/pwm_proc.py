#!/usr/bin/env python3
"""直接 PWM 进程 —— 摇杆 → GPIO,完全绕开 ROS。

为什么要单独一个进程
--------------------
原来 PWM 是 motor_driver 进程里的一条 1kHz 纯 Python 忙等线程
(motor_hw.DualSoftPwm._busy_until),它和 rclpy 的单线程执行器抢同一个 GIL。
2026-09-15 用 rate_probe.py 实测 /wheel_speed 的间隔:
    中位 20.3ms(正常) / p90 116.6ms / p99 466.9ms
即控制回调被**间歇性**饿死。松手时 duty 要连降数步才能归零,每步都卡住,叠加
成 931ms 的松手延迟。把 PWM 挪进独立进程就拿到了自己的 GIL,那一条争用消失。

⚠️ 但别以为分完进程就万事大吉 —— 同一个忙等线程在**本进程内**照样能饿死**本进程
   自己的**主循环。这是同一类 bug 的第二次发作,只是受害者换了:
       轮询 2Hz/目标200Hz       ← 200Hz 的更新循环被拖到 1~2Hz
       注入 60 包/s 只收到 24   ← 主循环 0.5s 才 drain 一次,UDP 接收缓冲溢出
   真凶仍是 `_busy_until` 对整个 PWM 周期纯自旋、占着 GIL 不放。修法是
   「粗睡 300µs + 末段精等」,修完:轮询 199~200Hz、包率 59.6/s、
   PWM 线程 CPU 从 93~100% 降到 6.7%,边沿精度不变。
   完整原因与数值记在 `motor_hw.DualSoftPwm._busy_until` 的 docstring 里 ——
   动那段代码之前务必先读它,很容易「优化」回原样。

数据通路(全在本机,不经过 ROS)
------------------------------
    网页摇杆 60Hz
      → websocket → slam_viz._on_motor_control   (驾驶权仲裁在这)
      → UDP 127.0.0.1:5813  {"l": -1..1, "a": -1..1, "armed": bool}
      → 本进程
      → GpioMotorHW (DualSoftPwm + 互补方向脚) → GPIO

用 UDP 不用 TCP:这是「最新值」流,旧包没有价值 —— 丢一个立刻被下一个覆盖。
TCP 的队头阻塞/重传只会让 duty 用上已经过期的值。非阻塞 recv,每轮只取最新的那个。

duty 怎么算
-----------
    s_l, s_r = joy_to_channels(l, a, ...)      # 归一化带符号轮速,|s| 就是 duty
    duty     = clip(|s|, 0, max_duty)          # 低于 deadband 记 0
    方向      = s 的符号,再按 flip_* 翻

直行时 duty 严格等于摇杆比值 —— 摇杆 0.8 就是 0.8,**没有斜率限幅、没有滤波**。
代价是换向瞬间的电流冲击更大,依赖硬件承受。这是明确选定的取舍,不是疏漏。

安全(每一条都不花额外延迟,每轮一次比较)
----------------------------------------
  * 上电 / 没收到过包   → duty 0。
  * 超过 deadman_ms 无新包 → duty 0。ROS 那条看门狗被绕开了,这是它在本进程里
    的零成本替代:网页崩了、断网、关标签页,机器人自己会停。
  * 没 armed           → duty 0(必须网页点「启用电机」)。
  * 退出 / 异常        → finally 里停 PWM、方向脚归零。

⚠️ 不要和 motor_driver 同时跑:两者申请同一批 GPIO line,libgpiod 独占,
后启动的那个会直接报 line busy 起不来。这个失败是响亮且安全的(不会两边
打架互相灌值),但看到这个报错要先把 motor_driver 停掉。

用法:
    python3 ~/pwm_proc.py                       # 用同目录的 motor_params.yaml
    python3 ~/pwm_proc.py --params ~/motor_params.yaml --port 5813
"""

import argparse
import json
import os
import signal
import socket
import subprocess
import sys
import threading

# 2026-10-19: 本进程里两个线程都要做 µs 级时序(电机 1kHz / 舵机 50Hz),
# 而 CPython 默认每 5ms 才强制切换一次 GIL —— 舵机线程 sleep 醒来后最长要等
# 5ms 才能跑,脉宽被拖到 3.4ms(p95)/16ms(max),现场表现就是「按住转向时抖」。
# 压到 0.2ms:切换开销可忽略,而两个线程的等待上限都降了一个数量级。
sys.setswitchinterval(0.0002)
import time

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

from motor_control import Deadman, IdlePark, joy_to_channels
from motor_hw import GpioMotorHW, SoftServoPwm

DEFAULT_PARAMS = os.path.join(_HERE, 'motor_params.yaml')
DEFAULT_PORT = 5813          # 5811/5812 已被 cam_stream / fire_detect 占用
DEFAULT_HZ = 200.0           # duty 更新率;一个 PWM 周期只有 1ms,200Hz 足够跟手
DEFAULT_DEADMAN_MS = 300.0
STATUS_EVERY = 5.0           # 每 N 秒打一行状态,方便从日志判断链路是否还活着


def load_params(path):
    """读 motor_params.yaml 的 motor_driver.ros__parameters 段。

    不需要 ROS 的 parameter 机制 —— 那就是个 YAML,直接解析。这样 pwm_proc
    可以完全跑在系统 python3 上(和 cam_stream.py / fire_detect.py 一样),
    ROS 环境变量破坏 V4L2 之类的坑也一并绕开了。
    """
    try:
        import yaml
    except ImportError:
        sys.exit('需要 PyYAML: sudo apt install python3-yaml')
    try:
        with open(path, 'r', encoding='utf-8') as f:
            doc = yaml.safe_load(f)
    except OSError as e:
        sys.exit('读不到参数文件 %s: %s' % (path, e))
    try:
        return doc['motor_driver']['ros__parameters']
    except (TypeError, KeyError):
        sys.exit('%s 里没有 motor_driver.ros__parameters 段' % path)


class ServoLink:
    """舵机 PWM 的**代理**:真正的脉冲由独立进程 servo_proc.py 生成。

    2026-10-19: 原来舵机 PWM 在本进程里跑线程,和 1kHz 电机 PWM 线程抢 GIL,
    实测脉宽 p50=1756µs 但 p95=3.4ms / max=16ms(指令 1.75ms)⇒ 按住转向时抖。
    本类只负责:起停子进程 + 把归一化指令发过去。
    接口与 SoftServoPwm 对齐(park/set_command/parked/pulse_us/stats),所以
    PwmProc 里的门控(stale/armed/idle_park)一个字都不用改。
    ⚠️ 子进程 300ms 收不到消息会自己 park —— 本进程崩了舵机也不会一直咬转角。
    """

    def __init__(self, chip_name, line, freq=50.0, center_us=1500.0,
                 range_us=500.0, min_us=500.0, max_us=2500.0,
                 invert=False, deadband=0.0, hysteresis_us=0.0, logger=None):
        self._center = float(center_us)
        self._range = float(range_us)
        self._lo = float(min_us)
        self._hi = float(max_us)
        self._invert = bool(invert)
        self._deadband = abs(float(deadband))
        self._hyst = abs(float(hysteresis_us))
        self._pulse = None
        self._lock = threading.Lock()
        self._port = 5816
        self._addr = ('127.0.0.1', self._port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        script = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'servo_proc.py')
        logf = open('/tmp/servo_proc.log', 'ab', buffering=0)
        self._proc = subprocess.Popen(
            [sys.executable, script,
             '--chip', str(chip_name), '--line', str(line),
             '--freq', str(freq), '--center', str(center_us),
             '--range', str(range_us), '--min', str(min_us),
             '--max', str(max_us), '--port', str(self._port)],
            stdout=logf, stderr=subprocess.STDOUT)
        time.sleep(0.2)
        if self._proc.poll() is not None:
            raise RuntimeError('servo_proc 启动即退出(见 /tmp/servo_proc.log)')

    def _send(self, text):
        try:
            self._sock.sendto(text.encode('ascii'), self._addr)
        except OSError:
            pass

    def set_command(self, x):
        x = max(-1.0, min(1.0, float(x)))
        if self._deadband > 0.0 and abs(x) < self._deadband:
            x = 0.0
        if self._invert:
            x = -x
        us = min(self._hi, max(self._lo, self._center + x * self._range))
        with self._lock:
            cur = self._pulse
            if cur is not None and self._hyst > 0.0 and abs(us - cur) < self._hyst:
                us = cur            # 滞回:微小抖动不逐帧追
            self._pulse = us
        self._send('%.1f' % us)

    def park(self):
        with self._lock:
            self._pulse = None
        self._send('park')

    @property
    def parked(self):
        return self._pulse is None

    @property
    def pulse_us(self):
        return self._pulse

    def stats(self):
        return None                 # 实测脉宽在 /tmp/servo_proc.log 里

    def start(self):
        pass

    def stop(self):
        with self._lock:
            self._pulse = None
        self._send('park')
        try:
            self._proc.terminate()
            self._proc.wait(timeout=2.0)
        except Exception:
            try:
                self._proc.kill()
            except Exception:
                pass


class PwmProc:
    """摇杆 UDP 包 → 占空比 → GPIO。单线程,无锁竞争。"""

    def __init__(self, p, port):
        self._max_lin = float(p['max_linear_speed'])
        self._max_ang = float(p['max_angular_speed'])
        self._wheel_base = float(p['wheel_base'])
        self._swap_lr = bool(p.get('swap_lr', False))
        self._flip_l = bool(p.get('flip_left', False))
        self._flip_r = bool(p.get('flip_right', False))
        self._max_duty = float(p.get('max_duty', 0.9))
        self._deadband = float(p.get('pid_deadband', 0.05))
        self._pwm_freq = float(p.get('pwm_freq', 1000.0))
        self._rate_hz = float(p.get('pwm_cmd_hz', DEFAULT_HZ))
        self._deadman = float(p.get('pwm_deadman_ms', DEFAULT_DEADMAN_MS)) / 1000.0
        self._port = port

        # 舵机(辅助转向)。全部用 p.get():老 YAML 里没有这些键时不能 KeyError。
        # servo_enable=false 时下面**连 line 都不申请**,行为和没这个功能一样。
        self._servo_enable = bool(p.get('servo_enable', False))
        self._servo_center = float(p.get('servo_center_us', 1500.0))
        self._servo_range = float(p.get('servo_range_us', 500.0))
        self._servo_deadband = float(p.get('servo_deadband', 0.0))
        self._servo_line = p.get('servo_line')
        # 静止多久之后 park 舵机(秒)。0 = 关闭。
        # 为什么需要它:不 park 时舵机全程被通电顶在 center 上,内部模拟控制环一直
        # 纠偏 ⇒ 嗡嗡响 + 微抖 + 白吃电流(5V 还和 L298N 逻辑、相机共用排线)。
        # 实测已证明脉宽本身是干净的,所以这是唯一能改善它的软件手段。
        self._servo_idle_park = float(p.get('servo_idle_park_ms', 1000.0)) / 1000.0
        self.servo = None

        # 转向由谁做。三选一,不是一个「叠加」开关:
        #   diff  —— 老行为。a 只进差速混控,舵机 park(不参与)。
        #   servo —— a **只**给舵机,轮子只吃油门、两轮等速。这就是「取消差速」。
        #   both  —— a 两边都吃(当初「保留差速 + 舵机做辅助」的那个设计)。
        # diff 与 servo/both 是互斥的:同时让差速和舵机转向,两者会互相打架
        # (舵机把轮子掰向一边、差速又把车身转向另一边),所以不做成开关。
        #
        # 代码默认 'diff':老 YAML 里没有这个键时,行为与加它之前逐字节一致。
        # 交付的 motor_params.yaml 里显式写着 servo —— 那才是当前想要的。
        self._steer_mode = str(p.get('steer_mode', 'diff')).strip().lower()
        if self._steer_mode not in ('diff', 'servo', 'both'):
            sys.exit('[pwm_proc] steer_mode 非法: %r —— 只能是 diff / servo / both'
                     % self._steer_mode)
        self._a_to_wheels = self._steer_mode in ('diff', 'both')
        self._a_to_servo = self._steer_mode in ('servo', 'both')

        # 硬件:方向脚 + 软 PWM。编码器不开 —— 本进程只驱动,不做里程计。
        try:
            self.hw = GpioMotorHW(
                chip_name=p['gpiochip'],
                pwm_l=p['pwm_l_line'], ina_l=p['ina_l_line'], inb_l=p['inb_l_line'],
                pwm_r=p['pwm_r_line'], ina_r=p['ina_r_line'], inb_r=p['inb_r_line'],
                enc_l_a=p['enc_l_a'], enc_l_b=p['enc_l_b'],
                enc_r_a=p['enc_r_a'], enc_r_b=p['enc_r_b'],
                pwm_freq=self._pwm_freq,
                enable_encoders=False,
            )
        except Exception as e:
            sys.exit('[pwm_proc] 申请 GPIO 失败: %s\n'
                     '  最常见的原因: motor_driver 正在跑,已经占着这批 line。\n'
                     '  先把它停掉再启动本进程(GPIO 是独占的,不能两边同时开)。' % e)

        # 舵机在 hw.start() **之前**构造:这样它若有问题,还没有任何 PWM 线程
        # 在跑,不会留下一个半启动的进程。
        if self._servo_enable:
            try:
                self.servo = ServoLink(
                    chip_name=p['gpiochip'],
                    line=self._servo_line,
                    freq=float(p.get('servo_freq', 50.0)),
                    center_us=self._servo_center,
                    range_us=self._servo_range,
                    min_us=float(p.get('servo_min_us', 500.0)),
                    max_us=float(p.get('servo_max_us', 2500.0)),
                    invert=bool(p.get('servo_invert', False)),
                    deadband=self._servo_deadband,
                    hysteresis_us=float(p.get('servo_hysteresis_us', 0.0)),
                )
            except Exception as e:
                # 文案必须点名是**舵机 line**。否则照抄上面那段「先停 motor_driver」
                # 会把人引向一个根本不存在的 GPIO 冲突 —— 电机那批 line 本进程
                # 已经拿到了,冲突只可能在 servo_line 自己身上。
                sys.exit('[pwm_proc] 舵机 GPIO 申请失败 (servo_line=%s): %s\n'
                         '  注意:这**不是** motor_driver 占着 line 的问题 —— 那批电机\n'
                         '  line 本进程已经拿到了。检查两点:\n'
                         '    1. servo_line 是否与电机/编码器 line 撞了;\n'
                         '    2. 该 line 是不是被别的进程占着(或本机 gpiochip 名字不对)。\n'
                         '  只想先跑电机:把 servo_enable 改回 false。'
                         % (self._servo_line, e))

        # steer_mode 选了 servo/both 但舵机没起来 —— 直接退出,不做「优雅降级」。
        # 因为那时轮子已经不吃 a 了(转向不再走差速),**整车完全没有转向**。
        # 而这种车在日志上和正常的完全一样,只表现为「打方向没反应」——
        # 正是 motor_wiring.md 里最不想再造一次的那种故障。半启动比不启动更糟。
        if self._a_to_servo and self.servo is None:
            sys.exit('[pwm_proc] steer_mode=%s 需要舵机,但舵机没构造起来 '
                     '(servo_enable=%s)。\n'
                     '  转向已经**从差速交给舵机**,没有舵机就是整车无法转向。\n'
                     '  二选一:\n'
                     '    1. 把 servo_enable 改成 true 并让 servo_line 接对、没被占用;\n'
                     '    2. 把 steer_mode 改回 diff —— 回到纯差速转向,舵机不参与。'
                     % (self._steer_mode, self._servo_enable))

        self.hw.start()
        if self.servo is not None:
            self.servo.start()

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock.bind(('127.0.0.1', port))
        self._sock.setblocking(False)

        self._l = 0.0
        self._a = 0.0
        self._armed = False
        self._last_pkt = 0.0        # 0 = 从没收到过,不是「很久以前」
        self._age_ms = 0.0
        self._dir_l = True
        self._dir_r = True
        self._duty = (0.0, 0.0)
        self._stale = True
        # 带滞回的失联判据(纯逻辑在 motor_control.Deadman,那里有它的回归测试)。
        # 实现在那个模块而不是这里,是因为**本文件没法单测** —— 见 Deadman 的类注释。
        self._deadman_gate = Deadman(self._deadman)
        # 「静止即 park」判据(纯逻辑在 motor_control.IdlePark,那里有它的单测)。
        # ⚠️ 它**不是**第二套链路门 —— 判的是指令内容(l/a 都在零附近),与链路通断
        #    无关,而且恢复侧没有延迟。别照着上面那个 Deadman 的样子给它加滞回。
        self._idle_park = IdlePark(self._servo_idle_park)
        self._n_pkt = 0
        self._n_loop = 0            # 实际跑过的更新轮数,用来核对 _rate_hz 有没有跑满
        self._run = True
        self._t_status = time.monotonic()

    # ---------------------------------------------------------------- UDP
    def _recv_latest(self):
        """非阻塞地把队列里的包**全部**取掉,只留最后一个的状态。

        取全部而不是取一个:网页以 60Hz 发,本进程 200Hz 收,每次只读一个也不会
        积压;但万一卡了一下,留在队列里的旧值正是最不该用的 —— 直接抽干,
        最后读到的那个就是最新状态。
        """
        got = False
        while True:
            try:
                data, _ = self._sock.recvfrom(512)
            except (BlockingIOError, InterruptedError):
                break
            except OSError:
                break
            try:
                m = json.loads(data.decode('utf-8', 'replace'))
            except ValueError:
                continue
            try:
                self._l = max(-1.0, min(1.0, float(m.get('l', 0.0))))
                self._a = max(-1.0, min(1.0, float(m.get('a', 0.0))))
            except (TypeError, ValueError):
                continue
            self._armed = bool(m.get('armed', False))
            got = True
            # 按**报文**计数 —— 原来 += 1 在循环外,数的是「收到东西的调用次数」,
            # 一轮里抽干两个包只算一个,报出来的包率会凭空少一半。
            self._n_pkt += 1
        if got:
            self._last_pkt = time.monotonic()

    # ---------------------------------------------------------------- control
    def _step(self):
        now = time.monotonic()
        if self._last_pkt > 0.0:
            self._age_ms = (now - self._last_pkt) * 1000.0
        # 没收到过包也算失联 —— 上电后不动,直到网页真的开始发。
        # ⚠️ 不要退回成裸阈值 `(now - self._last_pkt > self._deadman)`。那样在链路
        #    包间隔**恰好跨在阈值两侧**时会来回抖:来一包就恢复、过一会儿又跳闸。
        #    实测过一次(浏览器把后台标签页的 setInterval 钳到 1Hz,间隔 ~400ms):
        #    61 次严格交替的「中断/恢复」,而舵机与电机走**同一个门** ⇒ 舵机跟着
        #    park↔回中 61 次 ⇒ 现场表现就是「启用电机后舵机老是来回抖」。
        #    Deadman 的滞回把这种情况变成「跳闸一次,之后不再恢复」。
        age_s = None if self._last_pkt == 0.0 else (now - self._last_pkt)
        stale = self._deadman_gate.update(age_s, now)
        if stale != self._stale:
            self._stale = stale
            # 恢复那行把滞回时长打出来 —— 否则「为什么隔了 600ms 才恢复」在日志里
            # 完全没有解释,只看得到一行「恢复」,像是晚了。
            self._log('deadman: 摇杆数据中断 → duty 0' if stale
                      else 'deadman: 摇杆数据恢复(已连续 %.0fms 可收)'
                           % (self._deadman_gate.dwell_s * 1000.0))

        # 静止判据每拍都要喂,**不能只在舵机那个分支里喂** —— 否则死区/手动禁用的
        # 那段时间状态机的计时会停摆,等恢复时算出来的"静止了多久"是假的。
        idle_now = self._idle_park.update(self._l, self._a, now)

        if stale or not self._armed:
            s_l = s_r = 0.0
        else:
            # steer_mode='servo' 时给混控喂 a=0:轮子只吃油门,两轮等速。
            # 这不是「把 K 调小」那种程度上的减弱,是**彻底不给差速项** ——
            # a=0 代入后 s_l == s_r,方向只由 l 的正负决定。
            s_l, s_r = joy_to_channels(self._l, self._a if self._a_to_wheels else 0.0,
                                       self._max_lin, self._max_ang,
                                       self._wheel_base, self._swap_lr)

        d_l = min(self._max_duty, abs(s_l))
        d_r = min(self._max_duty, abs(s_r))
        if d_l < self._deadband:
            d_l = 0.0
        if d_r < self._deadband:
            d_r = 0.0

        dir_l = (s_l * (-1.0 if self._flip_l else 1.0)) >= 0.0
        dir_r = (s_r * (-1.0 if self._flip_r else 1.0)) >= 0.0

        # 换向保护:方向要翻时先把 duty 降到 0,空一个 PWM 周期再翻 —— 别让电机
        # 还在正转时 H 桥反向加压。原来这段在控制回调里 time.sleep(1ms) 阻塞,
        # 现在挂在本进程的更新循环上,占的是我们自己的时间,不占 ROS 执行器。
        if dir_l != self._dir_l or dir_r != self._dir_r:
            self.hw.set_duty(0.0, 0.0)
            time.sleep(1.0 / self._pwm_freq)
            self._dir_l, self._dir_r = dir_l, dir_r

        self.hw.set_direction(self._dir_l, self._dir_r)
        self.hw.set_duty(d_l, d_r)
        self._duty = (d_l, d_r)

        # 舵机走**同一个门**(stale / armed),这也是它不拆独立进程的全部理由。
        # 注意「同一个门」不等于原子:set_duty 和 set_command 是两次调用,两个线程
        # 周期分别是 1ms / 20ms,所以最坏有**一帧(20ms)**的错位窗口 —— 电机已归零、
        # 舵机还在发最后一个脉冲。deadman 情形最坏 = 300 + 20 = 320ms。比拆进程
        # (可能**永远**咬着转角)好得多,但别把它当原子。
        # ⚠️ 上面那句 320ms 在加了 Deadman 滞回之后**依然成立** —— 滞回只作用在
        #    **恢复**侧(跳闸后要连续 fresh 满 2×300ms 才准恢复),跳闸阈值本身没动。
        #    所以「多久归零」一个字没变,变的只是「不再抖着归零又放开」。
        #    **不要**给舵机单独做一套滞回:那样就会退回「电机停了、舵机还咬着转角」。
        # 不驱动时 park:发脉冲 = 舵机持续顶住 = 白吃电流,而它的 5V 走的是
        # 和 L298N 逻辑、相机共用的排针轨(见 motor_wiring.md 的红字)。
        if self.servo is not None:
            # not self._a_to_servo(即 steer_mode='diff')时恒 park:此时转向由
            # 差速做,舵机若跟着动就是两个机构对着掰,车会又转又拧。
            if stale or not self._armed or not self._a_to_servo:
                self.servo.park()
            # 车既没在动也没在转 → 舵机松掉(不发脉冲 = 不纠偏 = 不嗡嗡 = 不吃电)。
            # ⚠️ 这一层**不是**给链路加的滞回(那会回归成「电机停了、舵机还咬着
            #    转角」):它只看指令内容,且恢复侧是**立刻**的 —— 下一个非零样本
            #    就从上面那个分支走 set_command,舵机一秒都不会落后于指令。
            elif idle_now:
                self.servo.park()
            else:
                self.servo.set_command(self._a)

    def _log(self, msg):
        print('[pwm_proc] %s' % msg, flush=True)

    def _status(self):
        now = time.monotonic()
        dt = now - self._t_status
        if dt < STATUS_EVERY:
            return
        rate = self._n_pkt / dt if dt > 0 else 0.0
        loop = self._n_loop / dt if dt > 0 else 0.0
        self._n_pkt = self._n_loop = 0
        self._t_status = now
        # servo= 这一列是「servo_enable=true 但舵机没接线」**唯一**能被看见的地方。
        # 那种情况下没有 libgpiod 报错、没有别的不一样,只能表现为「转向没反应」。
        # 把「压根没构造」/「构造了但在 park」/「真的在发脉冲」区分开,而 park
        # 里又必须把「静止时我们主动 park」和「根本没在驱动」分开 —— 前者是正常的、
        # 后者才是故障,混在同一个 PARK 里就没人分得清转向为什么没反应了。
        # ⚠️ `PARK(idle)` 只在**idle park 确实是那根拦住它的原因**时才报,判据就是
        #    上面那个分支的条件取反:armed 且不 stale 且 a 真的该给舵机。否则
        #    「没使能 / 链路断了」(那样 servo 本来就会被 park)会被冒充成一个
        #    「正常静止」—— 而这两件事的排查方向正相反。
        driven = self._armed and not self._stale and self._a_to_servo
        if self.servo is None:
            sv = 'off'
        elif self.servo.parked:
            sv = 'PARK(idle)' if (driven and self._idle_park.idle) else 'PARK'
        else:
            sv = '%.0fus' % (self.servo.pulse_us or 0.0)
            _st = self.servo.stats()
            if _st:
                sv += '(实测 p50=%.0f p95=%.0f max=%.0f n=%d)' % (
                    _st['p50'], _st['p95'], _st['max'], _st['n'])
        # steer= 必须打出来:steer_mode=servo 时 a 只走舵机,**a 变化而 duty 不变
        # 是正常现象**。没有这一列的话,「打方向轮子不动」看起来和故障一模一样
        # —— 而这正是刚刚特意改成的行为。
        self._log('摇杆 %.1f包/s  轮询 %.0fHz/目标%.0fHz  龄期 %.0fms  armed=%s  '
                  'steer=%s  l=%+.2f a=%+.2f  duty L=%.2f R=%.2f  dir L=%d R=%d  servo=%s'
                  % (rate, loop, self._rate_hz, self._age_ms,
                     'Y' if self._armed else 'N', self._steer_mode,
                     self._l, self._a, self._duty[0], self._duty[1],
                     1 if self._dir_l else 0, 1 if self._dir_r else 0, sv))

    # ---------------------------------------------------------------- loop
    def run(self):
        period = 1.0 / self._rate_hz
        nxt = time.monotonic()
        while self._run:
            self._recv_latest()
            self._step()
            self._status()
            self._n_loop += 1
            nxt += period
            slack = nxt - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                nxt = time.monotonic()      # 落后了就重新对齐,别累积

    def stop(self):
        self._run = False

    def shutdown(self):
        try:
            self.hw.set_duty(0.0, 0.0)
            self.hw.set_direction(True, True)
        except Exception:
            pass
        try:
            self.hw.stop()
        except Exception:
            pass
        # 舵机在 hw.stop() **之后**停 —— 让舵机故障永远不可能挡住电机 park。
        try:
            if self.servo is not None:
                self.servo.stop()
        except Exception:
            pass
        try:
            self._sock.close()
        except Exception:
            pass


def main():
    ap = argparse.ArgumentParser(description='摇杆 → GPIO 直连 PWM 进程')
    ap.add_argument('--params', default=DEFAULT_PARAMS,
                    help='参数文件 (默认 %s)' % DEFAULT_PARAMS)
    ap.add_argument('--port', type=int, default=DEFAULT_PORT,
                    help='UDP 监听端口 (默认 %d)' % DEFAULT_PORT)
    args = ap.parse_args()

    # 本进程除了 PWM 线程就只有这个更新循环,把 GIL 切换间隔压低,让 set_duty
    # 的写入尽快落到 PWM 线程(默认 5ms → 1ms)。
    sys.setswitchinterval(0.001)

    params = load_params(args.params)
    proc = PwmProc(params, args.port)

    def _on_signal(signum, _frame):
        proc.stop()

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    proc._log('就绪 — UDP 127.0.0.1:%d  pwm_freq=%.0fHz  duty 直跟摇杆比值 '
              '(无限幅)  deadman=%.0fms  steer_mode=%s' %
              (args.port, params.get('pwm_freq', 1000.0),
               params.get('pwm_deadman_ms', DEFAULT_DEADMAN_MS),
               proc._steer_mode))
    if proc.servo is not None:
        proc._log('舵机 line=%s center=%.0fµs range=±%.0fµs invert=%s  idle_park=%s  (%s)' %
                  (params.get('servo_line'), proc._servo_center,
                   proc._servo_range, bool(params.get('servo_invert', False)),
                   ('%.0fms' % (proc._servo_idle_park * 1000.0))
                   if proc._servo_idle_park > 0.0 else 'off',
                   {'servo': 'a 只给舵机;轮子只吃油门、两轮等速',
                    'both':  'a 同时给舵机与差速混控(两个机构都在转向)',
                    'diff':  '未参与转向 —— 恒 park、不发脉冲'}
                   [proc._steer_mode]))
    try:
        proc.run()
    finally:
        proc.shutdown()
        proc._log('已停止,GPIO 已释放')


if __name__ == '__main__':
    main()
