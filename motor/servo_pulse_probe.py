#!/usr/bin/env python3
"""量 SoftServoPwm 实际发出的脉冲串 —— 回答「舵机抖,是不是我们的信号造成的」。

**为什么需要它**:舵机抖有两大类成因,而它们在现象上分不开 ——
  (a) 我们发出的脉宽本身在跳      → 软件问题,能改
  (b) 脉宽是准的,抖来自舵机/供电/共地/机械 → 软件一行都改不了
靠猜永远猜不出来,而这个脚本 30 秒就能把 (a) 排掉或坐实。

**它只测量,不改任何生产代码。** 用 Proxy 包住 `SoftServoPwm._line`,
`_loop()` 一个字都不动 —— 量的是真实路径,不是复刻出来的近似。

用法(在板子上):
    python3 servo_pulse_probe.py [每种工况秒数,默认 30]

⚠️ 它会申请 line 32 / 22 / 118。这三个是当前唯一确认空闲的,别和 pwm_proc
   抢(它占 33 舵机 + 91/61/62/63/60/124 电机)。冲突时 libgpiod 报 line busy。

=============================================================================
2026-09-15 实测结论(舵机「不操作时一直在微小抖动」,30s x 1500 帧/工况)
=============================================================================
    工况                     脉宽中位   p1/p99       >150µs 坏帧   >1000µs 坏帧
    空转 duty0(用户不操作时)  1502µs   1500/1625    0.40 次/秒     0.07 次/秒
    电机 duty0.9(最坏竞争)    1502µs   1499/1731    1.20 次/秒     0.03 次/秒

⇒ **静止时我们发出的信号是干净的**(中位误差 +2µs、帧周期 20008µs、无断流)。
  0.40 次/秒的孤立坏帧舵机根本听不见。所以「一直在抖」**不是**脉冲串造成的,
  往供电/共地/机械上查,别再来改软件。

  残余的 0.4~1.2 次/秒是软 PWM 在 GIL 下的固有抖动(自旋收尾时被交出 GIL,
  下降沿迟到),电机线程同时自旋时更明显。**不值得为它去动
  `DualSoftPwm._busy_until`** —— 那段 docstring 记着两次 GIL 饿死事故,
  为听不见的抖动去改它是在赌回归。

⚠️ **坏帧是随机的,逐次波动很大**:同样是 duty0.9,30s 那次最大偏差 +1434µs,
   8s 那次 +9501µs。**别拿单次运行的最大值下结论** —— 要么跑够 30s,要么只看
   中位/p1/p99 这些稳的量。结论「静止时信号是干净的」在所有次运行里都成立。

值得记下来的三件小事:`park()` 确实一个脉冲都不发(实测 0 次电平跳变,只有
100 次「把 0 写进已经为 0 的线」);帧周期无断流,可排除 park/unpark 循环;
舵机的 `servo_deadband` 对中位抖**无效** —— a=0 时指令本来就是 0,死区无事可做。
"""
import statistics as st
import sys
import threading
import time

sys.path.insert(0, '/home/aidlux')

# ★ 必须与 pwm_proc.main() 一致。默认 5ms 的切换间隔会让自旋收尾最多迟到 5ms,
#   量出来的抖动**远大于真实值** —— 我第一版就是漏了这一行,报出标准差 277µs、
#   最大偏差 6803µs 的假数据(真实是 72µs / 2276µs)。
sys.setswitchinterval(0.001)

import gpiod                                                    # noqa: E402
from motor_hw import DualSoftPwm, SoftServoPwm                  # noqa: E402

SERVO_LINE, MOT_L, MOT_R = 32, 22, 118
TARGET_US = 1500.0
FS = 50.0                    # 舵机帧率,只用来把坏帧率换算成「次/秒」

_events = []


class Proxy:
    """记录每次 set_value 的时刻和值,再原样转发给真的 Line。"""

    def __init__(self, real):
        self._real = real

    def set_value(self, v):
        _events.append((time.monotonic(), v))
        self._real.set_value(v)


def _pairs(events):
    """把事件流切成 (上升沿, 下降沿) 配对。

    ★ 必须**顺着走**:每个 1 配它后面第一个 0。

    不能写成「滤出所有 1 得到一个表、滤出所有 0 得到另一个表,再 zip」。那样
    只要列表开头多一个孤立的 0(比如清空缓冲区时正好卡在一帧中间),**之后
    每一对都会错开一帧**,算出来的脉宽会变成 -(帧周期-脉宽) ≈ -18500µs ——
    一个物理上不可能的数。我第一版就是这么写的,并且差点拿它去下结论。
    """
    out, i = [], 0
    while i < len(events) - 1:
        if events[i][1] == 1 and events[i + 1][1] == 0:
            out.append((events[i][0], events[i + 1][0]))
            i += 2
        else:
            i += 1
    return out


def _transitions(events):
    """电平**跳变**次数。

    ★ 不能数 len(events):park 分支每帧都调一次 `set_value(0)`,而线本来
      就是 0。于是「不发脉冲」时会数出 50 次/秒的写入 —— 看起来像还在发脉冲。
      要数的是真实跳变。
    """
    n, prev = 0, None
    for _, v in events:
        if prev is not None and v != prev:
            n += 1
        prev = v
    return n


def analyze(label, seconds, duty, mot, srv):
    global _events
    mot.set_duty(duty, duty)
    _events = []
    time.sleep(seconds)
    p = _pairs(_events)
    if len(p) < 20:
        print('  %s: 样本不足 (%d)' % (label, len(p)))
        return
    pulse = [(b - a) * 1e6 for a, b in p]
    per = [(p[i + 1][0] - p[i][0]) * 1e6 for i in range(len(p) - 1)]
    xs = sorted(pulse)
    err = [x - TARGET_US for x in pulse]
    hit = sum(1 for e in err if abs(e) > 150)
    print('  %s   帧数 %d  坏帧率(>150µs) %.2f%%'
          % (label, len(p), 100.0 * hit / len(p)))
    print('    脉宽 中位%.0f p1 %.0f p99 %.0f 均值%.0f 标准差%.0f  最大偏差 %+.0fµs'
          % (xs[len(xs) // 2], xs[int(len(xs) * .01)], xs[int(len(xs) * .99)],
             st.mean(pulse), st.pstdev(pulse), max(err, key=abs)))
    print('    帧周期 中位%.0f 最小%.0f 最大%.0fµs (目标 20000;最小值若 ≫20000 '
          '就是有断流/park 循环)' % (st.median(per), min(per), max(per)))
    for thr in (150, 300, 500, 1000):
        n = sum(1 for e in err if abs(e) > thr)
        print('      >%4dµs: %3d 帧 = %.2f%%  → 约 %.2f 次/秒'
              % (thr, n, 100.0 * n / len(p), n / (len(p) / FS)))


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0

    mot = DualSoftPwm(gpiod.Chip('gpiochip0'), MOT_L, MOT_R, freq=1000.0)
    # ⚠️ 注意两个类构造参数不对称:SoftServoPwm 要 **chip 名字符串**,
    #    DualSoftPwm 要 **gpiod.Chip 对象**。传错是 TypeError,不是静默错。
    srv = SoftServoPwm('gpiochip0', SERVO_LINE, freq=FS,
                       center_us=TARGET_US, range_us=500.0)
    srv._line = Proxy(srv._line)

    stop = threading.Event()

    def mainloop():
        # 复刻 pwm_proc._step():200Hz 下持续下同一个指令。舵机的 set_command
        # 必须一直在调,否则测的就不是真实工况。
        while not stop.is_set():
            srv.set_command(0.0)
            time.sleep(0.005)

    mot.start()
    srv.set_pulse_us(TARGET_US)
    srv.start()
    threading.Thread(target=mainloop, daemon=True).start()
    time.sleep(0.3)

    print('=== SoftServoPwm 脉冲串实测 (line %d, switchinterval=0.001, 每工况 %.0fs) ==='
          % (SERVO_LINE, secs))
    analyze('空转 duty0 (用户「不操作」时)', secs, 0.0, mot, srv)
    analyze('电机 duty0.9 (最坏竞争)', secs, 0.9, mot, srv)

    # park 是不是真的一个脉冲都不发?这是「省电流/松舵机」的依据,值得实测。
    mot.set_duty(0.0, 0.0)
    time.sleep(0.1)
    # ★ 必须**先停掉上面那个 set_command 循环**再 park。它 5ms 一轮,会把
    #   _pulse 立刻设回 1500µs,于是 park 看起来像失效了 —— 我第二版就踩了这个,
    #   报出「park 后 2 秒内 100 个事件」的假故障(实为 50Hz x 2s 的写入次数)。
    stop.set()
    time.sleep(0.1)
    srv.park()
    _events[:] = []
    time.sleep(2.0)
    print('  park 断言: 2 秒内电平跳变 = %d (必须 0),写入次数 %d'
          % (_transitions(_events), len(_events)))

    srv.stop()
    mot.stop()


if __name__ == '__main__':
    main()
