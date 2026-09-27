#!/usr/bin/env python3
"""把电机控制脚钉死在固定电平,方便用万用表逐点排查「为什么电机不动」。

不用软件 PWM —— 输出的是**稳定直流电平**,万用表读数不会跳。

  左电机 (A): ENA=1  IN1=1  IN2=0   → 正转,满速
  右电机 (B): ENB=1  IN3=1  IN4=0   → 正转,满速

跑起来后会一直保持,Ctrl-C 退出。退出时会把所有脚拉低。

⚠️ 轮子必须架空 —— 这个状态下接到位的电机会全速转。

排查顺序(每步红表笔在左,黑表笔接犀牛派 GND):
  1. 犀牛派 PIN_32 (GPIO_91) → 应为 3.3V
     不到 3.3V = 软件没驱动成功,回来找我
  2. L298N 的 ENA 端子 → 应为 3.3V
     排针有 3.3V 但这里没有 = 杜邦线没接好/接错脚
  3. L298N 的 5V 端子 → 应为 5V
     没有 = 逻辑供电缺失(跳线帽 + VS,或从排针 pin2/4 供 5V,二选一)
  4. L298N 的 12V/VS 端子 → 应为电池电压
     没有 = 电机没供电,这就是「完全没反应」最常见的原因
  5. L298N 的 OUT1/OUT2 之间 → 应接近 VS 电压
     以上都对但这里没电压 = L298N 模块坏了
"""
import signal
import sys
import time

sys.path.insert(0, "/usr/lib/python3/dist-packages")
sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")

import gpiod

PINS = [(91, 'PWM_L/ENA (pin32)', 1),
        (61, 'INA_L/IN1 (pin18)', 1),
        (62, 'INB_L/IN2 (pin31)', 0),
        (63, 'PWM_R/ENB (pin36)', 1),
        (60, 'INA_R/IN3 (pin37)', 1),
        (124, 'INB_R/IN4 (pin33)', 0)]

_stop = False


def _on_signal(signum, _frame):
    global _stop
    _stop = True


for _s in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
    try:
        signal.signal(_s, _on_signal)
    except (ValueError, OSError):
        pass

chip = gpiod.Chip('gpiochip0')
lines = []
try:
    for off, name, val in PINS:
        ln = chip.get_line(off)
        ln.request(consumer='pin_hold', type=gpiod.LINE_REQ_DIR_OUT)
        ln.set_value(val)
        lines.append(ln)

    print("已钉死下列电平(稳定直流,适合万用表):")
    for (off, name, val), ln in zip(PINS, lines):
        back = ln.get_value()
        flag = 'OK' if back == val else '<<< 回读不符!'
        print("  line %3d  %-20s = %d   回读 %d  %s" % (off, name, val, back, flag))
    print("\n保持中,按 Ctrl-C 退出。轮子应架空。")

    while not _stop:
        time.sleep(0.2)
finally:
    for ln in lines:
        try:
            ln.set_value(0)
        except Exception:
            pass
    try:
        for ln in lines:
            ln.release()
    except Exception:
        pass
    print("\n所有脚已拉低并释放。")
