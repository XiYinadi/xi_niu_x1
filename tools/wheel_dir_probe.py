#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""单通道方向探针：先停 pwm_proc，再逐通道给 A/B 两种方向位 + 15% 占空比。

  pkill -f 'python3 .*pwm_proc.py'
  python3 /home/aidlux/robot/tools/wheel_dir_probe.py

顺序（每段 2 秒，段间停 1 秒）：
  右轮通道 L：A=INA1/INB0 → B=INA0/INB1
  左轮通道 R：A=INA1/INB0 → B=INA0/INB1
观察每一段轮子转向（正=车头方向），据此判断哪一格是前进。
"""
import subprocess
import sys
import time

try:
    import gpiod
except ImportError:
    sys.exit('需要 python3-gpiod')

PWM_L, INA_L, INB_L = 91, 61, 62
PWM_R, INA_R, INB_R = 63, 60, 124
DUTY = 0.15
PERIOD = 0.001


def running():
    out = subprocess.run(['pgrep', '-af', 'python3 .*pwm_proc\\.py'],
                         capture_output=True, text=True).stdout
    return [ln for ln in out.splitlines() if 'pgrep' not in ln]


if running():
    sys.exit('pwm_proc 还在跑，先执行：pkill -f "python3 .*pwm_proc.py"')

chip = gpiod.Chip('gpiochip0')
lines = {}
for name, num in (('pwm_l', PWM_L), ('ina_l', INA_L), ('inb_l', INB_L),
                  ('pwm_r', PWM_R), ('ina_r', INA_R), ('inb_r', INB_R)):
    ln = chip.get_line(num)
    ln.request(consumer='wheel_dir_probe', type=gpiod.LINE_REQ_DIR_OUT)
    ln.set_value(0)
    lines[name] = ln


def spin(pwm):
    lines[pwm].set_value(1)
    time.sleep(PERIOD * DUTY)
    lines[pwm].set_value(0)
    time.sleep(PERIOD * (1 - DUTY))


def seg(wheel, ina, inb, pwm):
    for state, tag in ((True, 'A INA=1/INB=0'), (False, 'B INA=0/INB=1')):
        lines[ina].set_value(1 if state else 0)
        lines[inb].set_value(0 if state else 1)
        print('%s %s  转 2 秒…' % (wheel, tag), flush=True)
        end = time.time() + 2.0
        while time.time() < end:
            spin(pwm)
        lines[pwm].set_value(0)
        print('  停', flush=True)
        time.sleep(1.0)


try:
    print('开始（请观察每段轮子转向；正=车头方向）', flush=True)
    seg('右轮(通道L)', INA_L, INB_L, PWM_L)
    seg('左轮(通道R)', INA_R, INB_R, PWM_R)
finally:
    for ln in lines.values():
        try:
            ln.set_value(0)
            ln.release()
        except Exception:
            pass
    print('结束，GPIO 已释放', flush=True)
