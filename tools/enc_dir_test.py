#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用编码器判断左右轮的实际转向（pwm_proc 未申请编码器线，可直接读）。

前进 3 秒、倒车 3 秒，比较左右轮的计数符号：
  - 前进时两轮计数应同为 + 或同为 -
  - 若左轮与右轮符号相反，说明左轮电气方向反了
"""
import json
import socket
import sys
import time

sys.path.insert(0, '/home/aidlux/robot/motor')
import gpiod
from motor_hw import QuadDecoder

chip = gpiod.Chip('gpiochip0')
left = QuadDecoder(chip, 5, 1, 'enc_left')     # 左轮：enc_r_a=5, enc_r_b=1
right = QuadDecoder(chip, 0, 12, 'enc_right')  # 右轮：enc_l_a=0, enc_l_b=12
time.sleep(0.6)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(l, seconds):
    end = time.time() + seconds
    while time.time() < end:
        sock.sendto(json.dumps({"l": l, "a": 0.0, "armed": True}).encode(), ("127.0.0.1", 5813))
        time.sleep(0.02)
    for _ in range(6):
        sock.sendto(json.dumps({"l": 0.0, "a": 0.0, "armed": False}).encode(), ("127.0.0.1", 5813))
        time.sleep(0.02)


def delta():
    return left.count, right.count


try:
    l0, r0 = delta()
    print("前进 3 秒…", flush=True)
    send(0.10, 3.0)
    time.sleep(0.5)
    l1, r1 = delta()
    print("前进: 左轮 %+d 右轮 %+d" % (l1 - l0, r1 - r0), flush=True)

    l2, r2 = delta()
    print("倒车 3 秒…", flush=True)
    send(-0.10, 3.0)
    time.sleep(0.5)
    l3, r3 = delta()
    print("倒车: 左轮 %+d 右轮 %+d" % (l3 - l2, r3 - r2), flush=True)
finally:
    try:
        left._stop.set()
        right._stop.set()
    except Exception:
        pass
    print("结束", flush=True)
