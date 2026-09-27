#!/usr/bin/env python3
"""编码器接线测试 —— 手动转轮子,看 4 路输入有没有边沿。

不驱动电机,零风险。用于在通电试电机之前,先确认编码器接线正确。

用法:
    /usr/bin/python3 ~/enc_test.py          # 默认 20 秒
    /usr/bin/python3 ~/enc_test.py 30       # 自定义秒数

判读:
  * 手转左轮 → ENC_L_A / ENC_L_B 计数应明显增长,右轮两路应几乎不动
  * 手转右轮 → ENC_R_A / ENC_R_B 增长
  * 某一路恒为 0 → 该线没接好 / 编码器没供电
  * 同一轮 A、B 计数差很多 → 其中一路接触不良
"""
import sys
import time

sys.path.insert(0, "/usr/lib/python3/dist-packages")
sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")

import gpiod

# 复用 motor_hw 的事件排空逻辑 —— 本机 libgpiod 的事件 API 很反直觉
# (sec 必须整数、事件不从 bulk 读、event_read 空队列会阻塞)。
# 测试脚本必须走和驱动完全一样的代码路径,否则测了个寂寞。
from motor_hw import drain_events

ENC = [(0, 'ENC_L_A (pin7)'), (12, 'ENC_L_B (pin8)'),
       (5, 'ENC_R_A (pin11)'), (1, 'ENC_R_B (pin10)')]
BIAS = gpiod.LINE_REQ_FLAG_BIAS_PULL_UP

dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0

chip = gpiod.Chip('gpiochip0')
bulk = chip.get_lines([l for l, _ in ENC])
bulk.request(consumer='enc_test', type=gpiod.LINE_REQ_EV_BOTH_EDGES, flags=BIAS)

counts = [0] * len(ENC)
print("采样中,%.0f 秒。现在用手分别转动左右轮。" % dur)
print("(四路全不动 = 编码器没供电,或线没接)\n")

IDX = {l: i for i, (l, _) in enumerate(ENC)}


def on_edge(off, _is_rising):
    counts[IDX[off]] += 1


t0 = time.monotonic()
last = t0
while time.monotonic() - t0 < dur:
    if bulk.event_wait(0, 200000000):
        drain_events(bulk, on_edge)
    now = time.monotonic()
    if now - last >= 3.0:
        last = now
        print("  t=%4.1fs  %s" % (
            now - t0,
            "  ".join("%s=%d" % (n.split()[0], c) for (_, n), c in zip(ENC, counts))))

bulk.release()

print("\n=== 最终计数 (边沿数) ===")
for (line, name), c in zip(ENC, counts):
    print("  %-18s line %3d : %6d  %s" % (
        name, line, c, 'OK' if c > 0 else '<<< 无信号!'))

l_ok = counts[0] > 0 and counts[1] > 0
r_ok = counts[2] > 0 and counts[3] > 0
print("\n左轮 A+B: %s    右轮 A+B: %s" % (
    'OK' if l_ok else 'FAIL', 'OK' if r_ok else 'FAIL'))
print("\n注意: 本测试只证明编码器有信号。A/B 相位差是否 90 度、方向正负,")
print("需下一步通电转轮子时由 motor_driver 的 enc_sign_* 判定。")
sys.exit(0 if (l_ok or r_ok) else 1)
