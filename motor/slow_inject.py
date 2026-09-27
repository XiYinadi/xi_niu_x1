#!/usr/bin/env python3
"""按**固定低包率**发 UDP 摇杆包,专门复现「后台标签页被节流到 1Hz」那个工况。

    /usr/bin/python3 ~/slow_inject.py [间隔秒=0.4] [时长秒=30] [noarm]

用途:回归验证 `motor_control.Deadman` 的滞回。**先量再改** —— 400ms 包率下
改之前日志会刷屏 `deadman: 摇杆数据中断/恢复` 各几十次(2026-09-15 实测 61 对,
严格一次一翻),而舵机与电机走同一个门 ⇒ 舵机跟着 park↔全速回中 =「启用电机后
舵机来回抖」。改之后应当 **0 对**(跳闸一次且永不恢复 ⇒ 连边沿都不产生)。

为什么不直接用 `joy_inject.py`:它的 `--hz <= 3.3` 守卫(挡真实误用,别动)
恰好把 400ms 这个复现场景挡在门外,而且 `--hz` 是全程一个值,做不出
「正常 60Hz → 被节流 → 恢复」的时序。这里只要一个恒定间隔的发包器。

安全:l 恒 0 ⇒ s_l == s_r == 0 ⇒ duty 恒 0 ⇒ **两个轮子在任何时刻都不励磁**。
      a 恒 0 ⇒ 舵机被指令到中位(1500µs),不会往任何一侧偏。
      所以本脚本能观察的只有 deadman 判据与 `servo=` 列,**车一步都不会动** ——
      验这个判据不需要给驱动通电,也不需要架空车轮。
      收尾补发 `armed=false`(照 joy_inject.py 的做法),否则进程退出后 pwm_proc
      要等 deadman 到期才停,duty 会多转一会儿。
"""
import json
import socket
import sys
import time

PERIOD = float(sys.argv[1]) if len(sys.argv) > 1 else 0.4     # 秒
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 30.0
ARMED = (len(sys.argv) <= 3) or (sys.argv[3] != 'noarm')
ADDR = ('127.0.0.1', 5813)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send(l, a, armed):
    sock.sendto(json.dumps({"l": float(l), "a": float(a),
                            "armed": bool(armed)}).encode(), ADDR)


n = 0
t_end = time.monotonic() + SECS
while True:
    t0 = time.monotonic()
    if t0 >= t_end:
        break
    send(0.0, 0.0, ARMED)
    n += 1
    # 绝对时基 + 过载重对齐,和 pwm_proc 主循环同一套写法
    slp = t0 + PERIOD - time.monotonic()
    if slp > 0:
        time.sleep(slp)

print("[slow_inject] %d 包 / 间隔 %.0fms / armed=%s" % (n, PERIOD * 1000, ARMED),
      flush=True)
# 收尾必须补发 armed=false:否则进程退出后 pwm_proc 要等 deadman 到期才停
for _ in range(10):
    send(0.0, 0.0, False)
    time.sleep(0.01)
sock.close()
print("[slow_inject] 已补发 armed=false", flush=True)
