#!/usr/bin/env python3
"""测 /wheel_speed(即 motor_driver._control_tick)的**真实**周期分布。

为什么要单独写一个:cmd_log.py 每条消息都 print+flush,在负载高的板子上自己
就会丢消息 —— 拿它去判「控制环是不是被饿死」会把探针的瓶颈算到驱动头上。
这个探针**只往列表里追加时间戳**,每 20 秒才输出一次统计。

判读:
  * 中位间隔 ≈ 20ms(control_hz=50)          → 控制环正常,之前的慢是 cmd_log 造成的
  * 中位间隔几十上百毫秒 / 出现大尖峰        → 回调确实被饿死(PWM 忙等抢 GIL)
  * 松手后 duty 归零耗时 > 100ms             → 同上看 gap 的 p99

用法: python3 rate_probe.py [时长秒数]   驾驶期间跑,结束后自动出报告。
"""
import sys
import time

sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")
sys.path.insert(0, "/usr/lib/python3/dist-packages")

import rclpy
from geometry_msgs.msg import Twist

DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0


def pct(xs, p):
    if not xs:
        return float('nan')
    return xs[min(len(xs) - 1, int(len(xs) * p))]


def main():
    rclpy.init()
    n = rclpy.create_node('rate_probe')

    hits = []          # /wheel_speed 到达时刻
    cmd_hits = []
    last = {'duty': None}

    def on_duty(m):
        hits.append(time.monotonic())
        last['duty'] = (m.linear.z, m.angular.x)

    n.create_subscription(Twist, '/wheel_speed', on_duty, 500)
    n.create_subscription(Twist, '/cmd_vel',
                          lambda m: cmd_hits.append(time.monotonic()), 500)

    print("采集中,持续 %.0f 秒 —— 现在去驾驶。" % DUR, flush=True)
    t0 = time.monotonic()
    while time.monotonic() - t0 < DUR:
        rclpy.spin_once(n, timeout_sec=0.02)

    print("\n════ /wheel_speed (control_tick) 统计 ════")
    print("  总条数 %d   时长 %.1fs   平均 %.1f Hz"
          % (len(hits), DUR, len(hits) / DUR))
    if len(hits) > 2:
        gaps = sorted(hits[i + 1] - hits[i] for i in range(len(hits) - 1))
        print("  间隔: 中位 %.1fms  p90 %.1fms  p99 %.1fms  最大 %.1fms"
              % (1000 * pct(gaps, .5), 1000 * pct(gaps, .9),
                 1000 * pct(gaps, .99), 1000 * gaps[-1]))
        big = [g for g in gaps if g > 0.05]
        # 注意是 len(big):原来这里把**列表**直接塞给 %d,一跑到这行就
        # TypeError 崩掉 —— 最关键的空档计数反而从来没打出来过。
        print("  >50ms 的空档: %d 次(占 %.1f%%)"
              % (len(big), 100.0 * len(big) / len(gaps)))
        if big:
            print("    最长的 5 个: %s" % ", ".join("%.0fms" % (1000 * g) for g in sorted(big)[-5:]))
    else:
        print("  !! 一条都没收到 —— 电机没 arm(/wheel_speed 只在 armed 时发布)")

    if cmd_hits:
        cg = sorted(cmd_hits[i + 1] - cmd_hits[i] for i in range(len(cmd_hits) - 1))
        print("\n════ /cmd_vel (网页) ════")
        print("  总条数 %d   平均 %.1f Hz   间隔中位 %.1fms"
              % (len(cmd_hits), len(cmd_hits) / DUR, 1000 * pct(cg, .5)))

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
