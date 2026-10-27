#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""原地旋转固定角度（陀螺闭环）。

用法：
  python3 rotate_test.py                 # 默认左转 90°
  python3 rotate_test.py -90             # 右转 90°
  python3 rotate_test.py 180 0.12        # 角度 占空比（0.05~0.25）

输出每 15° 一次进度，最后给出「陀螺积分角度」与「结束原因」。
安全：|角度|<=360；结束时自动 disarm。
"""
import argparse
import json
import math
import socket
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

PWM = ("127.0.0.1", 5813)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("angle", nargs="?", type=float, default=90.0)
    parser.add_argument("rate", nargs="?", type=float, default=0.14)
    args = parser.parse_args()
    target = math.radians(args.angle)
    if abs(args.angle) > 360:
        raise SystemExit("角度必须在 ±360 度以内")
    if not 0.05 <= args.rate <= 0.50:
        raise SystemExit("占空比必须在 0.05~0.35")

    rclpy.init()
    node = Node("rotate_test")
    state = {"wz": 0.0, "stamp": 0.0}

    def on_imu(msg):
        state["wz"] = float(msg.angular_velocity.z)
        state["stamp"] = time.monotonic()

    node.create_subscription(Imu, "/imu/data_raw", on_imu, 20)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(rot, armed=True):
        sock.sendto(json.dumps({"l": 0.0, "a": 0.0, "rot": rot, "armed": armed}).encode(), PWM)

    end = time.time() + 5
    while not state["stamp"] and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if not state["stamp"]:
        raise SystemExit("没有收到 /imu/data_raw")

    sign = 1.0 if target > 0 else -1.0
    done = 0.0
    last = start = time.monotonic()
    next_report = 15.0
    reason = "到达目标"
    print("开始旋转 %.0f°（占空比 %.2f，正=左转）" % (args.angle, args.rate), flush=True)
    try:
        while True:
            rclpy.spin_once(node, timeout_sec=0.01)
            now = time.monotonic()
            dt = max(0.0, min(0.1, now - last))
            last = now
            if state["stamp"] and now - state["stamp"] < 0.3:
                done += state["wz"] * dt
            if abs(math.degrees(done)) >= next_report:
                print("  …已转 %.1f°" % math.degrees(done), flush=True)
                next_report += 15.0
            if abs(done) >= abs(target) - 0.02:
                break
            if now - start > 60:
                reason = "超时 60 秒"
                break
            send(sign * args.rate)
            time.sleep(0.01)
    finally:
        for _ in range(10):
            send(0.0, True)
            time.sleep(0.01)
        for _ in range(5):
            send(0.0, False)
            time.sleep(0.01)
        print("结束（%s）。陀螺积分角度 = %.1f°（目标 %.0f°），用时 %.1f 秒"
              % (reason, math.degrees(done), args.angle, time.monotonic() - start), flush=True)
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
