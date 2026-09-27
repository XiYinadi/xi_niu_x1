#!/usr/bin/env python3
"""Direct rclpy probe — no ros2 CLI, no daemon (bypasses broken AidLux daemon).

Answers with DDS ground truth:
  1. what services/topics the motor_driver node actually registered
  2. does arm via /motor_enable work
  3. does /odom advance while continuously streaming /cmd_vel
  4. does the watchdog freeze motion 0.5s after the cmd_vel stream stops
"""
import sys
import time

sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")
sys.path.insert(0, "/usr/lib/python3/dist-packages")

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

SERVICE = '/motor_enable'          # relative name → resolved against node namespace
MAX_LIN = 0.3


def stream(node, pub, tw, seconds):
    """Continuously publish cmd_vel so the 0.5s watchdog never fires."""
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        pub.publish(tw)
        rclpy.spin_once(node, timeout_sec=0.02)


def main():
    rclpy.init()
    n = rclpy.create_node('probe')
    time.sleep(2.0)

    print("=== 1. graph (rclpy direct, no daemon) ===")
    print("nodes:   ", n.get_node_names())
    print("services:", sorted(s for s, _ in n.get_service_names_and_types()))
    print("topics:  ", sorted(t for t, _ in n.get_topic_names_and_types()))

    odom = []
    n.create_subscription(Odometry, '/odom', lambda m: odom.append(m), 10)
    cmd_pub = n.create_publisher(Twist, '/cmd_vel', 10)
    time.sleep(1.0)

    # --- 2. arm via service ---
    cli = n.create_client(SetBool, SERVICE)
    ok = cli.wait_for_service(timeout_sec=4.0)
    print("\n=== 2. arm via service ===")
    print("wait_for_service(%s):" % SERVICE, ok)
    if not ok:
        print("service missing → ABORT")
        n.destroy_node()
        rclpy.shutdown()
        return
    req = SetBool.Request()
    req.data = True
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(n, fut, timeout_sec=4.0)
    if fut.done():
        resp = fut.result()
        print("response success=%s message=%s" % (resp.success, resp.message))
        if not resp.success:
            print("arm rejected → ABORT")
            n.destroy_node()
            rclpy.shutdown()
            return
    else:
        print("service call timed out → ABORT")
        n.destroy_node()
        rclpy.shutdown()
        return

    # --- 3. stream cmd_vel, sample odom at 1s and 3s ---
    print("\n=== 3. stream cmd_vel 0.1 m/s for 3s (watchdog stays alive) ===")
    tw = Twist()
    tw.linear.x = 0.1
    stream(n, cmd_pub, tw, 1.0)
    x1 = odom[-1].pose.pose.position.x if odom else None
    print("odom x after 1s: %.4f" % x1)
    stream(n, cmd_pub, tw, 2.0)
    x2 = odom[-1].pose.pose.position.x if odom else None
    print("odom x after 3s: %.4f  (Δ=%.4f, expect ≈0.3 minus PID ramp)" % (x2, (x2 or 0) - (x1 or 0)))
    print("odom samples: %d" % len(odom))

    # --- 4. stop cmd_vel → watchdog should freeze odom in ~0.5s ---
    print("\n=== 4. stop cmd_vel → watchdog 0.5s ===")
    x_before = odom[-1].pose.pose.position.x if odom else None
    # keep spinning so we read fresh post-watchdog odom samples
    for _ in range(60):
        rclpy.spin_once(n, timeout_sec=0.02)   # 1.2s of fresh samples
    x3 = odom[-1].pose.pose.position.x if odom else None
    if len(odom) >= 2:
        dx = abs(odom[-1].pose.pose.position.x - odom[-2].pose.pose.position.x)
        print("odom x before stop: %.4f → after: %.4f (moved %.4f more during 0.5s window)"
              % (x_before, x3, (x3 or 0) - (x_before or 0)))
        print("Δ/20ms at end: %.5f (frozen if ~0)" % dx)
    else:
        print("odom x after stop: %s" % x3)

    n.destroy_node()
    rclpy.shutdown()
    print("DONE")


if __name__ == '__main__':
    main()
