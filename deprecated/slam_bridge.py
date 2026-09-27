#!/usr/bin/env python3
"""ROS 2 → JSON bridge: subscribes /map and /scan, writes state to shared file."""

import json
import time
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan

STATE_FILE = "/tmp/slam_web_state.json"


class SlamBridge(Node):
    def __init__(self):
        super().__init__("slam_bridge")
        self.map_sub = self.create_subscription(
            OccupancyGrid, "/map", self.map_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, "/scan", self.scan_cb, 10)
        self.latest_map = None
        self.latest_scan_ranges = None
        self.latest_scan_angles = None
        self.get_logger().info("SlamBridge ready, writing to %s" % STATE_FILE)

        # Periodic file writer (5Hz)
        self.timer = self.create_timer(0.2, self.write_state)

    def map_cb(self, msg):
        self.latest_map = msg
        self.get_logger().debug("Map received: %dx%d" % (msg.info.width, msg.info.height))

    def scan_cb(self, msg):
        # Subsample to ~360 points
        step = max(1, len(msg.ranges) // 360)
        self.latest_scan_ranges = [msg.ranges[i] for i in range(0, len(msg.ranges), step)]
        self.latest_scan_angles = [
            msg.angle_min + i * msg.angle_increment
            for i in range(0, len(msg.ranges), step)
        ]

    def write_state(self):
        state = {"map_meta": None, "map_data": None, "scan_count": 0, "ts": time.time()}

        if self.latest_map is not None:
            m = self.latest_map
            state["map_meta"] = {
                "width": m.info.width,
                "height": m.info.height,
                "resolution": m.info.resolution,
                "origin_x": m.info.origin.position.x,
                "origin_y": m.info.origin.position.y,
            }
            state["map_data"] = list(m.data)

        if self.latest_scan_ranges is not None:
            state["scan_count"] = len(self.latest_scan_ranges)

        try:
            with open(STATE_FILE + ".tmp", "w") as f:
                json.dump(state, f)
            import os
            os.replace(STATE_FILE + ".tmp", STATE_FILE)
        except Exception as e:
            self.get_logger().error("Write error: %s" % e)


def main():
    rclpy.init()
    node = SlamBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
