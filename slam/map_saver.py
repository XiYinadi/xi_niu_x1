#!/usr/bin/env python3
"""Save SLAM map from /map topic to PGM + YAML files (no nav2_map_server needed)."""
import sys
import math
import struct
import argparse
import numpy as np
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


def occ_to_pgm(msg: OccupancyGrid, path: Path) -> None:
    """Convert OccupancyGrid to PGM (P5 binary) image."""
    w, h = msg.info.width, msg.info.height
    data = np.array(msg.data, dtype=np.int8).reshape(h, w)

    # OccupancyGrid: 0..100 (free..occupied), -1 = unknown
    # PGM: 0=black(occupied), 254=white(free), 205=unknown
    pgm = np.full((h, w), 205, dtype=np.uint8)  # default unknown
    pgm[data >= 0] = 254 - (data[data >= 0] * 254 // 100)

    # PGM P5 header
    header = f"P5\n{w} {h}\n255\n"
    with open(path, "wb") as f:
        f.write(header.encode())
        f.write(pgm.tobytes())


def write_yaml(msg: OccupancyGrid, pgm_name: str, path: Path) -> None:
    """Write map YAML metadata."""
    info = msg.info
    yaml = f"""image: {pgm_name}
mode: trinary
resolution: {info.resolution:.6f}
origin: [{info.origin.position.x:.6f}, {info.origin.position.y:.6f}, {info.origin.orientation.z:.6f}]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
    with open(path, "w") as f:
        f.write(yaml)


class MapSaver(Node):
    def __init__(self, filename: str):
        super().__init__("map_saver")
        self.filename = filename
        self.received = False
        self.sub = self.create_subscription(
            OccupancyGrid, "/map", self.callback, 10
        )
        self.get_logger().info(f"Waiting for /map... (will save to {filename})")

    def callback(self, msg: OccupancyGrid):
        if self.received:
            return
        self.received = True

        base = Path(self.filename)
        pgm_path = base.with_suffix(".pgm")
        yaml_path = base.with_suffix(".yaml")

        occ_to_pgm(msg, pgm_path)
        write_yaml(msg, pgm_path.name, yaml_path)

        self.get_logger().info(f"Map saved: {pgm_path} + {yaml_path}")
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description="Save /map to PGM+YAML")
    parser.add_argument(
        "-f", "--file", default="my_map",
        help="Output filename prefix (default: my_map)"
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv[1:])
    rclpy.spin(MapSaver(args.file))


if __name__ == "__main__":
    main()
