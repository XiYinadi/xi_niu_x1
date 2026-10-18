#!/usr/bin/env python3
"""Laser scan matching odometry — publishes odom->base_footprint TF.

Uses iterative closest point (ICP) between consecutive laser scans
to estimate robot motion. For robots without wheel encoders.
"""

import math
import os
import sys
import time

# AidLux: ROS2 env overrides PYTHONPATH, scipy is in system site-packages
for site_pkg in [
    "/usr/local/lib/python3.10/dist-packages",
    "/usr/lib/python3/dist-packages",
]:
    if site_pkg not in sys.path and os.path.isdir(site_pkg):
        sys.path.insert(0, site_pkg)

import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


def scan_to_points(scan_msg, max_range=5.0):
    """Convert LaserScan ranges to 2D numpy points in laser frame."""
    points = []
    angle = scan_msg.angle_min
    for r in scan_msg.ranges:
        if scan_msg.range_min < r < max_range and math.isfinite(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append([x, y])
        angle += scan_msg.angle_increment
    if len(points) < 20:
        return None
    return np.array(points, dtype=np.float64)


def icp_step(source, target, max_dist=1.0):
    """One ICP iteration: align source to target.

    Returns: (R, t, mean_error) or (None, None, float('inf')) on failure.
    """
    if source is None or target is None or len(source) < 10 or len(target) < 10:
        return None, None, float('inf')

    tree = cKDTree(target)
    dists, idxs = tree.query(source)

    mask = dists < max_dist
    if mask.sum() < 10:
        return None, None, float('inf')

    src_matched = source[mask]
    tgt_matched = target[idxs[mask]]

    src_mean = src_matched.mean(axis=0)
    tgt_mean = tgt_matched.mean(axis=0)

    src_centered = src_matched - src_mean
    tgt_centered = tgt_matched - tgt_mean

    W = src_centered.T @ tgt_centered
    U, _, Vt = np.linalg.svd(W)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    t = tgt_mean - R @ src_mean
    mean_err = dists[mask].mean()

    return R, t, mean_err


class LaserOdomNode(Node):
    def __init__(self):
        super().__init__('laser_odom')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('max_range', 5.0)
        self.declare_parameter('max_icp_dist', 1.0)
        self.declare_parameter('icp_iters', 10)
        self.declare_parameter('min_error', 0.005)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.max_range = self.get_parameter('max_range').value
        self.max_icp_dist = self.get_parameter('max_icp_dist').value
        self.icp_iters = self.get_parameter('icp_iters').value
        self.min_error = self.get_parameter('min_error').value

        self.prev_points = None
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0
        self.scan_count = 0
        self.icp_success_count = 0
        self.icp_fail_count = 0

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # Publish initial TF at origin immediately (don't wait for first scan)
        self._publish_tf(self.get_clock().now().to_msg())
        self.get_logger().info('Laser odometry ready (initial TF published at origin)')

    def _publish_tf(self, stamp):
        """Publish odom -> base_footprint TF at current pose."""
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.pose_x
        t.transform.translation.y = self.pose_y
        t.transform.translation.z = 0.0
        half_theta = self.pose_theta * 0.5
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(half_theta)
        t.transform.rotation.w = math.cos(half_theta)
        self.tf_broadcaster.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.orientation.z = math.sin(half_theta)
        odom.pose.pose.orientation.w = math.cos(half_theta)
        self.odom_pub.publish(odom)

    def scan_callback(self, scan_msg):
        self.scan_count += 1
        cur_points = scan_to_points(scan_msg, self.max_range)

        if cur_points is None:
            if self.scan_count <= 3:
                self.get_logger().warn(
                    f'Scan #{self.scan_count}: not enough valid points (< 20)')
            return

        if self.scan_count <= 3:
            self.get_logger().info(
                f'Scan #{self.scan_count}: {len(cur_points)} valid points')

        # ICP with previous scan to estimate motion
        if self.prev_points is not None and len(self.prev_points) >= 10:
            src = cur_points.copy()
            tgt = self.prev_points.copy()

            R_total = np.eye(2)
            t_total = np.zeros(2)
            prev_err = float('inf')
            converged = False

            for i in range(self.icp_iters):
                R, t, err = icp_step(src, tgt, self.max_icp_dist)
                if R is None:
                    break

                src = (R @ src.T).T + t
                t_total = R @ t_total + t
                R_total = R @ R_total

                if abs(prev_err - err) < self.min_error:
                    converged = True
                    break
                prev_err = err

            # Only update pose if ICP found a reasonable match
            dtheta = math.atan2(R_total[1, 0], R_total[0, 0])
            dx = t_total[0]
            dy = t_total[1]

            # Sanity check: robot can't move more than ~5cm/frame at walking speed
            dist = math.sqrt(dx*dx + dy*dy)
            if dist < 0.3 and abs(dtheta) < 0.3:
                self.pose_x += dx * math.cos(self.pose_theta) - dy * math.sin(self.pose_theta)
                self.pose_y += dx * math.sin(self.pose_theta) + dy * math.cos(self.pose_theta)
                self.pose_theta += dtheta
                self.pose_theta = math.atan2(
                    math.sin(self.pose_theta), math.cos(self.pose_theta))
                self.icp_success_count += 1
            else:
                self.icp_fail_count += 1

        self.prev_points = cur_points.copy()

        # The estimated pose belongs to this scan, not its delayed arrival time.
        # Using wall-clock NOW shifts the odometry relative to moving laser data.
        self._publish_tf(scan_msg.header.stamp)

        # Periodic status
        if self.scan_count % 100 == 0:
            total = self.icp_success_count + self.icp_fail_count
            success_rate = 100 * self.icp_success_count / total if total > 0 else 0
            self.get_logger().info(
                f'Pose=({self.pose_x:.2f},{self.pose_y:.2f},{math.degrees(self.pose_theta):.1f}°) '
                f'ICP OK={self.icp_success_count}/{total} ({success_rate:.0f}%) '
                f'scans={self.scan_count}')


def main():
    rclpy.init()
    node = LaserOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
