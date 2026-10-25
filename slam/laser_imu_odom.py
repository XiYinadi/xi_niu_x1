#!/usr/bin/env python3
"""Laser translation + gyro yaw prior. Publishes the sole /odom and odom->base_link."""
import math
import argparse
import time
import os
import sys
for directory in ('/usr/local/lib/python3.10/dist-packages', '/usr/lib/python3/dist-packages'):
    if os.path.isdir(directory) and directory not in sys.path:
        sys.path.insert(0, directory)
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from laser_odom_core import scan_to_points_normals, compose_pose
from gyro_core import GyroHistory, KeyframeMotion, MotionDiscontinuity, MotionRejected, align
from laser_mount import load_mount


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class LaserImuOdom(Node):
    def __init__(self):
        super().__init__('laser_imu_odom')
        self.history = GyroHistory()
        self.mount = load_mount()
        self.up = None
        self.reference = None
        self.motion = None
        self.pose = (0., 0., 0.)
        self.mode = None
        self.last_report = 0.
        self.good = self.failed = self.assisted = 0
        self.odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)
        self.create_subscription(Imu, '/imu/data_raw', self.on_imu, qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        self.get_logger().info('Encoder-free laser/gyro odometry; IMU missing => laser-only with warning')

    def on_imu(self, msg):
        acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        if self.up is None:
            magnitude = np.linalg.norm(acceleration)
            if not np.isfinite(magnitude) or not 8. < magnitude < 11.5:
                return
            self.up = acceleration/magnitude
            self.get_logger().info(f'Fixed sensor up axis from calibrated stationary gravity: {self.up.tolist()}')
        rate = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        self.history.add(stamp_seconds(msg.header.stamp), float(rate @ self.up))

    def on_scan(self, msg):
        if getattr(getattr(self,'motion',None),'fault',None) is not None:
            return  # Latched: publish no fresh pose/TF until a stationary restart.
        points, normals = scan_to_points_normals(msg, max_range=12.)
        if points is None:
            return
        # Registration must estimate motion in base_link axes. The mounting
        # transform also includes lever-arm effects when XY is measured later.
        c,s=math.cos(self.mount['yaw_rad']),math.sin(self.mount['yaw_rad'])
        rotation=np.array([[c,-s],[s,c]])
        points=points @ rotation.T + [self.mount['x_m'],self.mount['y_m']]
        normals=normals @ rotation.T
        stamp = stamp_seconds(msg.header.stamp)
        if self.reference is None:
            self.reference = (points, normals, stamp)
            self.motion = KeyframeMotion(stamp,self.pose)
            self.publish(msg.header.stamp, (0., 0., 0.))
            return
        target, target_normals, previous = self.reference
        dt = stamp-previous
        if dt <= 0.:
            return
        if stamp-self.motion.last_stamp > 1.:
            if self.good == 0:
                # A1's first startup scan can precede steady rotation by >1s.
                # No displacement has been integrated yet; establish the origin
                # from a steady scan pair instead of latching the startup gap.
                self.reference = (points, normals, stamp)
                self.motion = KeyframeMotion(stamp,self.pose)
                self.get_logger().warning('Discarding discontinuous startup scan; waiting for steady scan pair')
                return
            # Unknown displacement across a long outage: do not fabricate a pose.
            if time.monotonic()-self.last_report > 5.:
                self.last_report = time.monotonic()
                self.get_logger().error('Laser reference lost >1s; odom suspended, restart mapping at rest')
            return
        delta = self.history.delta(previous, stamp)
        mode = 'laser+gyro' if delta is not None else 'laser-only (IMU coverage missing)'
        if mode != self.mode:
            # Humble keys logging severity by callsite, so keep distinct calls.
            if delta is not None:
                self.get_logger().info('Odometry source: '+mode)
            else:
                self.get_logger().warning('Odometry source: '+mode)
            self.mode = mode
        result = align(points, target, target_normals, delta)
        if result is None:
            self.failed += 1
            if self.failed % 10 == 1:
                self.get_logger().warning('Scan match rejected; no fresh odom fabricated')
            return
        dx, dy, theta, error = result
        try:
            self.pose,velocity,advance = self.motion.update(dx,dy,theta,stamp)
        except MotionRejected as error:
            self.failed += 1
            self.get_logger().warning(str(error))
            return
        except MotionDiscontinuity as error:
            self.failed += 1
            self.get_logger().error(str(error))
            return
        if advance:
            self.reference = (points, normals, stamp)
        self.good += 1
        self.assisted += int(delta is not None)
        self.publish(msg.header.stamp, velocity)
        if time.monotonic()-self.last_report > 5.:
            self.last_report = time.monotonic()
            self.get_logger().info(f'{mode} pose={self.pose} good={self.good} gyro_frames={self.assisted} rejected={self.failed} residual={error:.4f}m')

    def publish(self, stamp, velocity):
        x, y, theta = self.pose
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(theta/2.)
        odom.pose.pose.orientation.w = math.cos(theta/2.)
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z = velocity
        odom.pose.covariance = [0.]*36
        for i in (0, 7):
            odom.pose.covariance[i] = .02
        for i in (14, 21, 28):
            odom.pose.covariance[i] = 1e6
        odom.pose.covariance[35] = .01 if self.mode == 'laser+gyro' else .05
        odom.twist.covariance = list(odom.pose.covariance)
        transform = TransformStamped()
        transform.header = odom.header
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x, transform.transform.translation.y = x, y
        transform.transform.rotation = odom.pose.pose.orientation
        self.tf.sendTransform(transform)
        self.odom.publish(odom)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.parse_known_args()
    rclpy.init()
    node = LaserImuOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
