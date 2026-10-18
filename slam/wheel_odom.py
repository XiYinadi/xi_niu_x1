#!/usr/bin/env python3
"""Input-only wheel odometry. Never requests motor GPIO or subscribes to drive.

Native sampler owns ONLY GPIO 0/12/5/1; current pwm_proc remains motor owner.
Uncalibrated/invalid/stale data never produces /odom or odom->base_link.
--diagnostic permits raw count inspection without publishing odometry.
"""
import argparse
import json
import math
from pathlib import Path
import queue
import subprocess
import threading
import time

from wheel_odom_core import integrate, sample_fault


def read_config(path, diagnostic=False):
    import yaml
    p = yaml.safe_load(Path(path).read_text())
    if not isinstance(p, dict):
        raise ValueError('wheel odometry configuration must be a mapping')
    if not math.isfinite(float(p['wheel_track_m'])) or p['wheel_track_m'] <= 0:
        raise ValueError('wheel_track_m must be positive')
    if p['sign_left'] not in (-1, 1) or p['sign_right'] not in (-1, 1):
        raise ValueError('encoder signs must be -1 or +1')
    if not diagnostic:
        if p.get('calibrated') is not True:
            raise ValueError('编码器标定未通过：禁止启动建图里程计；先核对两侧计数和每米比例')
        for key in ('counts_per_meter_left', 'counts_per_meter_right', 'max_wheel_speed_mps'):
            if p.get(key) is None or not math.isfinite(float(p[key])) or p[key] <= 0:
                raise ValueError(f'{key} must be finite and positive')
    return p


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--params', default=str(Path(__file__).with_suffix('.yaml')))
    parser.add_argument('--sampler', default=str(Path(__file__).with_name('encoder_sampler')))
    parser.add_argument('--diagnostic', action='store_true')
    parser.add_argument('--check', action='store_true', help='Validate configuration without opening GPIO')
    args = parser.parse_args()
    try:
        p = read_config(args.params, args.diagnostic)
    except (ValueError, KeyError, TypeError, OSError) as exc:
        parser.exit(2, f'wheel_odom: {exc}\n')
    if args.check:
        return
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped
    from std_msgs.msg import String
    from tf2_ros import TransformBroadcaster
    from rclpy.duration import Duration

    rclpy.init()
    node = Node('wheel_odom')
    counts_pub = node.create_publisher(String, '/encoder_counts', 10)
    pub = node.create_publisher(Odometry, '/odom', 10) if not args.diagnostic else None
    tf = TransformBroadcaster(node) if not args.diagnostic else None
    child = subprocess.Popen([args.sampler, '0'], stdout=subprocess.PIPE, text=True, bufsize=1)
    pending = queue.Queue(maxsize=100)
    overflow = threading.Event()
    def read():
        try:
            for line in child.stdout:
                try:
                    pending.put_nowait((time.monotonic(), json.loads(line)))
                except (ValueError, queue.Full):
                    overflow.set()
                    return
        finally:
            child.stdout.close()
    threading.Thread(target=read, daemon=True).start()
    pose = (0., 0., 0.)
    prev = None
    last_seen = time.monotonic()
    last_log = 0.
    fault = None
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0)
            try:
                received, data = pending.get(timeout=.02)
            except queue.Empty:
                if child.poll() is not None or time.monotonic()-last_seen > .3:
                    raise RuntimeError('编码器采样已中断；停止发布里程计，禁止沿用旧位姿')
                continue
            last_seen = received
            counts_pub.publish(String(data=json.dumps(data)))
            if overflow.is_set() or time.monotonic()-received > .1:
                fault = '编码器采样队列积压或数据过期'
            if prev is not None:
                fault = fault or sample_fault(prev, data)
            if time.monotonic()-last_log > 2:
                node.get_logger().info(f'counts={data["counts"]} invalid={data["invalid"]} '
                                       f'diagnostic={args.diagnostic} fault={fault}')
                last_log = time.monotonic()
            if args.diagnostic:
                prev = data
                continue
            if fault:
                raise RuntimeError(fault)
            if prev is None:
                prev = data
                continue
            dt = data['elapsed']-prev['elapsed']
            if not 0 < dt < .1:
                raise RuntimeError('invalid encoder sample interval')
            dl = (data['counts'][0]-prev['counts'][0])*p['sign_left']/p['counts_per_meter_left']
            dr = (data['counts'][1]-prev['counts'][1])*p['sign_right']/p['counts_per_meter_right']
            if max(abs(dl), abs(dr))/dt > p['max_wheel_speed_mps']:
                raise RuntimeError('encoder displacement exceeds calibrated speed bound')
            pose = integrate(pose, dl, dr, p['wheel_track_m'])
            prev = data
            # Timestamp the acquisition, not the later Python/ROS callback.
            age = time.monotonic() - data['monotonic']
            if not 0 <= age < .1:
                raise RuntimeError('encoder acquisition timestamp is stale')
            stamp = (node.get_clock().now() - Duration(nanoseconds=int(age*1e9))).to_msg()
            msg = Odometry()
            msg.header.stamp = stamp
            msg.header.frame_id = 'odom'
            msg.child_frame_id = 'base_link'
            msg.pose.pose.position.x, msg.pose.pose.position.y = pose[:2]
            msg.pose.pose.orientation.z = math.sin(pose[2]/2)
            msg.pose.pose.orientation.w = math.cos(pose[2]/2)
            msg.twist.twist.linear.x = (dl+dr)/(2*dt)
            msg.twist.twist.angular.z = (dr-dl)/(p['wheel_track_m']*dt)
            for i in (0, 7, 35):
                msg.pose.covariance[i] = .02
                msg.twist.covariance[i] = .05
            for i in (14, 21, 28):
                msg.pose.covariance[i] = 1e6
                msg.twist.covariance[i] = 1e6
            transform = TransformStamped()
            transform.header = msg.header
            transform.child_frame_id = msg.child_frame_id
            transform.transform.translation.x, transform.transform.translation.y = pose[:2]
            transform.transform.rotation = msg.pose.pose.orientation
            pub.publish(msg)
            tf.sendTransform(transform)
    finally:
        child.terminate()
        try:
            child.wait(timeout=2)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
