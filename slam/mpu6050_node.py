#!/usr/bin/env python3
"""GY-521 reader: /imu/data_raw in sensor axes, SI units, no estimated orientation."""
import argparse
import ctypes
import fcntl
import math
import os
import struct
import time
import sys
for directory in ('/usr/local/lib/python3.10/dist-packages', '/usr/lib/python3/dist-packages'):
    if os.path.isdir(directory) and directory not in sys.path:
        sys.path.insert(0, directory)


class Message(ctypes.Structure):
    _fields_ = [('addr', ctypes.c_uint16), ('flags', ctypes.c_uint16),
                ('length', ctypes.c_uint16), ('buf', ctypes.POINTER(ctypes.c_uint8))]


class Transfer(ctypes.Structure):
    _fields_ = [('messages', ctypes.POINTER(Message)), ('count', ctypes.c_uint32)]


class MPU6050:
    def __init__(self, device='/dev/i2c-1', address=0x68):
        self.fd = os.open(device, os.O_RDWR)
        self.address = address
        try:
            if self.read(0x75, 1) != b'\x68':
                raise ValueError('WHO_AM_I is not MPU6050 0x68')
        except Exception:
            self.close()
            raise

    def close(self):
        os.close(self.fd)

    def read(self, register, length):
        reg = (ctypes.c_uint8*1)(register)
        buf = (ctypes.c_uint8*length)()
        messages = (Message*2)(Message(self.address, 0, 1, reg), Message(self.address, 1, length, buf))
        result = fcntl.ioctl(self.fd, 0x707, Transfer(messages, 2))
        if result != 2:
            raise OSError('Incomplete I2C read')
        return bytes(buf)

    def write(self, register, value):
        buf = (ctypes.c_uint8*2)(register, value)
        messages = (Message*1)(Message(self.address, 0, 2, buf))
        if fcntl.ioctl(self.fd, 0x707, Transfer(messages, 1)) != 1:
            raise OSError('Incomplete I2C write')

    def configure(self):
        # PLL clock, all axes awake; 44Hz DLPF, 100Hz output, +/-250dps, +/-2g.
        for reg, value in ((0x6B, 1), (0x6C, 0), (0x1A, 3), (0x19, 9), (0x1B, 0), (0x1C, 0)):
            self.write(reg, value)
        time.sleep(.15)
        for reg, value in ((0x6B, 1), (0x6C, 0), (0x1A, 3), (0x19, 9), (0x1B, 0), (0x1C, 0)):
            if self.read(reg, 1)[0] != value:
                raise ValueError(f'Configuration readback failed at {reg:#x}')

    def sample(self):
        values = struct.unpack('>7h', self.read(0x3B, 14))
        accel = [v*9.80665/16384. for v in values[:3]]
        gyro = [v*math.pi/(180.*131.) for v in values[4:]]
        return accel+gyro


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='Read identity only, without configuring sensor')
    args, ros_args = parser.parse_known_args()
    if args.check:
        sensor = MPU6050()
        sensor.close()
        print('MPU6050 WHO_AM_I=0x68 on /dev/i2c-1 address 0x68')
        return
    import numpy as np
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Imu
    from gyro_core import calibrate

    class Reader(Node):
        def __init__(self):
            super().__init__('mpu6050')
            self.sensor = None
            self.bias = None
            self.samples = []
            self.last = 0.
            self.retry = 0.
            self.failures = 0
            self.pub = self.create_publisher(Imu, '/imu/data_raw', 20)
            self.timer = self.create_timer(.01, self.tick)
            self.get_logger().info('Keep chassis stationary for 3s gyro calibration')

        def tick(self):
            now = time.monotonic()
            if now < self.retry:
                return
            try:
                if self.sensor is None:
                    self.sensor = MPU6050()
                    self.sensor.configure()
                sample = self.sensor.sample()
                self.failures = 0
                if self.bias is None:
                    if self.last and now-self.last > .15:
                        self.samples.clear()
                    self.last = now
                    self.samples.append(sample)
                    if len(self.samples) < 300:
                        return
                    try:
                        self.bias, up = calibrate(self.samples)
                    except ValueError as exc:
                        self.samples.clear()
                        self.get_logger().warning(str(exc))
                        return
                    self.get_logger().info(f'Calibration ready bias_rad_s={self.bias.tolist()} up={up.tolist()}')
                msg = Imu()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.orientation_covariance[0] = -1.
                for i, axis in enumerate(('x', 'y', 'z')):
                    setattr(msg.linear_acceleration, axis, float(sample[i]))
                    setattr(msg.angular_velocity, axis, float(sample[i+3]-self.bias[i]))
                msg.angular_velocity_covariance = [.0001, 0., 0., 0., .0001, 0., 0., 0., .0001]
                msg.linear_acceleration_covariance = [.04, 0., 0., 0., .04, 0., 0., 0., .04]
                self.pub.publish(msg)
            except (OSError, ValueError) as exc:
                self.get_logger().error(f'IMU unavailable; laser fallback: {exc}')
                if self.sensor is not None:
                    self.sensor.close()
                self.sensor = None
                # Retain a valid bias across transient I/O errors; never recalibrate in motion.
                self.failures += 1
                # Retry a transient bus fault promptly; repeated faults back
                # off. No cached samples are published, bias stays unchanged.
                self.retry = now+min(2., .02*2**min(self.failures-1,7))

    rclpy.init(args=ros_args)
    node = Reader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.sensor is not None:
            node.sensor.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
