#!/usr/bin/env python3
"""Wheel odometry + PID motor driver — two-wheel differential robot.

Subscribes /cmd_vel (geometry_msgs/Twist), drives a dual H-bridge via libgpiod
(software PWM + direction pins), reads GMR quadrature encoders for closed-loop
speed control, and publishes wheel odometry (/odom + odom→base_link TF).

Replaces laser_odom.py as the odometry source when wheel encoders are present.
Run headless against FakeMotorHW with use_mock_hw:=true (see motor_params.yaml).

Safety model:
  * motors are disarmed on startup (arm_on_start=false);
  * /cmd_vel watchdog: no command for cmd_vel_timeout seconds → stop + disarm;
  * direction reversal first drops duty to 0 for a dead-band (no H-bridge shoot-through).
"""

import math
import os
import sys
import threading
import time

# AidLux: ROS2 env overrides PYTHONPATH, libgpiod is in system site-packages
for site_pkg in [
    "/usr/local/lib/python3.10/dist-packages",
    "/usr/lib/python3/dist-packages",
]:
    if site_pkg not in sys.path and os.path.isdir(site_pkg):
        sys.path.insert(0, site_pkg)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
from tf2_ros import TransformBroadcaster

from motor_control import Pid, map_channels, twist_to_wheel_speeds, wheel_duties
from motor_hw import GpioMotorHW, FakeMotorHW


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # ---------- parameters ----------
        p = self._declare_params()
        self._wheel_base = p['wheel_base']
        self._counts_per_meter = p['counts_per_meter']
        self._flip_l = p['flip_left']
        self._flip_r = p['flip_right']
        self._swap_lr = bool(p['swap_lr'])
        self._enc_sign_l = p['enc_sign_left']
        self._enc_sign_r = p['enc_sign_right']
        self._max_lin = p['max_linear_speed']
        self._max_ang = p['max_angular_speed']
        self._max_duty = p['max_duty']
        self._deadband = p['pid_deadband']
        self._max_dduty = p['max_dduty_per_s']
        self._ff = p['pid_ff']
        self._use_encoders = p['use_encoders']
        self._cmd_timeout = p['cmd_vel_timeout']
        self._control_hz = p['control_hz']
        self._odom_hz = p['odom_hz']
        self._arm_on_start = p['arm_on_start']
        self._odom_frame = p['odom_frame']
        self._base_frame = p['base_frame']

        self._pid_l = Pid(p['pid_kp'], p['pid_ki'], p['pid_kd'],
                          p['pid_i_limit'], self._max_duty)
        self._pid_r = Pid(p['pid_kp'], p['pid_ki'], p['pid_kd'],
                          p['pid_i_limit'], self._max_duty)

        # ---------- hardware ----------
        if p['use_mock_hw']:
            self.get_logger().warn('use_mock_hw=true → FakeMotorHW (no GPIO)')
            self.hw = FakeMotorHW(
                max_linear_speed=self._max_lin,
                counts_per_meter=self._counts_per_meter,
                wheel_base=self._wheel_base,
                flip_l=self._flip_l, flip_r=self._flip_r,
                feedback=p['mock_encoder_feedback'],
                logger=self.get_logger())
        else:
            self.hw = GpioMotorHW(
                chip_name=p['gpiochip'],
                pwm_l=p['pwm_l_line'], ina_l=p['ina_l_line'], inb_l=p['inb_l_line'],
                pwm_r=p['pwm_r_line'], ina_r=p['ina_r_line'], inb_r=p['inb_r_line'],
                enc_l_a=p['enc_l_a'], enc_l_b=p['enc_l_b'],
                enc_r_a=p['enc_r_a'], enc_r_b=p['enc_r_b'],
                pwm_freq=p['pwm_freq'],
                enable_encoders=self._use_encoders,
                logger=self.get_logger())
            self.get_logger().info(
                f'GpioMotorHW on {p["gpiochip"]} '
                f'(pwm L{p["pwm_l_line"]}/R{p["pwm_r_line"]}, '
                f'ina L{p["ina_l_line"]}/R{p["ina_r_line"]}, '
                f'inb L{p["inb_l_line"]}/R{p["inb_r_line"]}, '
                f'enc L({p["enc_l_a"]},{p["enc_l_b"]}) R({p["enc_r_a"]},{p["enc_r_b"]}))')
        self.hw.start()

        # ---------- state ----------
        self._target_v = 0.0
        self._target_w = 0.0
        self._armed = bool(self._arm_on_start)
        self._last_cmd_ts = time.monotonic()
        self._last_enc = (0, 0)
        self._last_speed_ts = time.monotonic()
        self._last_odom_ts = time.monotonic()
        self._v_l_meas = 0.0
        self._v_r_meas = 0.0
        self._dir_l = True
        self._dir_r = True
        self._duty_l = 0.0
        self._duty_r = 0.0
        # pose
        self._x = 0.0
        self._y = 0.0
        self._th = 0.0

        # ---------- ROS ----------
        self._cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self._arm_sub = self.create_subscription(
            Bool, '/motor_arm', self._on_arm, 10)
        self._enable_srv = self.create_service(
            SetBool, 'motor_enable', self._on_enable)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self._debug_pub = self.create_publisher(
            Twist, '/wheel_speed', 10)
        self._tf = TransformBroadcaster(self)

        # timers
        self.create_timer(1.0 / self._control_hz, self._control_tick)
        self.create_timer(1.0 / self._odom_hz, self._odom_tick)

        # watchdog
        self._watchdog_stop = threading.Event()
        threading.Thread(target=self._watchdog_loop, daemon=True).start()

        # publish initial TF so slam_toolbox has odom→base_link immediately
        self._publish_odom(self.get_clock().now().to_msg())

        self.get_logger().info(
            f'Motor driver ready. armed={self._armed}  '
            f'encoders={"on" if self._use_encoders else "OFF (open-loop)"}  '
            f'(call motor_enable or publish /motor_arm to enable)')

    # ------------------------------------------------------------------ helpers
    def _declare_params(self):
        d = {}
        names = [
            'wheel_base', 'counts_per_meter', 'flip_left', 'flip_right', 'swap_lr',
            'enc_sign_left', 'enc_sign_right', 'max_linear_speed',
            'max_angular_speed', 'max_duty', 'pid_deadband', 'max_dduty_per_s',
            'pid_ff', 'pid_kp', 'pid_ki', 'pid_kd', 'pid_i_limit',
            'cmd_vel_timeout', 'control_hz', 'odom_hz', 'arm_on_start',
            'odom_frame', 'base_frame', 'use_mock_hw', 'use_encoders', 'gpiochip',
            'mock_encoder_feedback',
            'pwm_freq', 'pwm_l_line', 'ina_l_line', 'inb_l_line',
            'pwm_r_line', 'ina_r_line', 'inb_r_line',
            'enc_l_a', 'enc_l_b', 'enc_r_a', 'enc_r_b',
        ]
        for n in names:
            self.declare_parameter(n)
            d[n] = self.get_parameter(n).value
        return d

    def _set_armed(self, on):
        self._armed = bool(on)
        if not on:
            self._target_v = 0.0
            self._target_w = 0.0
            self._pid_l.reset()
            self._pid_r.reset()
            self._duty_l = self._duty_r = 0.0
            self.hw.set_duty(0.0, 0.0)
            self.hw.set_direction(True, True)
        self.get_logger().info(f'motors {"ARMED" if on else "disarmed"}')

    # ------------------------------------------------------------------ ROS callbacks
    def _on_cmd_vel(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        v = max(-self._max_lin, min(self._max_lin, v))
        w = max(-self._max_ang, min(self._max_ang, w))
        self._target_v = v
        self._target_w = w
        self._last_cmd_ts = time.monotonic()

    def _on_arm(self, msg):
        self._set_armed(bool(msg.data))

    def _on_enable(self, req, resp):
        self._set_armed(bool(req.data))
        resp.success = True
        resp.message = 'armed' if self._armed else 'disarmed'
        return resp

    def _watchdog_loop(self):
        while not self._watchdog_stop.is_set():
            if self._armed and (time.monotonic() - self._last_cmd_ts) > self._cmd_timeout:
                self.get_logger().warn(
                    f'cmd_vel timeout ({self._cmd_timeout:.1f}s) → motors stopped & disarmed')
                self._set_armed(False)
            time.sleep(0.1)

    # ------------------------------------------------------------------ control
    def _read_wheel_speeds(self):
        c_l, c_r = self.hw.read_encoders()
        now = time.monotonic()
        dt = now - self._last_speed_ts
        self._last_speed_ts = now
        d_l = c_l - self._last_enc[0]
        d_r = c_r - self._last_enc[1]
        self._last_enc = (c_l, c_r)
        if dt <= 1e-6:
            return 0.0, 0.0, 0.0
        v_l = self._enc_sign_l * d_l / self._counts_per_meter / dt
        v_r = self._enc_sign_r * d_r / self._counts_per_meter / dt
        # 返回「通道系」速度:通道 L = 驱动 L 组 H 桥的那个轮子。编码器是跟电机
        # 走的,所以 c_l 天然就是通道 L 的轮速 —— 这里**不**过 swap_lr。
        self._v_l_meas = v_l
        self._v_r_meas = v_r
        return v_l, v_r, dt

    def _dir_from_speed(self, v, flip):
        s = -1.0 if flip else 1.0
        return (s * v) >= 0.0

    def _control_tick(self):
        v_l_meas, v_r_meas, dt = self._read_wheel_speeds()

        if not self._armed:
            self._duty_l = self._duty_r = 0.0
            self.hw.set_duty(0.0, 0.0)
            return

        t_l, t_r = twist_to_wheel_speeds(
            self._target_v, self._target_w, self._wheel_base)
        # 从这里往下就换成「通道系」了:下面的 wheel_duties / flip_* / duty /
        # set_duty 全部按 GPIO 通道(L 组 / R 组)定义,不再按物理左右轮。
        t_l, t_r = map_channels(t_l, t_r, self._swap_lr)

        # 见 motor_control.wheel_duties —— 目标归零清积分、开环走纯前馈,这两条
        # 是本次「松手不停 / 轻推就窜」的修复。逻辑抽成纯函数是为了能单测。
        d_l, d_r = wheel_duties(
            t_l, t_r, v_l_meas, v_r_meas, dt,
            use_encoders=self._use_encoders,
            ff=self._ff, max_lin=self._max_lin,
            max_duty=self._max_duty, deadband=self._deadband,
            pid_l=self._pid_l, pid_r=self._pid_r)

        # slope limit (avoid current spikes on step commands)
        max_step = self._max_dduty / self._control_hz
        d_l = self._duty_l + max(-max_step, min(max_step, d_l - self._duty_l))
        d_r = self._duty_r + max(-max_step, min(max_step, d_r - self._duty_r))
        self._duty_l, self._duty_r = d_l, d_r

        # direction reversal: drop duty to 0 first (no H-bridge shoot-through)
        new_dir_l = self._dir_from_speed(t_l, self._flip_l)
        new_dir_r = self._dir_from_speed(t_r, self._flip_r)
        if new_dir_l != self._dir_l:
            if d_l > 0.0:
                self.hw.set_duty(0.0, 0.0)
                time.sleep(0.001)
            self._dir_l = new_dir_l
        if new_dir_r != self._dir_r:
            if d_r > 0.0:
                self.hw.set_duty(0.0, 0.0)
                time.sleep(0.001)
            self._dir_r = new_dir_r

        self.hw.set_direction(self._dir_l, self._dir_r)
        self.hw.set_duty(d_l, d_r)

        # debug
        dbg = Twist()
        dbg.linear.x = v_l_meas
        dbg.linear.y = v_r_meas
        dbg.linear.z = self._duty_l
        dbg.angular.x = self._duty_r
        dbg.angular.z = self._target_w
        self._debug_pub.publish(dbg)

    # ------------------------------------------------------------------ odometry
    def _odom_tick(self):
        now = time.monotonic()
        dt = now - self._last_odom_ts
        self._last_odom_ts = now
        # _v_*_meas 是通道系(_control_tick 用),里程计要的是物理系
        # —— map_channels 是自逆的,同一个调用换回来。
        v_l_meas, v_r_meas = map_channels(self._v_l_meas, self._v_r_meas,
                                          self._swap_lr)
        dl = v_l_meas * dt
        dr = v_r_meas * dt
        v = (dr + dl) / 2.0
        w = (dr - dl) / self._wheel_base
        self._x += v * math.cos(self._th)
        self._y += v * math.sin(self._th)
        self._th += w * dt
        self._th = math.atan2(math.sin(self._th), math.cos(self._th))
        self._publish_odom(self.get_clock().now().to_msg())

    def _publish_odom(self, stamp):
        half = self._th * 0.5
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = self._x
        t.transform.translation.y = self._y
        t.transform.rotation.z = math.sin(half)
        t.transform.rotation.w = math.cos(half)
        self._tf.sendTransform(t)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = math.sin(half)
        odom.pose.pose.orientation.w = math.cos(half)
        odom.twist.twist.linear.x = (self._v_l_meas + self._v_r_meas) / 2.0
        odom.twist.twist.angular.z = (self._v_r_meas - self._v_l_meas) / self._wheel_base
        self._odom_pub.publish(odom)

    # ------------------------------------------------------------------ shutdown
    def shutdown(self):
        self._set_armed(False)
        self._watchdog_stop.set()
        try:
            self.hw.stop()
        except Exception as e:
            self.get_logger().warn(f'hw.stop(): {e}')
        self.destroy_node()


def main():
    rclpy.init()
    node = MotorDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.shutdown()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
