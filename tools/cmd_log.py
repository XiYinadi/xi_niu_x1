#!/usr/bin/env python3
"""把 /cmd_vel 与 /wheel_speed 逐条带时间戳打到 stdout,用于定位「电机抖动」的
来源:

  * /cmd_vel   —— 网页摇杆发出来的原始指令
  * /wheel_speed —— 驱动算出来的 duty(_control_tick 里发的调试话题)

判读:
  * cmd_vel 的 linear.x / angular.z 在正负之间来回跳 → 前端(摇杆/失焦/重连)
  * cmd_vel 平稳但 duty 在跳               → 驱动内部(斜率限幅/换向保护)
  * 两个都平稳                             → 抖动在引脚层以下(接线/供电)

用 nohup 挂着跑,随时去拖摇杆,回来读日志即可 —— 不需要卡时间:
    nohup python3 ~/cmd_log.py > /tmp/cmdvel.log 2>&1 &

/wheel_speed 的字段映射见 motor_driver._control_tick 的调试发布:
    linear.z = 左轮 duty, angular.x = 右轮 duty
"""
import sys
import time

sys.path.insert(0, "/usr/local/lib/python3.10/dist-packages")
sys.path.insert(0, "/usr/lib/python3/dist-packages")

import rclpy
from geometry_msgs.msg import Twist

T0 = time.monotonic()


def main():
    rclpy.init()
    n = rclpy.create_node('cmd_log')

    def fmt(tag, s):
        # flush 每行,否则 nohup 重定向到文件时会攒在缓冲区里,读日志时是空的
        print("%7.3f %s %s" % (time.monotonic() - T0, tag, s), flush=True)

    n.create_subscription(
        Twist, '/cmd_vel',
        lambda m: fmt('CMD', 'v=%+8.4f w=%+8.4f' % (m.linear.x, m.angular.z)), 50)
    n.create_subscription(
        Twist, '/wheel_speed',
        lambda m: fmt('DUTY', 'L=%7.4f R=%7.4f  (v_meas L=%+.4f R=%+.4f  tw=%+.4f)'
                      % (m.linear.z, m.angular.x, m.linear.x, m.linear.y,
                         m.angular.z)), 50)

    fmt('----', 'logger started; drag the joystick now')
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
