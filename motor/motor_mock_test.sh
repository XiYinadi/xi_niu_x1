#!/bin/bash
# 犀牛派 mock 端到端联调 (FakeMotorHW, 不碰 GPIO)
#
# 实际逻辑在同目录的 motor_mock_test.py —— 单进程内完成,见那边的 docstring
# 说明为什么不能用 shell + ros2 CLI(echo 的 daemon xmlrpc 崩溃会让断言假通过;
# 反复重启 ros2 topic pub 的空档会被看门狗当成超时)。
#
# 用法: bash ~/motor_mock_test.sh
cd ~
source /opt/ros/humble/setup.bash
source ~/lidar_ws/install/setup.bash
python3 motor_mock_test.py
