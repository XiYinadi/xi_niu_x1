#!/bin/bash
# 犀牛派 火焰/烟雾检测(VLM 直判)启动脚本
# 用法: bash ~/start_fire_detect.sh
pkill -f fire_detect.py 2>/dev/null
sleep 1

# 火焰检测:独立进程(系统 python3,不导入 rclpy,避免 ROS env 破坏 cv2 V4L2)
# setsid+</dev/null:SSH 断开后进程不被挂到会话上,不会跟着死
setsid nohup /usr/bin/python3 ~/fire_detect.py > /tmp/fire_detect.log 2>&1 </dev/null &
echo "fire_detect PID=$!"

sleep 1
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "访问   http://${IP:-192.168.31.218}:5812/"
echo "标注流 http://${IP:-192.168.31.218}:5812/stream.mjpg"
echo "日志   tail -f /tmp/fire_detect.log"
