#!/bin/bash
# 犀牛派 SLAM Web 可视化 + 火情检测 启动脚本
# 用法: bash ~/start_slam_viz.sh
# 相机是单读设备:fire_detect 独占 /dev/video2 并出原始帧(/raw.jpg)+标注帧(/frame.jpg),
# 故不单独起 cam_stream。
pkill -f slam_viz.py 2>/dev/null
pkill -f cam_stream.py 2>/dev/null
pkill -f fire_detect.py 2>/dev/null
sleep 1

# 火情检测:独立进程(系统 python3,不导入 rclpy,避免 ROS env 破坏 cv2 V4L2)
# setsid+</dev/null:SSH 断开后进程不挂到会话上,不会跟着死
setsid nohup /usr/bin/python3 ~/fire_detect.py > /tmp/fire_detect.log 2>&1 </dev/null &
echo "fire_detect PID=$!"

sleep 1
source /opt/ros/humble/setup.bash
source ~/lidar_ws/install/setup.bash
source ~/robot/navigation/navigation_env.sh
setsid nohup python3 ~/slam_viz.py > /tmp/slam_viz.log 2>&1 </dev/null &
echo "slam_viz PID=$!"

sleep 1
IP=$(hostname -I 2>/dev/null | awk '{print $1}')
echo "访问   http://${IP:-<LAN-IP>}:5800   (地图/相机/火情/日志)"
echo "原始帧 http://${IP:-<LAN-IP>}:5812/raw.jpg"
echo "火情流 http://${IP:-<LAN-IP>}:5812/frame.jpg"
echo "日志   tail -f /tmp/fire_detect.log /tmp/slam_viz.log"
