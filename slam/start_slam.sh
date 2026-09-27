#!/bin/bash
# 犀牛派 SLAM v3 一键启动（两轮差速电机驱动 + 轮式里程计）
# 里程计源二选一:
#   USE_WHEEL_ODOM=1 (默认) → motor_driver.py 轮式里程计 + 电机驱动(需 Web 先启用电机)
#   USE_WHEEL_ODOM=0        → laser_odom.py 激光 ICP 里程计(无电机)
# 用法: bash ~/start_slam.sh   或   USE_WHEEL_ODOM=0 bash ~/start_slam.sh

USE_WHEEL_ODOM=${USE_WHEEL_ODOM:-1}
echo "=== 犀牛派 SLAM v3 启动 (wheel_odom=$USE_WHEEL_ODOM) ==="

# 检查雷达
if [ ! -e /dev/ttyUSB0 ]; then
    echo "❌ /dev/ttyUSB0 未找到，请插入 RPLidar USB"
    exit 1
fi
echo "✅ 雷达已连接"

# 加载环境
source /opt/ros/humble/setup.bash
source ~/lidar_ws/install/setup.bash

# 清理旧进程
pkill -f rplidar_node 2>/dev/null
pkill -f static_transform_publisher 2>/dev/null
pkill -f async_slam_toolbox_node 2>/dev/null
pkill -f laser_odom 2>/dev/null
pkill -f motor_driver 2>/dev/null
pkill -f slam_viz 2>/dev/null
pkill -f rviz2 2>/dev/null
sleep 1

# 安全: 任何退出路径都先停电机(节点 SIGTERM 处理器会 estop)
trap 'echo "[slam] 停止并急停电机"; pkill -f motor_driver 2>/dev/null; exit 0' INT TERM

# 1. 雷达
nohup ros2 launch rplidar_ros rplidar_a1_launch.py > /tmp/lidar.log 2>&1 &
echo "✅ 雷达驱动已启动"

# 2. 里程计源（二选一，避免 /odom + TF 冲突）
if [ "$USE_WHEEL_ODOM" = "1" ]; then
    nohup python3 ~/motor_driver.py --ros-args --params-file ~/motor_params.yaml \
        > /tmp/motor_driver.log 2>&1 &
    echo "✅ 轮式里程计 + 电机驱动已启动 (初始电机禁用，需 Web 启用)"
else
    nohup python3 ~/laser_odom.py --ros-args -p base_frame:=base_link > /tmp/laser_odom.log 2>&1 &
    echo "✅ 激光里程计已启动 (ICP)"
fi

# 3. 静态 TF: base_link → laser
nohup ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link laser > /tmp/tf_laser.log 2>&1 &
echo "✅ TF 已发布"

# 4. 建图
nohup ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file ~/mapping_no_odom.yaml > /tmp/slam.log 2>&1 &
echo "✅ SLAM 建图已启动"

# 5. Web 可视化
nohup python3 ~/slam_viz.py > /tmp/slam_viz.log 2>&1 &
echo "✅ Web 可视化: http://192.168.31.220:5800"

# 6. rviz2（可选）
export DISPLAY=:2
nohup rviz2 -d ~/slam.rviz > /tmp/rviz2.log 2>&1 &
echo "✅ rviz2 已启动"

sleep 3
echo ""
echo "=== 话题检查 ==="
ros2 topic list | grep -E "/scan|/map|/odom|/tf|/cmd_vel"
echo ""
echo "🎉 全部就绪！"
echo "   Web:  http://192.168.31.220:5800   (先点[启用电机]再驾驶)"
echo "   VNC:  192.168.31.220:5902"
echo "   保存: python3 ~/map_saver.py -f ~/my_map"
echo "   急停: 浏览器 [急停] 按钮, 或 Ctrl-C 本脚本"
