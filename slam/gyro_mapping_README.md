# 激光 + GY-521 建图里程计

2026-10-20：主建图入口和加载地图定位入口已去掉编码器依赖。

数据流：RPLidar `/scan` + GY-521 `/imu/data_raw` → `laser_imu_odom.py` → `/odom` + `odom→base_link` → slam_toolbox `/map`（或已保存地图的 AMCL 定位）。平移来自点对面激光匹配；陀螺仪提供扫描间转角约束。不对加速度双重积分，不把陀螺仪当成绝对位置或磁罗盘。

## 使用

仍使用网页“启动建图”，或 `bash ~/mapping_ctl.sh start|stop|status`。启动后保持车体静止约 3 秒供零偏校准。等待 `/tmp/mpu6050.log` 出现 `Calibration ready`，以及 `/tmp/laser_odom.log` 出现 `Odometry source: laser+gyro`。校准未通过会重试，期间使用激光匹配。

模块必须固定在车体，不能固定在会转动的相机云台。启动时用静止重力方向建立传感器向上的轴，即使模块倒置，平面转角符号仍可投影得到；该方向随后保持固定，不追随车辆加速度。`/imu/data_raw` 使用模块自身 `imu_link` 坐标系、SI 单位，并标记没有绝对姿态估计；融合节点直接投影角速度，无需虚构模块安装位置。

## 故障行为

- 编码器标定、编码器 GPIO、采样质量均不再是建图/定位前提；保留旧文件和标定，以便日后回退。
- IMU 覆盖不足或通信失败：明确告警并使用激光匹配；数据恢复后自动融合。不会因单次 IMU 读取失败退出整条管线。短暂通信错误保留有效零偏，避免在运动中重新校准。
- 激光匹配失败不会发布假里程计，也不会推进参考帧。启动首帧的长间隔会被丢弃；已建立轨迹后参考丢失超过 1 秒会停止发布里程计，需静止重启，避免把未知位移伪装成零位移。
- MPU6050 没有绝对航向，温漂和航向漂移仍然存在；长走廊几何退化、激光缺失等情况不能靠陀螺仪完全补救。

## 部署

板上新文件：`~/robot/slam/{gyro_core,laser_odom_core,mpu6050_node,laser_imu_odom}.py`。

已修改：`~/robot/slam/mapping_ctl.sh`、`~/robot/navigation/navigation_launch.py`、`~/robot/navigation/navigation_runtime.py`。后两项的修改版本也保存在本目录，对应本地 `robot_navigation_work` 已同步改动。原始部署备份：`~/robot/backups/gyro_mapping_20261020_213059/`。

`fix-imu-perms.service` 仅将已验证的 `/dev/i2c-1` 设为 `root:aidlux 660`，按 AidLux 容器实际情况使用开机 oneshot 恢复权限。当前用户访问和服务启用已验证，尚未重启验证。没有改变其他 I²C 总线权限。

主启动入口使用 5 个进程：雷达、IMU 读取、激光/陀螺仪里程计、激光静态 TF、slam_toolbox。stop 也清理历史轮式/激光里程计，避免重复 TF。没有改变电机 armed/deadman 门控。此实现不调用 legacy `start_slam.sh`。

## 验证记录

- 纯数学 7 项测试：扫描时间窗插值积分、正负转向、丢样/时钟回退、静止校准/倒置安装、已知平移旋转、陀螺仪约束实际改变解、几何退化拒绝。
- ROS 回调 2 项：A1 启动首帧长间隔、激光/陀螺仪模式往返、Humble 日志级别契约、匹配失败不发布假数据。
- 真实静止数据流 20 秒：138 条里程计、1732 条 IMU、135 帧激光、7 帧地图；唯一 `/odom` 发布节点 `laser_imu_odom`，`map→base_link` 可查。IMU 年龄约 0 秒，里程计年龄 0.067 秒。
- 暂停 IMU 读取进程 2 秒后恢复：日志实际经历 `laser+gyro → laser-only → laser+gyro`；建图节点未退出。
- 实际网页 SocketIO 10 秒：77 帧扫描、4 帧地图、11 次轨迹事件，状态 `mapping`，里程计年龄约 0.024 秒。
- 板上 `node ~/web_drive_test.js`：148 通过、0 失败。
- 全程电机 `armed=N`、双轮 duty=0。未验证移动车辆时的航向符号、尺度、闭环误差，也未启动保存地图的 AMCL 实地定位；定位入口已改用新里程计。

测试命令：系统 Python 运行 `test_gyro_core.py`；source ROS 后运行 `test_ros_adapter.py`、`verify_mapping.py`。ROS 运行时需可访问系统 scipy，程序已按现有 AidLux 模式补充系统 site-packages 路径。

寄存器配置依据：[TDK MPU-6050 Register Map](https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf)。IMU 消息未估计姿态的标记依据：[ROS sensor_msgs/Imu 定义](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg)。
