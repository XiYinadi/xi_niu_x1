# 阶段一：犀牛派直接 ROS 2 控制与实车验收

不接 STM32，不启用编码器。`/odom` 为雷达扫描匹配 + GY-521 测量，
不是编码器轮式里程计；指令积分不作为位移反馈。

`ros_bridge.py` 订阅 SI 单位 `/cmd_vel`，使用实际 `/odom` 速度反馈生成
受限控制量，经本机 Socket.IO 驾驶权和现有 PWM/舵机进程输出。
默认禁用；每次使用 `/stage1/enable` (`std_srvs/SetBool`) 显式使能，
单次租期最多 40 秒。网页急停、控制权变更、连接丢失均锁定停止，
恢复必须重新显式使能。普通网页驾驶保留现有路径。

安全限制：仅前进 0..0.08m/s、转动 +/-0.20rad/s；两轮实际占空比均不超过
10%；指令超时 250ms、雷达/里程计/IMU 数据过期 400ms 即解除使能。
检测前向净空、转动周边净空、无运动卡滞、异常速度和直行偏航。
独立 watchdog 线程输出，不在阻塞 DDS 回调内等待停车。PWM 原有 300ms
deadman 与 armed 门保留。发现障碍物或未确认自遮挡时不屏蔽近点继续行驶。

`servo_hold_center` 仅供临时差速试验配置：`steer_mode: diff`、
`servo_enable: true`、`servo_hold_center: true`；后轮差速而前轮保持中位。
四轮接地时前轮需要横向滑动，因此“后轮能反转”不等于实车能原地转向。
开始短时低占空比试验，卡滞或拖胎明显即停止，不通过提高扭矩强行转动。
普通舵机转向配置不允许纯角速度命令。完成后恢复原参数和手柄状态。

## 试验和记录

源 ROS Humble 和 lidar_ws 环境，在阶段一目录运行。先以默认禁用状态
启动桥，再运行明确运动的 `run_trial.py`：

```
python3 ros_bridge.py --motor-params ~/motor_params.yaml --journal straight-bridge.jsonl
python3 run_trial.py straight_probe --output straight-probe.json
python3 run_trial.py watchdog --output watchdog.json
python3 run_trial.py one_meter --output one-meter.json
```

差速试验需要先停用电机、备份并切换临时参数、重启 PWM，再重启桥并添加
`--diff`。依次做 `spin_probe`、确认机械运动后才做 `ninety_deg`。
每次运行均独立使能/解除，最终记录停止后 1 秒的里程计。
原地 90° 的旋转方向为 ROS 正向（逆时针）。

试验结束必须独立用卷尺、地面角度线测量，输入 `evaluate_trial.py`。
记录实际测量的同一个车体参考点，不得用里程计读数代替地面真值。
默认验收候选标准：直行 1m 误差 <=5cm、横向漂移 <=5cm、偏航 <=5°；
90° 实际角度误差 <=5°、里程计与实测差 <=5°、参考点平移 <=5cm。
这是本轮拟用的测试标准，不能代替底盘设计要求。
如果实际绕后轴转动造成基准点明显平移，应报告转动中心与 TF 安装位置，
不能把这次运动标成纯原地旋转通过。

记录缺少 ground_truth 时 `accepted` 始终为 false。短脉冲、模拟测试和
静止数据只作为预检，不计入 1m/90° 的通过结论。
本轮不改 Nav2 `calibrated: false` 和缺失的实测参数。

测试：`python3 -m unittest test_core test_pwm_mode test_bridge`。
