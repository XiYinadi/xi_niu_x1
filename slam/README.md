# SLAM 建图管线

> 代码位置：`~/robot/slam/`；网页前端 `slam_viz.py` / `start_slam_viz.sh` 在 `~/robot/web/`。
> `~/` 下的同名文件是软链，旧命令（`bash ~/mapping_ctl.sh` 等）照常可用。

## SLAM 管线

```
RPLidar → /scan → (laser_odom → /odom + TF) → slam_toolbox → /map → rviz2 (VNC :5902)
                                                    ↓
                                    slam_viz.py (Web :5800) ← fire_detect.py (:5812 相机)
```

| 组件 | 作用 |
|------|------|
| rplidar_ros | 发布 /scan |
| **laser_odom.py** | ICP 扫描匹配 → 发布 /odom + 动态 odom→base_link TF |
| slam_toolbox | 建图，消费 /scan + /odom |
| rviz2 | VNC :5902 可视化 |
| **slam_viz.py** | Web :5800 平铺窗口:地图(跟随/缩放/HUD)+相机+火情+运行日志 |
| **fire_detect.py** | **相机的实际持有者**(:5812),`/raw.jpg` `/frame.jpg` `/stream.mjpg` `/`(状态 JSON),系统 python3 跑,**不导入 rclpy** |
| ~~cam_stream.py~~ | **已被 fire_detect 取代**(:5811 仍在但没人拉)。⚠️ 相机**单读**,再多一个抓相机的进程会把 fire_detect 饿死 |

### 网页「建图」按钮（2026-09-15）

地图窗口标题栏的按钮：一下拉起整条链路，再一下停。实际执行的是 `mapping_ctl.sh`。

| 要点 | 说明 |
|------|------|
| 状态判据 = **实际数据流**，不是「点过按钮」 | `slam_viz.mapping_status(scan_age, map_age)`（**纯函数**，可 headless 单测）：`map_age<10s` → **mapping**；否则 `scan_age<2s` → **lidar_only**；都不是 → **off**。出图要求雷达+里程计+TF+slam_toolbox **全活着**，缺一个都不出图 —— 所以它**不可能假装在跑** |
| ⚠️ `lidar_only` 是刻意的第三个态 | 「雷达在出数据但地图没在动」= 半启动 / 中途崩，**正是最容易被当成「建图中」的那个状态**，所以单独显式告警（黄色「SLAM 未起 ⟳」），不跟 mapping 混。`off` 文案「启动建图」，`mapping` 文案「建图中」 |
| ⚠️ 为什么不复用 `start_slam.sh` | ① 它第 28 行 `pkill -f slam_viz` → **会杀掉正在提供这个按钮的网页服务**；② 默认 `USE_WHEEL_ODOM=1` → 起 `motor_driver.py`，和 `pwm_proc` **抢同一批 GPIO line**（libgpiod 独占）；③ 无条件拉 rviz2（撞过 GLSL link error）。按钮的爆炸半径必须**恰好是那 4 个 SLAM 进程** |
| ⚠️ `mapping_no_odom.yaml` 的名字骗人 | "no_odom" 指**没有轮式编码器里程计**，不是「不需要 /odom」。它写着 `odom_frame: odom`，而 async 节点的 `laserCallback` 在 `getOdomPose` 失败时**直接丢弃那一帧扫描**（日志只刷 "Failed to compute odom pose"）。所以里程计源是**前提**，不是陪衬 |
| ⚠️ `-p base_frame:=base_link` 是**承重**参数 | `laser_odom.py` 默认 `base_footprint`，而静态 TF 发的是 `base_link→laser`、YAML 写的是 `base_frame: base_link`。少了这个覆盖 ⇒ TF 树裂成两截 ⇒ **地图全空**，而日志上只表现为刷警告 |
| 不碰电机 | `start` **不会 arm**，`stop` **不是 estop**，`pwm_proc` 的 deadman / armed 完全不受影响。按这个按钮车不会能动 |
| 只启停，不做地图操作 | **`/slam_toolbox/reset` 在 Humble 里不存在**（`srv/` 里没有 `Reset.srv`；`clear_queue` 只有 sync 节点有）。「重开新图」只能重启进程 —— 网页上没做 |
| 双击防火墙 | `_map_lock` + `_map_busy` 门，忙时按钮 disabled。两个 slam_toolbox 同时往 `/map` 和 TF 上写是那种**不报错**的坏 |
| 日志 | `subprocess.run` 的 stdout/stderr **逐行灌进既有 `_push_log(tag="slam")`** ⇒ 「雷达设备不存在」「motor_driver 在跑」这类失败**直接出现在网页日志窗口**，不用 ssh |
| ⚠️ `LIDAR_DEV` 可覆盖 | `LIDAR_DEV=${LIDAR_DEV:-/dev/ttyUSB0}` —— 留着是为了**不拔硬件就能故意测失败路径**：`LIDAR_DEV=/dev/ttyUSB_nonexistent bash ~/mapping_ctl.sh start` → 退出码 2 |
| 重启网页不影响建图 | 4 个进程全 `setsid nohup`，`start_slam_viz.sh` 的 `pkill slam_viz` **杀不到**它们 |
| ⚠️⚠️ `mapping_ctl.sh` 里必须临时关 `set -u` 再 source ROS | `/opt/ros/humble/setup.bash` 会引用**未定义**的变量（`COLCON_TRACE` 等）。开着 nounset 去 source，shell **在那一行直接退出** —— 表现是「返回 1、什么都没起、日志一行没动、也没有任何 ❌」，比报错还难查。脚本里还加了 `command -v ros2` 复核（否则会变成 4 个 "ros2: command not found" 写进 4 个日志文件，看着像 4 个独立故障） |
| ⚠️ 页面日志窗口的**截断**问题（已修） | `mapping_ctl.sh` 用 `>` 重定向原地截断日志，而 `log_tail_loop` 原本每文件只 open 一次 + `seek(END)`，**认不出截断** ⇒ 新日志头部被静默跳过。现在按 `os.fstat().st_size` 变小就重开文件 |

⚠️ 改这个按钮后**必须跑 `node ~/web_drive_test.js`**（第 9 节覆盖三态文案/class/disabled、点击发出的 `mapping_control`、busy 期间不发第二次、断线复位）。**一个会说谎的按钮比没有按钮更坏** —— 那正是这一节存在的理由。

## 相关文件

| 文件 | 用途 |
|------|------|
| `slam/mapping_no_odom.yaml` | SLAM 配置（无里程计模式也适用） |
| `slam/slam.rviz` | rviz2 配置模板 |
| `slam/laser_odom.py` | **ICP 激光里程计** |
| `slam/map_saver.py` | 保存地图 PGM+YAML (不依赖 nav2_map_server) |
| `slam/mapping_ctl.sh` | **建图管线启停开关**（`bash ~/mapping_ctl.sh start\|stop\|status`）— 只碰 4 个 SLAM 进程，网页「建图」按钮调的就是它；**不碰电机**。退出码见脚本头 |
| `slam/start_slam.sh` | 完整管线一键启动脚本 |
| `slam/launch_rviz2.sh` | rviz2 启动脚本 |
| `slam/my_map.pgm` `my_map.yaml` | 已保存的地图 |
| `web/slam_viz.py` | **Web 实时可视化** (Flask+SocketIO, 地图+扫描+轨迹+相机) —— 文件在 `web/` |
| `web/start_slam_viz.sh` | Web 可视化启动脚本 —— 文件在 `web/` |
