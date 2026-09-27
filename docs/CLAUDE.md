# 犀牛派 — RPLidar SLAM 项目 (ROS2 原生)

## 设备连接

**犀牛派 (AidLux 盒子)**
- SSH: `ssh aidlux@192.168.31.220` (密码: `aidlux`, 已配密钥免密)
- 系统: Ubuntu 22.04 / ARM64 / 6核 / 14G RAM
- VNC: `192.168.31.220:5902` (DISPLAY=:2) —— ⚠️ 原文档写的是 `.218`，**那是错的**：
  `hostname -I` 实测板子只有 `192.168.31.220`(wlan0) 和 `192.168.1.1`(br-lan)，
  `.218` 是历史地址。⚠️ 且 2026-09-26 实测 **59xx 上没有任何 VNC 在监听** ——
  用到时先 `ss -lntp | grep 590` 确认，别默认它在跑（**别照抄旧文档里的 IP**）

**RPLidar 激光雷达 A1**
- USB 芯片: CP210x (`10c4:ea60`)
- 设备节点: `/dev/ttyUSB0`
- S/N: `75A899F6C9E59AD2C5E59CF7897A3412`

## ROS2 环境

- ROS2 Humble: `/opt/ros/humble/` (AidLux aidros-pyenv)
- 工作空间: `~/lidar_ws/` (rplidar_ros + slam_toolbox)
- 编译前: `source /opt/ros/humble/setup.bash`
- 运行时: `source ~/lidar_ws/install/setup.bash`

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

## 项目文件

| 文件 | 用途 |
|------|------|
| `mapping_no_odom.yaml` | SLAM 配置（无里程计模式也适用） |
| `slam.rviz` | rviz2 配置模板 |
| `laser_odom.py` | **ICP 激光里程计** |
| `slam_viz.py` | **Web 实时可视化** (Flask+SocketIO, 地图+扫描+轨迹+相机) |
| `cam_stream.py` | **已被 `fire_detect.py` 取代** (:5811 仍在但没人拉)。见「相机」一节 |
| `fire_detect.py` | **相机的实际持有者** (:5812) |
| `start_camera.sh` | ⚠️ **已弃用/有害** —— 起的是 `camera_preview.py`，见下方「相机」一节 |
| `motor_hw.py` | 电机硬件层：`DualSoftPwm` 软 PWM / `SoftServoPwm` 舵机(50Hz,µs 脉宽) / `GpioMotorHW` / `QuadDecoder` / `FakeMotorHW` |
| `motor_control.py` | 纯逻辑/数学（可 headless 单测，不碰 GPIO/rclpy）：`Pid` / `wheel_duties` / `joy_to_channels` / `map_channels` / **`Deadman`**（带滞回的链路失联判据，见下方「安全网」） |
| `motor_driver.py` | ROS 电机节点（/cmd_vel → GPIO、看门狗、轮式里程计）— **与 pwm_proc 互斥** |
| `pwm_proc.py` | **直连 PWM 独立进程**（UDP :5813，不导入 rclpy）— 见下方「电机驱动链路」 |
| `start_pwm.sh` | 启动 pwm_proc（会先停掉 motor_driver 让出 GPIO） |
| `motor_params.yaml` | **参数单一来源**：几何 / PID / GPIO line / 直连 PWM 段（含 `swap_lr`） |
| `motor_wiring.md` | 接线表 + 「通道 ≠ 左右轮」实测定案 + 舵机接线与 5V 风险 + 故障对照 |
| `joy_inject.py` | **摇杆注入器**：绕开浏览器直灌 UDP，单轮隔离 / 阈值扫描 / 全时序 / 舵机扫描(`--servo-sweep`)。⚠️ `--hz <= 3.3` 会**直接退出**（那守卫是挡真实误用，别放宽）—— 想注入低于它的包率用下面那个 |
| `slow_inject.py` | **低包率注入器**（`[间隔秒] [时长秒] [noarm]`）：按恒定间隔发包，专门复现「**后台标签页被节流到 1Hz**」那种包率，用来回归验证 deadman 的滞回。`l`/`a` 恒 0 ⇒ 轮子不励磁、舵机不偏，**车一步都不会动**，验判据不需要通电也不需要架空 |
| `servo_pulse_probe.py` | **舵机脉宽实测**：Proxy 包住真实 `_line` 量脉冲串，判「抖」是不是我们信号造成的 |
| `gimbal_proto.py` | **云台协议纯函数层**：`encode_frame`/`decode_frame`/`clamp_cmd100`，**只 import `struct`**（多一个 `import termios` 离线测试就死）。协议原文 + C 黄金向量在它的 docstring 里 |
| `gimbal_test.py` | **云台协议离线测试**（本机 Windows 可跑）：逐字节黄金向量 / 值域全量往返 / 小端盲区 / 帧头碰撞可达性 —— 见下方「二维云台」 |
| `gimbal_proc.py` | **云台串口独立进程**（UDP :5814 → `/dev/ttyHS1`）。⚠️ **全仓库唯一允许 import `termios` 的地方** |
| `gimbal_inject.py` | **云台注入器**（绕开浏览器直灌 5814）：`--axis-sweep` 定方向/行程、`--single-frame` 判绝对量还是增量。⚠️ **没有 `--arm`** —— 见下方「二维云台」 |
| `start_gimbal.sh` | 启停云台串口进程（`bash ~/start_gimbal.sh start\|stop\|status`）— **只碰 `gimbal_proc` 一个进程** |
| `gimbal_params.yaml` | 云台参数：串口节点 / 波特率 / 方向反转。⚠️ **三项都未定案**，见文件头 |
| `gimbal_wiring.md` | 云台接线表 + 3.3V 电平风险 + **节点名 `ttyHS1`/`ttyHS3` 陷阱** + 回环测试的能与不能 + 5V 轨供电警告 |
| `web_drive_test.js` | **网页驾驶控件离线测试**（`node web_drive_test.js`，退出码 0 = 全过，现 104 项）：从 `slam_viz.py` 抽出**真实内联 JS** 在假 DOM 上跑，覆盖双杆/单杆/切换安全/键盘/模式记忆/建图按钮/**页面不可见时停发**/**拖拽 pointermove（第 11 节）**/**云台摇杆（第 12 节）**；顺带静态核对 HTML↔JS 的 id 接线。⚠️ 假 DOM 要能验「停发」就得先有真的 `clearInterval` 与 `window.addEventListener` 捕获（原来一个是空函数、一个直接丢弃），`read()` 还必须跳过已停的定时器 |
| `motor_test.py` | 最小单轮测试（`--observe` 用人眼判定；不跑 ROS） |
| `diag_client.py` | SocketIO 诊断客户端（按指定 Hz 灌 vel，验服务端链路） |
| `rate_probe.py` | 采样话题/回调的实际间隔，用来发现控制环被饿死 |
| `map_saver.py` | 保存地图 PGM+YAML (不依赖 nav2_map_server) |
| `mapping_ctl.sh` | **建图管线启停开关**（`bash ~/mapping_ctl.sh start\|stop\|status`）— 只碰 4 个 SLAM 进程，网页「建图」按钮调的就是它；**不碰电机**。退出码见脚本头 |
| `start_slam.sh` | 完整管线一键启动脚本 |
| `start_slam_viz.sh` | Web 可视化启动脚本 |
| `rplidar_web.py` | 旧版 Web 控制（已弃用） |
| `slam_web.py` | 旧版 SLAM Web（已弃用） |
| `slam_bridge.py` | 旧版数据桥接（已弃用） |

## 电机驱动链路

**当前走「直连 PWM」这条**（2026-09-15 起）。两条路互斥，GPIO 是独占的。

```
网页摇杆 60Hz → slam_viz → UDP 127.0.0.1:5813 → pwm_proc(独立进程) → GPIO
```

| 要点 | 说明 |
|------|------|
| 为什么独立进程 | 软 PWM 是 1kHz 纯 Python 忙等线程，留在 motor_driver 里会和 rclpy 单线程执行器抢 GIL，把控制回调**间歇性饿死**（实测 `/wheel_speed` 间隔中位 20.3ms 但 p90 116.6ms / p99 466.9ms）→ 松手延迟 931ms |
| ⚠️ 独立进程只解决一半 | 同一个忙等线程在 pwm_proc **自己进程内**又饿死了它**自己的** 200Hz 主循环（掉到 1~2Hz），并撑爆 UDP 接收缓冲（发 60 包/s 只收到 24）。真凶是 `_busy_until` 对整个 PWM 周期纯自旋；改成「粗睡 300µs + 末段精等」后轮询回到 199~200Hz、包率 59.6/s、PWM 线程 CPU 93~100%→6.7%。**动那段前先读它的 docstring** |
| ⚠️ 通道 ≠ 左右轮 | **通道 L（GPIO 91/61/62，即 PWMA/IN1/IN2）驱动的是【右】轮**，通道 R（63/60/124）驱动【左】轮 —— 与 `motor_wiring.md` 的原始设计相反。故 `swap_lr: true` 是定案值；**整体转向反向只能改 `swap_lr`，改 `flip_*` 修不回来**；且通道交叉后 `flip_left` 实际作用于右轮、`flip_right` 作用于左轮 |
| duty 怎么算 | `duty = abs(joy_to_channels(l, a, ...))`，**无斜率限幅**。直行时 duty 严格等于摇杆比值（摇杆 0.8 → duty 0.8）。⚠️ `a` 是否参与取决于 `steer_mode`：**当前 `servo` 模式下喂进去的是 `a=0`**，故 `s_l == s_r`、满杆直行 0.9。只有在 `diff`/`both` 下才有那个「满杆转向只有 0.333」的权限不对称 |
| 安全网 | pwm_proc 本地 deadman `pwm_deadman_ms`（默认 300ms 无摇杆包 → duty 0）+ `armed` 门控（上电默认 false）。ROS 看门狗被绕开，这是零成本替代。⚠️ **跳闸 300ms / 恢复需连续 600ms（=2×300）**，判据是 `motor_control.Deadman`（纯状态机，`motor_driver_test.test_deadman` 覆盖）。恢复侧必须**严格大于**跳闸侧，否则链路包间隔跨在阈值两侧时会 chattering —— **2026-09-15 实测**：浏览器把后台标签页的 `setInterval` 钳到约 1Hz（包间隔 ~400ms），裸阈值实现产生 **61 次严格交替的「中断/恢复」**，而舵机与电机**同一个门** ⇒ 舵机跟着 park↔全速回中 ⇒ 用户报的「**启用电机后舵机老是来回抖**」。滞回后：包间隔 >300ms 的链路跳闸一次就**永远恢复不了**（单个 fresh 窗口最多 300ms，攒不够 600ms），机械上不可能再抖。代价：进程刚起来有 600ms 对摇杆没反应（那时 `armed` 还是 false，无感） |
| 启动 | `bash ~/start_pwm.sh`（自动停 motor_driver 让出 GPIO） |
| 观测 | `tail -f /tmp/pwm_proc.log` — 每 5 秒一行：包率 / 龄期 / `steer=` / duty / `servo=`（`off`=没构造 / `PARK(idle)`=**指令**静止、我们主动放掉 / `PARK`=不发脉冲 / `1500us`=正在发）。⚠️ 静置时「包率 0」「l/a 全 0」都是**正常**的（没人开网页 / 死人手柄在发零），别当成故障。⚠️ `steer=servo` 时**「a 变了而 duty 不变」也是正常的** —— 那正是「转向不经过轮子」。`servo=` 是「开了舵机但没接线」**唯一**能被看见的地方。⚠️ `PARK(idle)` 只在**idle park 确实是拦住它的原因**时才报（`armed` 且不 stale 且 `a` 真该给舵机）；「没使能 / 链路断了」照样报 `PARK` —— 这两件事的排查方向正相反，不能混 |
| 免浏览器验证 | `python3 ~/joy_inject.py --arm --wheel L`（单轮隔离）、`--sweep`（扫启动阈值）。它是唯一能把「网页链路坏」和「GPIO/电机坏」分开的工具 |
| 舵机（转向） | 加在**同一进程、同一个 armed/deadman 门**上的第三路输出，由摇杆的 `a` 驱动 —— **前端与 UDP 契约一个字都没改**。不驱动时 park（**完全不发脉冲** → 舵机松掉、几乎不吃电）。引脚 pin28/GPIO_33，用 `--a ±1` 单独验证（`servo` 模式下 `l` 默认 0 ⇒ 两轮 duty 恒 0，**不会驱动电机**）。⚠️ 供电走排针 5V（与 L298N 逻辑、相机**同轨**），**风险见 `motor_wiring.md` 第三节**，软件侧无法检测低压。**「静止时一直轻微抖」已实测判定不是本进程的信号**（`servo_pulse_probe.py`：脉宽中位 1502µs / p1-p99 1498-1625 / 坏帧 0.4 次/秒）—— 往**共地**、机械限位、舵机自身找，别改 `_busy_until`。⚠️⚠️ **但「启用电机后舵机来回抖」是另一回事，那是本进程的锅，已修**（deadman 在 400ms 包率下 chattering，见上方「安全网」行）。**别拿上面那条结论去套它** —— 上面量的是「电机没通电、舵机也没被驱动」的工况；而「启用电机后」`armed=Y`，舵机才真的开始收脉冲，是**另一个工况**，旧的结论不覆盖它。判据：日志里 `deadman:` 有没有成对刷屏。⚠️⚠️ **2026-09-16 再加一层「静止 park」**（`servo_idle_park_ms`，默认 1000，`0` = 关）：`armed` 但指令一直停在零 → 舵机在 `armed` 期间被持续通电顶在 `servo_center_us` 上，内部模拟控制环一直纠偏 ⇒ 嗡嗡 + 微抖 + 白吃电流。判据是 `motor_control.IdlePark`（纯状态机，`motor_driver_test.test_idle_park` 覆盖）。**滞回是不对称的：进 idle 要连续静满 `hold_s`，退 idle 是第一个非零样本立刻退** —— 所以「舵机落后于指令」结构上不可能。⚠️ **它不是第二套链路门**，别照着 `Deadman` 的样子给它加滞回/单独门控：那会回归成「电机停了、舵机还咬着转角」，而两者共用同一个门正是它不拆成独立进程的全部理由。⚠️ 想 A/B 对照就把 `servo_idle_park_ms` 改 `0` 重启，行为退回「一 armed 就恒顶中位」 |
| 转向归谁（`steer_mode`） | **互斥三选一**，不是叠加开关：`servo`（当前）＝`a` **只**给舵机、轮子只吃油门两轮等速；`diff`＝老行为，只走差速、舵机恒 park；`both`＝两者都吃。不做成可叠加是因为同时转向会**互相打架**。⚠️ `servo`/`both` 要求 `servo_enable: true`，否则 `pwm_proc` **直接退出**（退出码 1）—— 没舵机就是整车无法转向，不做降级。⚠️ 选 `servo` 后「整体转向反只能改 `swap_lr`」**失效**（`a` 到不了轮子，交换恒等通道等于没交换），改判 `servo_invert`；`joy_inject.py --wheel L\|R` 的单轮隔离也**无解**，它会拒绝执行而不是静默同时驱动两轮 |
| 驾驶权门控 | `armed` 只认**点了「启用电机」的那个客户端**（`slam_viz._drive_owner`）；未使能时任何客户端发的 `vel` 一律丢弃。2026-09-15 修掉的洞：原判据 `_drive_owner is not None and ...` 在 `_drive_owner is None` 时**短路成 False**，于是**不点启用就能驱动电机** |
| 网页控件（单杆/双杆**可切换**） | **双杆**（默认）：左盘**只认纵轴**=油门 `l`，右盘**只认横轴**=转向 `a`，各自 `setPointerCapture`（双指可同时按），两轴独立可同时满舵。**单杆**：一个 180px 的 2D 盘同时给两轴（`x→a`、`y→l`），**圆门** —— 斜推到角落是 0.707/0.707 而非 1/1（模长钳到 1，保住指令方向；要「满油门+同时满转向」就用双杆）。切换按钮在 Drive 标题行，选择记在 `localStorage`。**两种模式只是输入方式不同：UDP 契约、`l`/`a` 符号约定、键盘、驾驶权仲裁、armed 门一个字没改** —— 都只是往同一个 `driveTarget()` 里填 `l`/`a`。⚠️ 每个盘带 `vx/vy` 两个分量（单轴盘死轴恒 0），所以 `driveTarget()` 一个式子通吃两种盘。⚠️ **切模式必须先 `releaseAll()` 再隐藏**：隐藏一个正持着 pointer capture 的元素时 pointerup 不保证送达，`active` 残留就会让看不见的盘继续出力（车一直跑）；`driveTarget()` 还**只读当前模式的那几个盘**（`MODE_PADS`）作为双保险。⚠️ **2026-09-16 修掉两个拖拽缺陷**（用户报「单杆时前后正常、转向有时无反应」；服务器只是把前端算出的 `a` **原样转发**，所以前端映射缺陷会一比一传到舵机）：① **指令会冻死** —— `padUpdate()` 原来是「先按轴各钳到 ±1、**再**按模长归一化」，只要某一轴超出 1（手指离开那个 180px 的盘子），两轴双双钳死，整条指令固定成 `0.707/-0.707` 再也不变；而 `setPointerCapture` 让盘外的 move 照样送进来 ⇒ 「推满前进 → 再往右滑想转向」这种**最正常的手势**一越界就卡死。现在**只钳模长**（方向原样保留，`(1.5,-1.0)` → `0.832/0.555` 而不是冻住的 `0.707/-0.707`）。⚠️ **顺序是承重的**，别退回「先按轴钳、再归一化」。② **失焦后拖拽失效** —— `blur → releaseAll()` 把 `p.active` 清成 false，而 `pointermove` 只在 `if (p.active)` 时才处理 ⇒ **手指没抬起来，后续拖动全被丢弃**，必须松手重按。现在每根手指记 `p.pid`，判据是 `p.active || p.pid === e.pointerId`：失焦/切标签页传 `releaseAll(true)` **保留 pid**（那时手指往往还按着、`pointerup` 不会来），切模式用默认值清掉。安全侧不降级 —— `blur` 仍**立刻归零并补发零速**，恢复只发生在用户**真的又动了**那根还按着的手指时、用的是手指**当前位置**。⚠️ 单轴盘**死轴必须在归一化之前摁成 0**：摁完另一轴恒 0 ⇒ 模长就等于该轴自己的绝对值，「只钳模长」对它是**恒等变换**，与旧的按轴钳位逐点等价，横向偏移依然折不进来（旧注释说的「单轴盘不做 hypot 归一化」只对**摁死轴之前**的顺序有意义，那句话正是让人以为这里可以分支处理的源头）。⚠️ 单杆盘的 180px **不是**把双杆改回大 2D 盘的那个老错误 —— 2D 盘确实要靠角度区分前后左右所以盘子要大，「别放大到 240px」那条只针对单轴盘。⚠️ 改这个控件后**必须跑 `node ~/web_drive_test.js`**：它是唯一不靠浏览器就能验这个页面逻辑的东西（含「按住摇杆切模式必须立刻归零」），而这类 bug 的特点是**不报错、只是在错误的时候出力**。⚠️ 第 11 节是上面两条的回归 —— **测试台原来整条拖拽路径零覆盖**（老 `touch()` 只发 `pointerdown`，全文件 `pointermove` 出现 0 次），所以「拖动」这件事今后必须有用例：加盘、改映射、改指针事件时先看那一节。⚠️ **页面不可见时发送循环必须真的停**（`setDriveRunning(!document.hidden)` + `clearInterval`）：浏览器会把不可见标签页的 `setInterval` 钳到约 1Hz，而 1Hz（包间隔 ~400ms）落在 deadman 300ms **以下** ⇒ 舵机跟着抖（见「安全网」行）。节流绕不过去，所以「不可见就不发」，读数照实显示**「已暂停(页面不可见)」**——定时器停了读数不再刷新，留着最后一帧的 `前进 +0.80` 是**假**的。⚠️ 起始用 `!document.hidden` 而**不是** `true`：页面**一开始就在后台标签页**打开时不触发 `visibilitychange`，写 `true` 等于让它带着 1Hz 的包率起跑，恰好是这个 bug 本身。⚠️ `blur`（失焦但仍可见）**不**在此列，不被节流、循环不停 |

`motor_driver.py` 那条 ROS 路径（/cmd_vel、PID、轮式里程计）仍然保留可用，
但**不能和 pwm_proc 同时跑** —— 两者申请同一批 GPIO line，libgpiod 独占，
后启动的报 line busy。

## 二维云台（串口输出，2026-09-27）

```
网页云台摇杆 62.5Hz → slam_viz → UDP 127.0.0.1:5814 → gimbal_proc(独立进程)
                                                      → /dev/ttyHS1 (PIN_29) → 云台
```

犀牛派是**发端**（只发帧，从不接收），所以**只接 pin29(TX) + pin30(GND)**；pin16 不接。
**完全不碰 GPIO** ⇒ 不可能和 `pwm_proc` 撞 `line busy`。

| 要点 | 说明 |
|------|------|
| 协议（**出处不在本仓库**） | 6 字节 `0xAA \| dxH dxL \| dyH dyL \| SUM`；`dx/dy` = **signed int16 大端**，值域 −100..+100（**不是角度**，参考实现里那个 `s_gimbal_dx_deg` 名字骗人）；`SUM = (dxH+dxL+dyH+dyL) & 0xFF` —— **算术和，不是 XOR 不是 CRC**；115200 8-N-1 无流控无转义单向无 ACK；(0,0)=回中。参考实现在 ESP32-S3 `hello_world_main.c:241-268`，**仓库里没有任何 `.c`** ⇒ 逐字节黄金向量 + C 转写是**唯一**的记录，在 `gimbal_proto.py` docstring 与 `gimbal_test.py` 里 |
| ⚠️⚠️ 校验和对**字节序全瞎** | SUM 是四字节**普通和**，与顺序无关 ⇒ **小端实现照样过校验**（`AA 9C FF 00 00 9B` 校验通过却解出 `dx=−25345`）。**所以测试绝不能断言「校验和正确」**，必须断言**逐字节固定向量**。位错它倒是完美（27024 个单比特翻转全检出）|
| ⚠️⚠️ `0xAA` 在**数据字节里可达** | 401/40401 组帧含假帧头，**全部来自 `dx` 或 `dy == −86`**（`gimbal_test.py` 断言了这个充要条件）。协议**无转义、无帧头重同步** ⇒ 接收端一旦错位就**永久错位**。⇒ **短写是承重的**，不是「尽力而为」（见下）|
| 为什么独立进程 | 串口坏了只坏云台，不连累网页+地图；且要能单独重启。⚠️ **尤其不要塞进 `pwm_proc`** —— 那个 200Hz 严格节拍里加一个**可能无限阻塞**的 syscall 是同类事故的第三次 |
| ⚠️ **没有 `armed` 门**（刻意的） | 转相机 ≠ 驱动轮子。网页上拨云台摇杆**立刻动**，不必点「启用电机」。⚠️ 这**不是**「不点启用就能驱动电机」那个已修 bug 的翻版，别照搬。副作用必须知情：**急停按钮 `estop` 现在会顺手把云台发一帧 (0,0)**（让「急停」这个标签诚实），但不 arm 也不会锁云台 |
| ⚠️ 断链 ⇒ **保持不动**，不回中 | 网页不可见 / blur / 断线都**只停发**，**不发 (0,0)** —— 否则「切个标签页相机就甩回正中」。⚠️ 但**操作者的明确动作**（松手、失焦归零、按急停）**照样发 (0,0)**：那两件事必须分开 |
| ⚠️ 发送节拍 **16ms 定节拍连发**（不是参考实现的事件驱动） | 参考实现**没有死手**所以事件驱动是对的；我们加了死手（500ms）就必须改成定节拍 —— 否则「摇杆按住不动 = 不发帧 = 判为断链」。⚠️ 但这个决定**依赖「dx/dy 是绝对量」**，而那**尚未定案**（见下）|
| ⚠️ 死手是**无滞回**的，动作是「停发」 | 和 `pwm_proc` 的 `Deadman`（跳 300 / 恢复 600）**不是一回事**：云台没有「舵机跟着抖」那种代价，所以不需要滞回，也就**不要**照抄那套滞回 —— 串口的失败形状是**相位错位**，不是 chattering |
| ⚠️⚠️ 短写**必须补齐剩余字节** | 不能照抄 `slam_viz._send_drive` 的「吞掉 OSError」（那对 UDP 的**最新值语义**是对的，对**字节流**是错的）：一次短写 = 把接收端的解析相位**永久推移半个帧**，协议没有重同步兜底。补齐代价可忽略（6 字节），但**也不许抛出去杀掉循环**（一次抖动变永久停机）⇒ 补齐 + 计数 + 状态行里说出来；真补不齐就**标记端口 broken 并停发**（响亮），不是继续按错相位写 |
| ⚠️ `tty.setraw` **不够** | 它**不设波特率**（⇒ 停在驱动默认 ⇒ **格式正确的乱码**）、**不清 `IXOFF`**（我们**从不 read**，缓冲填满后**内核会替我们往线上吐 `0x13`/`0x11`** —— 而这两个字节在值域内可达）、**不清 `CRTSCTS`**（三线接法下输出被永久闸住）、**不清 `CSTOPB`**。Python 的 `termios` **没有 `cfmakeraw`** ⇒ `gimbal_proc._cfmakeraw()` 是自己写的，且 **`tcsetattr` 之后必须 `tcgetattr` 读回断言**（读**内核状态**，不是读我们的意图）|
| ⚠️ 写出去 ≠ 上了线 | `os.write` 返回 6 只说明**驱动缓冲收了**。⇒ 状态行里**不许出现「发送成功」**，只准报 帧/s、包/s、龄期、`HOLD(停发)`/发送中、dx/dy、写入 B/s、短写/未补齐/写错、端口状态。⚠️ 这些是**5 秒窗均**，而 `发送中`/`HOLD(停发)` 是**瞬时**状态 —— 两者可以同时看起来矛盾（`HOLD(停发)` 旁边挂着 `写入 131B/s`，其实是窗口尾巴还在发）。所以行尾自己标了「第1/2/3/5 个数为 N 秒窗均」 |
| 实测能力（2026-09-27 上板） | 62.5Hz **余量巨大**：把 `gimbal_cmd_period_s` 调到 `0.002`（目标 500Hz）时循环实跑到 **481Hz**。⚠️ 真正的上限是**线速**：115200 只扛 **11.52kB/s**，而 5000Hz×6 字节 = 30kB/s ⇒ 缓冲满 ⇒ `未补齐` + **端口坏并停发**（实测触发，行为正确 —— 半帧上线后本协议无帧头同步，**不能接着写**）|
| ⚠️ 饿死告警的判据 | **必须用「发送期间」的实测节拍**（发送时段首尾跨度算出），**不能**拿「5 秒窗口平均帧率」去比目标帧率 —— 窗口只要跨一次 `HOLD → 发送中` 就被稀释（实测 2.1s 发送 + 2.9s 停发 ⇒ 报 26.2Hz/目标 62.5Hz），于是**正常工况也响**。⚠️ 也**不能**用「该发次数 vs 实际帧数」：那两个恒相等（`_write_frame` 唯一不计数的路径已经设了 `_port_broken`，那时提前 return）⇒ 判据**永远不响**。两个版本都实测踩过。用 `SIGSTOP` 反复挂起进程验证它**确实会响**（实测真响：`发送期间只跑到 22.8Hz/目标62.5Hz`）|
| 独占 | `TIOCEXCL` ⇒ 第二个 opener 报 `EBUSY`。把「别人偷偷往我的线上写字节」从**静默污染**变成**响亮失败** |
| 前端（**本任务风险最高处**） | 云台盘 `#joy-gimbal` 进 `padList`（要指针注册和 `releaseAll` 清 active），**但不进 `MODE_PADS`** ⇒ `driveTarget()` 结构上够不到它，驾驶语义一个字不改。⚠️ **`driveGate()` 的循环必须 `if (p.isGimbal) continue;`** —— 它遍历 `padList`，而判据 `!armed \|\| driverSid===null \|\| …` 会让「**别人点了启用电机** ⇒ 我的云台被锁死（要拿回相机得先给电机上电）」和「`armed=false` ⇒ **两个浏览器同时灌云台**」。⚠️ 这个 `!armed \|\|` 的形状和 `slam_viz.py` 已修过的 `_drive_owner is not None and …` **是同一个坑第二次** |
| ⚠️⚠️ 定时器**必须共用一个 16ms** | 写成两个 `setInterval(...,16)` 会让 `web_drive_test.js` 的 `liveTimer(16)`（`find` 不是 `filter`）拿到**云台**那个 ⇒ `read()` 重放出陈旧文本 ⇒ **测试照样绿，而驾驶链路已断**。⇒ `setInterval(tickAll,16)`，`tickAll(){ driveTick(); gimbalTick(); }`，并加断言「16ms 定时器恰好 1 个」|
| ⚠️ 读数必须是**独立元素** `#gimbal-val` | 写进 `drive-val` 会让 `read()` 的正则整段失配 ⇒ 每个驾驶用例都红，而原因看起来毫不相干 |
| 方向 | `gimbal_invert_x` / `gimbal_invert_y` **两个独立未知量**，性质和 `servo_invert` 一样（**别照着名字猜**）。⚠️ **反转发生在 `gimbal_proc` 的接收侧**，不在前端也不在 `gimbal_inject.py` ⇒ 只看注入器日志会得出**错误的**方向结论（所以它把「注入的值」和「进程真正发的值」都打出来）|
| ⚠️ 圆门对云台**照样生效** | `padUpdate` 的 `hypot` 归一化不分支 ⇒ 斜推到角落是 `dx=dy=70`，**不是 100**。云台的 pan/tilt 是两个**正交独立**的电机、没有「方向」可言，所以圆门在这里是**无缘无故把两轴耦合起来** —— 但 **v1 决定接受**（第 11 节整段回归盯着 `padUpdate`，为云台加分支会把那些用例的语义搞浑）。**要方形门就另开一次改动** |
| 两浏览器打架 | 独立 `_gimbal_owner`，**第一次拨动摇杆**时抢占，断线释放。⚠️ **不能重用 `_drive_owner`**（它只在 arm 时抢占 ⇒ 会把云台重新绑回 armed）。⚠️ 服务端判据必须是 `request.sid != _gimbal_owner`，**不是** `is not None and …`；非操作者发**零**要丢弃（否则另一台设备的松手会**把别人的云台扯回中**），非操作者发**非零**才抢占 |
| ⚠️ 日志会灌爆网页 | `LOG_FILES` 加了 `("/tmp/gimbal_proc.log","gimbal")`，而 `_push_log` **没有任何限频** ⇒ `gimbal_proc` 按帧打日志会变成消防水管（就是饿死客户端定时器那类事故）⇒ **最多 5 秒一行**（`STATUS_EVERY`）|
| ⚠️ 云台盘**不进** `#joy-mode` | 那个元素下**正好两个按钮**是既有断言 |
| 改前端后**必须**跑 | `node ~/web_drive_test.js`（现 **104 项**）|

### ⚠️ 三项**未定案**，一项都不能靠推理

| 项 | 现在的值 | 证据等级 | 怎么定案 |
|---|---|---|---|
| **串口节点名** | **`/dev/ttyHS1` —— 2026-09-27 上板实测：`open` 成功、`TIOCEXCL` 成功、`tcgetattr` 读回 `8N1 raw baud=115200` 逐位相符** | **高** | 已定案，见 `gimbal_wiring.md` 第三节 |
| **dx/dy 正方向** | `gimbal_invert_x/y: false` | **无** | `python3 ~/gimbal_inject.py --axis-sweep` |
| **绝对量还是增量** | 按绝对量设计 | **无** | `gimbal_proc.py --single-frame 50 0` 只发一帧 |

⚠️ **节点名这条已实测，顺带纠正一个旧说法**：`ttyHS1`（我们这条）确实存在且 `666`；
而 `ttyHS3` 是 `root:root 600` ⇒ **`aidlux` 打不开它**，所以「选错节点 = 日志全绿而云台
不动」这个可怕形状**在本板不成立**，会响亮失败（`EACCES`；`start_gimbal.sh` 预检给退出码 7）。
⚠️ 但那**不等于可以放松**：以 root 跑或以后权限放宽，静默形状立刻回来；而且**真正剩下的
静默风险是「节点对了、排针接反了」** —— 回环证明不了 PIN_29 是 TX 还是 RX。

⚠️ **绝对量/增量没定案之前别改发送节拍**：若接收端把它当**每帧增量**，那**帧率就是协议的
一部分** —— 按 62.5Hz 连发 = 摇杆不动云台也**一直朝一个方向转下去**直到撞限位。真如此，
**必须**改成「只在指令变化时发」，**不是**调 `gimbal_cmd_period_s`。

### ⚠️ 软件只能**证伪**，不能确认（照 `laser_odom_test.py` 的措辞）

| 命题 | 真判据 |
|---|---|
| 节点存在 / 权限 / 能被独占 | `ls -l`、`open` 成功、`TIOCEXCL` 成功 |
| 真的是 115200 8N1 raw | `tcgetattr` **读回**断言具体位；5 秒状态行里**周期性比对**（有 `serial-getty` 抢 tty 就会漂移）|
| 该节点是 pin29/pin16 那一对 | **回环**（临时短接 **PIN_29↔PIN_16**）—— ⚠️⚠️ **不是 29↔30**（那是把驱动器输出直接对地），**测完必须拔掉跳线**。它**排除了 `ttyHS3`**，并**免费验了 raw 配置**：回环载荷**刻意含 `0x0A/0x0D/0x11/0x13/0xFF`**，OPOST 没关则 `0x0A`→`0D 0A`（长度变），IXON 没关则那个 `0x13` 让驱动**当场暂停输出** |
| ⚠️ **pin29 是 TX 而不是 RX** | ⚠️ **回环证明不了**（短接是对称的，两种接法都通）。对**只发**的应用这是致命的：若 TX 其实是 pin16，云台**永远收不到**而回环照样通过。⇒ **只有外部观测**：USB-TTL **只接 GND**、适配器 **RX 接 pin29**（⚠️ **绝不接适配器的 TX**，那会把电平打进排针）|
| 字节真的完整上了线 | **被动监听**最省事也最强：抓 115200 流，断言每段 6 字节 / `0xAA` 开头 / 校验和正确 / **连续帧无缝**（直接验相位）/ 帧率符合预期 |
| 云台真的听话 | **要眼睛** —— 先做绝对/增量实验，再定方向 |
| ⚠️ 编码对不对 | ⚠️ **回环证明不了**（回环用的 `encode_frame` 和被验的是同一个）⇒ 只有 `gimbal_test.py` 里从 C 源码抄的**黄金向量**能判 |

### ⚠️ 接线前的两条硬风险（详见 `gimbal_wiring.md`）

- **电平**：排针是 **3.3V**，而云台侧若是 **5V CMOS**，`VIH ≈ 0.7×VDD ≈ 3.5V` ⇒ 我们的 3.3V
  **卡在阈值下面**。表现是**时好时坏**（比"完全不通"更难查），而丢帧在本协议里**没有重同步兜底**。
  ⚠️ 反过来那一半是安全的：我们**不接**云台的 TX，所以不存在「5V 打进 3.3V 引脚」这条路径
  —— **别把云台的 TX 接到 pin16**。pin29 自 2026-09-27 起**已被云台占用**，不再「空着别动」。
- **供电**：⚠️ **不要**把云台接到排针 5V（pin2/4）—— 那条轨**正给 L298N 逻辑和相机供电，且没有
  任何电流预算文档**。接上去的失败表现是「**相机/SLAM 先出问题**」，**排查方向会完全跑偏**。
  ⇒ 云台走**独立 5V BEC**，但**必须与犀牛派共地**（串口要参考电平）。**软件检测不到低压。**

### ⚠️ 安全边界

端口 **5814 和 5813 一样没有任何认证**。绑回环 ⇒ 边界是「谁能在这台机器上跑进程」。
**不要把 5814 暴露出去或做端口转发** —— 那等于把云台（和电机）交给任何能连上的人。

⚠️ **这是本项目第一个没有 `armed` 门的执行器**，而 `slam_viz.py` **没有认证**（`SECRET_KEY`
是字面量）—— 任何能访问 5800 的人随时能动云台。对相机云台可以接受，但**必须是一个写下来的决定**。

## 相机（2026-09-26 实测：**正常**，且「坏了」是误判）

| 要点 | 说明 |
|------|------|
| ⚠️ **认驱动名，别认 video 编号** | `/dev/videoN` 的编号**不稳定**，至少变过两次。2026-09-15 一度重排成 video0/1/32/33，当时据此记下「相机坏了、`/dev/video2` 不存在了」—— **是误判，别再照它排查**。唯一可靠的认法：`v4l2-ctl --list-devices` 找 **`USB 2.0 Camera: USB Camera`** |
| 节点地图（2026-09-26 实测） | `/dev/video2` = **uvcvideo 相机本体**（MJPG+YUYV）✅；`/dev/video3` = 同一相机的 **UVC metadata 节点**（无格式，`META_CAPTURE`）❌；`/dev/video32/33` = `msm_vidc_decoder`（高通**解码器**）❌；`/dev/video0/1` = 别的东西 ❌ |
| ⚠️ 单读 | 相机同一时刻只能一个进程打开。**`fire_detect.py` 是持有者**，别人再开就 `capture error: open failed`。⚠️ `start_camera.sh` → `camera_preview.py` 是个 **GUI 预览**且循环里**从不 `release()`**，跑起来会**永久占死**相机 ⇒ **别跑它** |
| 为什么必须独立进程 | 同进程 `import rclpy` 之后 cv2 的 V4L2 打不开，ROS pyenv 的 python 一样 ⇒ 相机进程必须 `/usr/bin/python3` 且**不导入 rclpy** |
| 验收（不出帧就不算恢复） | `v4l2-ctl -d /dev/video2 --set-fmt-video=width=640,height=480,pixelformat=MJPG --stream-mmap --stream-count=1 --stream-to=/tmp/p.jpg` —— 退出码 0、几十 KB、首尾是 `\xff\xd8`/`\xff\xd9`。2026-09-26 实做：70168 字节 640×480 ✅ |

## GPIO 权限（2026-09-26 已根治，别再手动 chmod）

`/dev/gpiochip*` 每次重启退回 `root:root 0600` ⇒ `pwm_proc` / `motor_driver` 都打不开，
而报错文案却把人引向「是不是 motor_driver 占着 line」。**已装开机自愈服务，正常情况下什么都不用做。**

| 要点 | 说明 |
|------|------|
| 根因（与旧记录不同） | **不是 udev 规则写错。** `/sys/class/gpio/gpiochip0/uevent` **不存在** ⇒ 内核从不发 gpio 子系统的 uevent ⇒ `/etc/udev/rules.d/99-gpiochardev.rules`（写法正确、`gpio` 组存在、`aidlux` 已在组内 gid 3018、udevd 在跑，四条全齐）**永远不可能被触发**。环境是 **AidLux 容器**（`/proc/1/cgroup` → `aidluxd/init.scope`），`/dev` 是 **tmpfs 而非 devtmpfs**。**别去"修"那条规则**，它是根因线索 |
| 自愈 | `fix-gpio-perms.service`（仿厂商自带的 `chgrp-diag.service`：oneshot + RemainAfterExit + WantedBy=multi-user.target）+ `/usr/local/sbin/fix-gpio-perms.sh`，做 `chgrp gpio` + `chmod 660`，自带最多等 10 秒不依赖启动顺序。**2026-09-26 已重启验证通过** |
| ⚠️ 判据不是返回码 | 脚本以**复核每个节点的实际属主/权限**为成功判据。第一版无条件打印「已修正 N 个」，在无权时照样报成功 —— 那是**会说谎的日志**，装进开机自愈等于埋哑雷。改后坏状态下普通用户跑报 `❌ 10/10 仍不对` 且退出码 1 |
| 验收（不需万用表/硬件/碰电机） | `gpioinfo gpiochip0` —— **注意不加 sudo**。报 `Permission denied` = 没修好；能列出引脚 = 好了 |
| 手动补救 | 先 `systemctl is-active fix-gpio-perms`；再 `sudo systemctl restart fix-gpio-perms`，或 `sudo chgrp gpio /dev/gpiochip* && sudo chmod 660 /dev/gpiochip*`。⚠️ **不再用 `chmod 666`**（全局可写），走 `gpio` 组 `660`，与官方规则本意一致 |
| `start_pwm.sh` 预检 | 启动前先验 `/dev/gpiochip0` 可读写，不可读写则**在停 motor_driver 之前**退出（码 6）—— 否则会留下「旧驱动停了、新驱动没起」的空窗。预检文案直接给出修法。⚠️ 2026-09-27 把判据**加强**了：原先是 `[ ! -r ... ] \|\| [ ! -w ... ]`（只看**权限位**），现在改成**真的按 `pwm_proc` 的方式 `os.open(O_RDWR)` 一次** —— 真正要问的是「内核让不让我开」，权限位只是它的一个原因 |
| ⚠️ 硬件 PWM 这条替代路走不通 | 板上 6 路硬件 PWM **全在 PM8550 PMIC 内部**（LPG 灯效发生器，`/sys/class/pwm/pwmchip0\|5`），设备树里**无任何 SoC 侧 PWM 节点**，**接不到电机所用的 GPIO 91/63**。`/sys/class/leds/red` 的真实身份是 `qcom,leds@ef00`（板载状态灯）。⇒ **软 PWM 是既定方案，没有硬件 PWM 可换** |
| ⚠️ `pgrep -f` 假警报 | 在 ssh 里跑 `pgrep -f "pwm_proc"` 会命中**你自己那条 ssh 命令行**（字符串里含 pattern）—— 2026-09-26 亲历一次假警报，误报「pwm_proc 竟然起来了」。判据必须锚定行首：`pgrep -f '^[^ ]*python3 .*pwm_proc\.py'` |
| ⚠️ `rc.local` 不可靠 | `rc-local` 是 `enabled-runtime` 且 `/run/systemd/system/` 下无符号链接 ⇒ **重启后不保证执行**。别拿它当开机钩子 |

## 注意事项

- IP DHCP 可能变，当前 `.220`（用过 .218/.222）
- 设备重启后需重新插拔 RPLidar USB
- 不需要 docker、rosdep init（AidLux 已配置）
- `_ssh_login.py` 依赖 `pty`（Windows 不可用）
