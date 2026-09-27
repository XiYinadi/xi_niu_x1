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

## 目录结构（2026-09-27 重组）

代码从 `~/` 平铺重组到 `~/robot/` 子目录，`~/` 下留**软链**兼容旧命令
（`bash ~/mapping_ctl.sh`、`node ~/web_drive_test.js` 等**照常可用**，一个字没改）。
**新代码一律用 `~/robot/...` 真路径**，软链只作兼容层、逐步退役。

```
~/robot/
├── slam/     建图管线        → 见 slam/README.md
├── motor/    电机+直连 PWM   → 见 motor/README.md
├── gimbal/   二维云台        → 见 gimbal/README.md
├── camera/   相机(fire_detect)→ 见 camera/README.md
├── web/      slam_viz + 前端测试(web_drive_test.js)
├── tools/    诊断/探测工具
├── ai/       LLM 相关(与 SLAM 无关)
├── deprecated/  已废弃归档(含 git 前的 .bak 快照)
├── docs/     本文档 + 各子系统 README
├── data/     fire_events/ 运行数据
└── logs/     历史日志
```

- ROS 工作空间 `~/lidar_ws/` **不在** `~/robot/` 内（colcon 需要原布局，保持原位）。
- git：本仓库 `~/robot/.git`（**本地，无远端**）。⚠️ 密钥文件 `ai/claude_config.sh`、
  `ai/litellm_config.yaml`、`ai/api_proxy.py` 已 `.gitignore`，**不进历史**。

## 工具 / 废弃文件

- `tools/`：`diag_client.py`（SocketIO 诊断客户端，按 Hz 灌 vel）、`rate_probe.py`（采样回调间隔，
  发现控制环被饿死）、`topic_probe.py`、`cmd_log.py`、`enc_test.py`、`gpio_smoke.py`、
  `pin_hold.py`、`motor_probe.py`、`start_vnc.sh`。
- `deprecated/`：`rplidar_web.py`/`slam_web.py`/`slam_bridge.py`（旧版 Web/桥接）、
  `cam_stream.py`/`camera_preview.py`/`start_camera.sh`（旧相机链路，见 camera/README.md）、
  `test*.jpg`（散落测试图）、`CLAUDE.md.bak-20260926` 与 `bak-20260927-gimbal/`（git 建立前的快照）。
- `ai/`：`api_proxy.py`、`litellm_config.yaml`、`claude_config.sh`、`_vlm_*`、`_sock_*`、
  `_api_fg_diag.sh` —— LLM 相关，与 SLAM 无关。

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
