# 相机（fire_detect 是持有者）

> 代码位置：`~/robot/camera/`。`~/` 下的同名文件是软链，旧命令照常可用。

## 相机（2026-09-26 实测：**正常**，且「坏了」是误判）

| 要点 | 说明 |
|------|------|
| ⚠️ **认驱动名，别认 video 编号** | `/dev/videoN` 的编号**不稳定**，至少变过两次。2026-09-15 一度重排成 video0/1/32/33，当时据此记下「相机坏了、`/dev/video2` 不存在了」—— **是误判，别再照它排查**。唯一可靠的认法：`v4l2-ctl --list-devices` 找 **`USB 2.0 Camera: USB Camera`** |
| 节点地图（2026-09-26 实测） | `/dev/video2` = **uvcvideo 相机本体**（MJPG+YUYV）✅；`/dev/video3` = 同一相机的 **UVC metadata 节点**（无格式，`META_CAPTURE`）❌；`/dev/video32/33` = `msm_vidc_decoder`（高通**解码器**）❌；`/dev/video0/1` = 别的东西 ❌ |
| ⚠️ 单读 | 相机同一时刻只能一个进程打开。**`fire_detect.py` 是持有者**，别人再开就 `capture error: open failed`。⚠️ `start_camera.sh` → `camera_preview.py` 是个 **GUI 预览**且循环里**从不 `release()`**，跑起来会**永久占死**相机 ⇒ **别跑它** |
| 为什么必须独立进程 | 同进程 `import rclpy` 之后 cv2 的 V4L2 打不开，ROS pyenv 的 python 一样 ⇒ 相机进程必须 `/usr/bin/python3` 且**不导入 rclpy** |
| 验收（不出帧就不算恢复） | `v4l2-ctl -d /dev/video2 --set-fmt-video=width=640,height=480,pixelformat=MJPG --stream-mmap --stream-count=1 --stream-to=/tmp/p.jpg` —— 退出码 0、几十 KB、首尾是 `\xff\xd8`/`\xff\xd9`。2026-09-26 实做：70168 字节 640×480 ✅ |

## 相关文件

| 文件 | 用途 |
|------|------|
| `camera/fire_detect.py` | **相机的实际持有者** (:5812)，系统 python3 跑、不导入 rclpy |
| `camera/start_fire_detect.sh` | 启动 fire_detect（`setsid nohup`，独立进程） |
| `camera/camera_test.py` `camera/show_camera.py` | 一次性相机测试 / 预览工具 |
| `deprecated/cam_stream.py` | **已被 `fire_detect.py` 取代** (:5811 仍在但没人拉) |
| `deprecated/start_camera.sh` | ⚠️ **已弃用/有害** —— 起的是 `camera_preview.py` |
| `deprecated/camera_preview.py` | GUI 预览，循环里**从不 `release()`**，跑起来会**永久占死**相机 ⇒ 别跑 |
