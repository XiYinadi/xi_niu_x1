#!/usr/bin/env python3
"""
slam_viz.py — 犀牛派 SLAM 实时 Web 可视化
===========================================
单文件方案: ROS2 订阅 + Flask-SocketIO WebSocket 推送 + 平铺多窗口前端
窗口: 地图 / 相机 / 运行日志 (三窗口平铺网格)

相机画面: fire_detect.py 独占 USB 相机 → JPEG → slam_viz 按订阅转发 SocketIO
运行日志: tail 各管线进程日志文件 → SocketIO 推送

依赖 (全部已安装在犀牛派):
  rclpy, nav_msgs, sensor_msgs, numpy, Pillow, flask, flask-socketio

用法:
  source /opt/ros/humble/setup.bash
  source ~/lidar_ws/install/setup.bash
  python3 slam_viz.py
  浏览器打开 http://<LAN-IP>:5800
"""

import base64
import io
import json
import logging
import math
import os
import re
import signal
import socket
import subprocess
import sys
import threading
import time
import urllib.request
from collections import deque

# AidLux: ROS2 env overrides PYTHONPATH, Flask/Pillow are in system site-packages
for site_pkg in [
    "/usr/local/lib/python3.10/dist-packages",
    "/usr/lib/python3/dist-packages",
]:
    if site_pkg not in sys.path and os.path.isdir(site_pkg):
        sys.path.insert(0, site_pkg)

import numpy as np
from PIL import Image

# ── Flask + SocketIO ──────────────────────────────────────────────
from flask import Flask, redirect, render_template_string, request, send_file
from flask_socketio import SocketIO, join_room, leave_room
from werkzeug.serving import WSGIRequestHandler


class RealtimeRequestHandler(WSGIRequestHandler):
    # Send frame/command ACKs promptly on both WebSocket and polling sockets.
    disable_nagle_algorithm = True

# ── ROS2 ───────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("slam_viz")

# ⚠️ Werkzeug 的访问日志会经 root logger 落进 /tmp/slam_viz.log —— 而那个文件
#    正是 log_tail_loop 要 tail 的(LOG_FILES 里的 "viz")。于是形成自持循环:
#        每次 HTTP 请求(含 Socket.IO 轮询的 GET/POST)被写进日志
#          → log_tail_loop 读到 → socketio.emit("log", …) 推给客户端
#          → 这条事件本身又必须靠一次 GET 轮询投递 → 那次轮询又被记录 → …
#    每一轮请求都必然生产一条消息,每条消息又必须靠一轮请求投递,**永不衰减**。
#    后果:客户端主线程被 log DOM 更新淹没,60Hz 的摇杆定时器被饿死到 4~18Hz
#    (2026-09-15 用 diag_client.py 实测复现)。必须在起服务前把 werkzeug 掐掉。
logging.getLogger("werkzeug").setLevel(logging.ERROR)

# ── Constants ──────────────────────────────────────────────────────
PORT = int(os.environ.get("SLAM_VIZ_PORT", 5800))

# ── Motor control limits (must match motor_params.yaml) ─────────────
MAX_LIN = float(os.environ.get("SLAM_VIZ_MAX_LIN", 0.3))   # m/s
MAX_ANG = float(os.environ.get("SLAM_VIZ_MAX_ANG", 1.0))   # rad/s

# ── 直接 PWM 通道 (绕开 ROS) ────────────────────────────────────────
# 摇杆值除了走下面的 /cmd_vel,还会以 UDP 直发 pwm_proc(独立进程,自己的
# GIL)。为什么另开一条:原来自进程内的 PWM 忙等线程和 rclpy 执行器抢同一个
# GIL,把控制回调间歇性饿死 —— 实测 /wheel_speed 间隔中位 20.3ms 但
# p90 116.6ms / p99 466.9ms,松手后 duty 要连降数步、步步卡住,叠加成 931ms。
#
# ⚠️ 另开进程只解决了「和 rclpy 抢」这一半。**进程内**还有一次同样的饿死:
#    PWM 线程对一个完整 PWM 周期纯自旋,把 pwm_proc 自己的主循环从 200Hz 饿到
#    1~2Hz(日志 `轮询 2Hz/目标200Hz`),并撑爆 UDP 接收缓冲(发 60 收 24)。
#    真正的修法在 motor_hw.DualSoftPwm._busy_until(粗睡 + 末段精等),不是拆分进程。
# 详见 pwm_proc.py 模块注释。
# 没有 pwm_proc 在跑时 UDP 只是丢包,不影响 ROS 那条路,所以这里不发探测包。
PWM_UDP_HOST = os.environ.get("SLAM_VIZ_PWM_HOST", "127.0.0.1")
PWM_UDP_PORT = int(os.environ.get("SLAM_VIZ_PWM_PORT", 5813))
_pwm_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
log.info("直连 PWM 通道 → udp://%s:%d (需另起 pwm_proc.py: bash ~/start_pwm.sh)",
         PWM_UDP_HOST, PWM_UDP_PORT)

# ── 二维云台串口通道(独立进程 gimbal_proc.py,不碰 GPIO)──────────────
# 云台走**串口**(40-pin 排针 PIN_29/PIN_30),和电机那条 GPIO 链**零资源共享**
# ⇒ 这里单独一个 socket、单独一个进程、单独一份参数(gimbal_params.yaml)。
#
# ⚠️⚠️ **云台没有 armed 门** —— 这是刻意的设计决定:转相机 ≠ 驱动轮子。
#    它和「不点启用就能驱动电机」那个已修的洞**是两回事**,别照搬那套门控。
# ⚠️ 代价必须写下来:5800 上**没有认证**(SECRET_KEY 是字面量),所以任何能打开
#    这个网页的人都能动云台。对相机云台可以接受,但这是一个**决定**,不是疏忽。
# ⚠️ 5814 和 5813 一样**无任何认证**。绑回环 ⇒ 边界是「谁能在这台机器上跑进程」。
#    **不要**把 5814 暴露出去或做端口转发。
GIMBAL_UDP_HOST = os.environ.get("SLAM_VIZ_GIMBAL_HOST", "127.0.0.1")
GIMBAL_UDP_PORT = int(os.environ.get("SLAM_VIZ_GIMBAL_PORT", 5814))
_gimbal_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
log.info("云台串口通道 → udp://%s:%d (需另起 gimbal_proc.py: bash ~/start_gimbal.sh)",
         GIMBAL_UDP_HOST, GIMBAL_UDP_PORT)

# ── Camera config (env-overridable) ───────────────────────────────
# 相机帧来自独立进程 fire_detect.py 的原始帧端点(相机是单读设备,
# fire_detect 独占 /dev/video2,故不再用 cam_stream;ROS env 会破坏 cv2 的 V4L2)
CAMERA_STREAM_URL = os.environ.get(
    "SLAM_VIZ_CAM_URL", "http://127.0.0.1:5812/raw.jpg"
)
CAMERA_FPS = max(1, min(60, int(os.environ.get("SLAM_VIZ_CAM_FPS", 30))))

# ── Fire detection config (env-overridable) ─────────────────────────
# 火情标注帧/状态来自独立进程 fire_detect.py(ROS env 会破坏 cv2 的 V4L2,故分离)
FIRE_STREAM_URL = os.environ.get(
    "SLAM_VIZ_FIRE_URL", "http://127.0.0.1:5812/frame.jpg"
)
FIRE_STATUS_URL = os.environ.get(
    "SLAM_VIZ_FIRE_STATUS_URL", "http://127.0.0.1:5812/"
)
FIRE_STATUS_INTERVAL = float(
    os.environ.get("SLAM_VIZ_FIRE_STATUS_S", 2.0)
)  # 状态 JSON 拉取间隔

# ── Log tail config (env-overridable) ─────────────────────────────
LOG_FILES = [
    ("/tmp/navigation.log", "nav"),
    ("/tmp/slam_viz.log", "viz"),
    ("/tmp/motor_driver.log", "motor"),
    ("/tmp/laser_odom.log", "odom"),
    ("/tmp/slam.log", "slam"),
    ("/tmp/lidar.log", "lidar"),
    ("/tmp/tf_laser.log", "tf"),
    ("/tmp/gimbal_proc.log", "gimbal"),
]
LOG_BUFFER_SIZE = int(os.environ.get("SLAM_VIZ_LOG_BUF", 300))

# 轨迹最多留这么多点(和客户端一致)。超出后裁掉最旧的,客户端靠 base 变化
# 知道该整体重建(见 get_state_snapshot 里 reset 的判据)。
TRAJ_MAX = 5000


# ══════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════

def occgrid_to_png_base64(msg: OccupancyGrid) -> dict:
    """Convert OccupancyGrid → base64 PNG + metadata dict.

    Returns dict with keys: png (base64 str), width, height, resolution,
    origin_x, origin_y.
    """
    w, h = msg.info.width, msg.info.height
    if w == 0 or h == 0:
        return None

    data = np.array(msg.data, dtype=np.int8).reshape(h, w)
    img = np.zeros((h, w, 3), dtype=np.uint8)

    # Unknown cells (-1): subtle dark bluish-gray
    img[data == -1] = [25, 28, 36]

    # Known cells: grayscale 254 (free/white) → 0 (occupied/black)
    known = data >= 0
    val = (254 - data[known] * 2.54).clip(0, 254).astype(np.uint8)
    for c in range(3):
        img[known, c] = val

    # Flip: ROS row 0 = bottom edge, PNG row 0 = top
    img = np.flipud(img)

    pil_img = Image.fromarray(img, "RGB")
    buf = io.BytesIO()
    pil_img.save(buf, format="PNG", optimize=True)
    b64 = base64.b64encode(buf.getvalue()).decode("ascii")

    return {
        "png": b64,
        "width": w,
        "height": h,
        "resolution": msg.info.resolution,
        "origin_x": msg.info.origin.position.x,
        "origin_y": msg.info.origin.position.y,
    }


def laser_scan_to_xy(msg: LaserScan) -> list:
    """Convert LaserScan ranges → flat [x1, y1, x2, y2, ...] in laser frame.

    Filters out-of-range and non-finite values. Returns empty list if
    no valid points.
    """
    points = []
    angle = msg.angle_min
    r_min, r_max = msg.range_min, msg.range_max
    for r in msg.ranges:
        if r_min < r < r_max and math.isfinite(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.extend([round(x, 4), round(y, 4)])
        angle += msg.angle_increment
    return points


# ══════════════════════════════════════════════════════════════════════
# ROS2 Node
# ══════════════════════════════════════════════════════════════════════

class SlamVizNode(Node):
    """ROS2 node that subscribes to /map and /scan, makes data available
    thread-safe to the broadcast loop."""

    def __init__(self):
        super().__init__("slam_viz_node")

        self._lock = threading.Lock()

        # Shared state
        self._latest_map_msg = None
        self._latest_scan_xy = []
        self._scan_stamp = None
        self._scan_frame = "laser"
        self._map_dirty = False
        self._scan_count = 0    # /scan 变更序号(broadcast_loop 用它判「新一帧」)

        # 「建图到底在不在跑」只能靠**数据流**判 —— 计数器只能证明「曾经收到过」,
        # 证明不了「现在还在收」。0.0 表示从没收到过(monotonic 单调,不会是 0)。
        self._last_scan_t = 0.0
        self._last_map_t = 0.0
        self._last_odom_t = 0.0
        self._odometry = None
        self._last_imu_t = 0.0
        self._imu_acceleration = None
        self._imu_gyro = None
        self._imu_gyro_t = 0.0
        self._imu_frame = None

        # QoS: /map needs transient_local to receive last map immediately
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        # Control/health consumers need the newest sample, never a queued history
        # replayed after a slow callback or a scheduling pause.
        scan_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )
        # Sensor receipt must continue while a navigation/service callback runs.
        # A separate group plus two executor workers provides that isolation.
        self._sensor_callbacks = MutuallyExclusiveCallbackGroup()

        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, scan_qos,
            callback_group=self._sensor_callbacks,
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, scan_qos,
            callback_group=self._sensor_callbacks,
        )
        self._imu_sub = self.create_subscription(
            Imu, "/imu/data_raw", self._imu_cb, scan_qos,
            callback_group=self._sensor_callbacks,
        )

        # Motor control publishers (Web → /cmd_vel + /motor_arm)
        self._twist_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        arm_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._arm_pub = self.create_publisher(Bool, "/motor_arm", arm_qos)
        self._twist_lock = threading.Lock()
        self._pending_twist = None
        self._pending_arm = None
        # HTTP/socket handlers must never wait inside a reliable DDS publish.
        # Mirror only the latest command on the ROS executor; direct PWM is immediate.
        self._drive_mirror_timer = self.create_timer(.02, self._flush_drive_mirror)

        # Trajectory: list of [x, y] tuples in odom frame
        self._trajectory = []  # up to TRAJ_MAX points
        # 累计追加过的点数(**不随裁剪减少**),以及已经发给客户端的点数。
        # 两者之差就是「本次要补发的增量」;差值若大于现存点数,说明中间被裁过,
        # 客户端那边有洞,必须整体重建而不是追加。
        self._traj_total = 0
        self._traj_sent = 0
        self._latest_pose = None  # (x, y, yaw) for the HUD

        log.info("SlamVizNode ready — subscribing /map + /scan + /odom")

    def publish_cmd_vel(self, linear, angular):
        """Scale normalized (-1..1) inputs to m/s & rad/s, publish /cmd_vel."""
        t = Twist()
        t.linear.x = max(-1.0, min(1.0, float(linear))) * MAX_LIN
        t.angular.z = max(-1.0, min(1.0, float(angular))) * MAX_ANG
        with self._twist_lock:
            self._pending_twist = (t, time.monotonic())

    def set_motor_arm(self, on):
        b = Bool()
        b.data = bool(on)
        with self._twist_lock:
            self._pending_arm = (b, time.monotonic())

    def _flush_drive_mirror(self):
        with self._twist_lock:
            twist, arm = self._pending_twist, self._pending_arm
            self._pending_twist = self._pending_arm = None
        now = time.monotonic()
        if twist is not None:
            msg, received = twist
            if now-received > .2:
                msg = Twist()  # Never replay an old ROS motion after executor starvation.
            self._twist_pub.publish(msg)
        if arm is not None:
            msg, received = arm
            if msg.data and now-received > .2:
                msg = Bool()
            self._arm_pub.publish(msg)

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._latest_map_msg = msg
            self._map_dirty = True
            self._last_map_t = time.monotonic()

    def _scan_cb(self, msg: LaserScan):
        # Convert outside lock (CPU work, no shared state access)
        xy = laser_scan_to_xy(msg)
        with self._lock:
            self._latest_scan_xy = xy
            self._scan_stamp = msg.header.stamp
            self._scan_frame = msg.header.frame_id
            self._scan_count += 1
            self._last_scan_t = time.monotonic()

    def _imu_cb(self, msg: Imu):
        values = [float(msg.linear_acceleration.x), float(msg.linear_acceleration.y),
                  float(msg.linear_acceleration.z)]
        if msg.linear_acceleration_covariance[0] == -1 or not all(math.isfinite(v) for v in values):
            return
        with self._lock:
            self._last_imu_t = time.monotonic()
            self._imu_acceleration = values
            self._imu_frame = msg.header.frame_id
            g = (float(msg.angular_velocity.x), float(msg.angular_velocity.y),
                 float(msg.angular_velocity.z))
            if all(math.isfinite(v) for v in g):
                self._imu_gyro = g
                self._imu_gyro_t = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        wz = msg.twist.twist.angular.z
        if not all(math.isfinite(v) for v in (x, y, q.x, q.y, q.z, q.w, vx, vy, wz)):
            return
        if abs(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w - 1.0) > .01:
            return
        # yaw from quaternion (ZYX convention)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self._lock:
            now = time.monotonic()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            frame = (msg.header.frame_id, msg.child_frame_id)
            previous = self._odometry
            # A source interruption or clock/frame reset invalidates a browser's
            # measurement origin. Map/SLAM corrections never change this raw pose.
            restarted = (previous is None or now - self._last_odom_t >= 1.0
                         or frame != (previous['frame'], previous['child_frame'])
                         or stamp < previous['stamp'])
            self._odometry = dict(x=x, y=y, yaw=yaw, vx=vx, vy=vy, wz=wz,
                                  frame=frame[0], child_frame=frame[1], stamp=stamp,
                                  generation=now if restarted else previous['generation'])
            # 存的时候就四舍五入到 3 位(1mm,远超地图精度):全精度浮点
            # 像 1.2345678901234567 每个 18 字节,是纯浪费 —— 单条轨迹本来
            # 能到 ~200KB,砍掉 2/3 是在这里而不是在发送端。
            self._trajectory.append((round(x, 3), round(y, 3)))
            self._traj_total += 1
            if len(self._trajectory) > TRAJ_MAX:
                self._trajectory = self._trajectory[-TRAJ_MAX:]
            self._latest_pose = (x, y, yaw)
            self._last_odom_t = now

    def full_trajectory(self, map_frame=True) -> dict:
        """整条轨迹,只给新连上来的客户端补历史用。

        **不动 `_traj_sent`** —— 那个游标只属于广播循环的增量语义。这里如果
        也去推进它,广播那边就会以为新客户端已经拿过历史,后续增量发不全。
        """
        with self._lock:
            if not self._trajectory:
                return None
            history = {
                "reset": True,          # 让客户端整体重建,而不是追加
                "from": self._traj_total - len(self._trajectory),
                "points": [c for pt in self._trajectory for c in pt],
                "total": self._traj_total,
            }

        if map_frame:
            if _navigation is None:
                return None
            return _navigation.transform_snapshot({'trajectory': history}).get('trajectory')
        return history

    def get_state_snapshot(self) -> dict:
        """Thread-safe snapshot. Returns dict with optional 'map' and 'scan' keys."""
        result = {}
        with self._lock:
            # Map: only encode PNG if dirty (expensive op outside lock)
            if self._map_dirty and self._latest_map_msg is not None:
                msg = self._latest_map_msg
                self._map_dirty = False
                # Release lock briefly? No — we do encoding outside below
                result["_map_msg"] = msg

            if self._latest_scan_xy:
                # Copy reference atomically
                result["scan"] = {
                    "points": self._latest_scan_xy,
                    "stamp": self._scan_stamp,
                    "frame": self._scan_frame,
                    "count": len(self._latest_scan_xy) // 2,
                    "seq": self._scan_count,   # 变更序号:broadcast 用它判「新一帧」
                }

            # Trajectory: 只给**增量**,没有新点就整个不放进 result
            # (广播循环是「有 key 才发」,于是没新点自然不发 —— 原来是无条件
            #  重建整个 10000 个浮点的扁平列表并每轮 20Hz 全量重发,约 4MB/s,
            #  注释写的「只在有新点时发」根本不成立)。
            n = len(self._trajectory)
            if n:
                base = self._traj_total - n       # 已被裁掉的旧点数
                if base > self._traj_sent:
                    # 客户端手里的数据中间断了,只能整体重建
                    new = self._trajectory
                    reset = True
                else:
                    new = self._trajectory[self._traj_sent - base:]
                    reset = False
                if new:
                    # from = new[0] 的**累计序号**(不是本次下标)。
                    # reset 时窗口整体从 base 开始;增量时从 _traj_sent 开始。
                    # 带上它,刚连上来的客户端就能算出「这几条我已经有了」——
                    # 连接时补发的全量快照和紧随其后的第一次增量必然会重叠几条,
                    # 没有这个序号就会重复追加成折线里的重复顶点。
                    result["trajectory"] = {
                        "reset": reset,
                        "from": base if reset else self._traj_sent,
                        "points": [c for pt in new for c in pt],
                        "total": self._traj_total,
                    }
                    self._traj_sent = self._traj_total

            # Latest pose for the HUD / follow mode
            if self._latest_pose is not None:
                px, py, pyaw = self._latest_pose
                result["pose"] = {
                    "x": round(px, 3),
                    "y": round(py, 3),
                    "yaw": round(pyaw, 3),
                }

            # 建图状态判据(见 mapping_status)。feed 是**独立键** —— 广播循环只转发
            # map/scan/trajectory/pose,feed 只在 _map_state_payload 里被消费,不进客户端。
            now = time.monotonic()
            result["feed"] = {
                "scan_age": (now - self._last_scan_t) if self._last_scan_t else None,
                "map_age": (now - self._last_map_t) if self._last_map_t else None,
                "odom_age": (now - self._last_odom_t) if self._last_odom_t else None,
                "odometry": self._odometry,
                "imu_age": (now - self._last_imu_t) if self._last_imu_t else None,
                "imu_acceleration": self._imu_acceleration,
                "imu_frame": self._imu_frame,
            }

        # Encode map PNG outside lock (CPU-heavy, no shared state)
        if "_map_msg" in result:
            map_data = occgrid_to_png_base64(result.pop("_map_msg"))
            if map_data:
                result["map"] = map_data

        if _navigation is not None:
            result = _navigation.transform_snapshot(result)
        else:
            for key in ('scan', 'pose', 'trajectory'):
                result.pop(key, None)
        return result


# ══════════════════════════════════════════════════════════════════════
# Flask + SocketIO
# ══════════════════════════════════════════════════════════════════════

app = Flask(__name__)
app.config["SECRET_KEY"] = "CHANGE-ME-public"
socketio = SocketIO(
    app,
    async_mode="threading",
    cors_allowed_origins="*",
    logger=False,
    engineio_logger=False,
)


@app.route("/")
@app.route("/mapping")
def index():
    return render_template_string(HTML_TEMPLATE, page_class="mapping-page")


@app.route("/console")
def console():
    return redirect('/mapping')


@app.route("/control")
def control():
    return render_template_string(HTML_TEMPLATE, page_class="control-page")


@app.route('/explore')
@app.route('/patrol')
@app.route('/navigation')
def mission_page():
    mode = {'/explore':'explore-page', '/patrol':'patrol-page', '/navigation':'navigation-page'}[request.path]
    return render_template_string(HTML_TEMPLATE, page_class='mission-page '+mode)


@app.route('/missions.css')
@app.route('/missions.js')
@app.route('/navigation_page.js')
def mission_asset():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)), request.path.lstrip('/')))


@app.get('/api/missions/readiness')
def mission_readiness():
    config = _navigation.config if _navigation is not None else {}
    error = _navigation.calibration_error() if _navigation is not None else '任务组件尚未就绪'
    keys = ('wheelbase_m','length_m','width_m','max_steering_rad','full_throttle_mps','max_speed_mps','trial_max_steering_rad','trial_pwm_cap')
    measurements = {key:config.get(key) if type(config.get(key)) in (int, float)
                    and math.isfinite(config[key]) else None for key in keys}
    result = dict(calibrated=error is None, calibration_error=error,
                  drive_error=_navigation.drive_error() if _navigation is not None else error,
                  drive_mode='supervised_trial' if config.get('navigation_mode')=='supervised_trial' else 'calibrated',
                  measurements=measurements)
    return app.response_class(json.dumps(result, allow_nan=False), mimetype='application/json')


@app.route('/control.css')
@app.route('/control.js')
def control_asset():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                 request.path.lstrip('/')))


@app.route('/mapping.css')
@app.route('/mapping.js')
def mapping_asset():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                 request.path.lstrip('/')))


@app.route('/vendor/socket.io.min.js')
def socketio_client_asset():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                 'vendor', 'socket.io.min.js'), mimetype='text/javascript')


@app.route('/navigation.js')
def navigation_script():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'navigation.js'), mimetype='text/javascript')


@app.route('/autonomy.js')
def autonomy_script():
    return send_file(os.path.join(os.path.dirname(os.path.realpath(__file__)), 'autonomy.js'), mimetype='text/javascript')


def _automatic_active():
    return any(getattr(getattr(_navigation,name,None),'active',False) is True
               for name in ('autonomy','patrol_runner'))


@socketio.on('navigation_control')
def navigation_control(data):
    if _navigation is None:
        socketio.emit('navigation_error', {'message': _navigation_error or '导航尚未就绪'}, to=request.sid)
        return
    try:
        if not isinstance(data, dict):
            raise ValueError('导航请求格式无效')
        with _map_lock:
            if _map_busy:
                raise ValueError('建图操作正在进行，请稍候')
            _navigation.control(data, request.sid)
    except Exception as e:
        socketio.emit('navigation_error', {'message': str(e)}, to=request.sid)
    if data.get('action') != 'heartbeat':
        _navigation.emit()


def _navigation_disarm():
    global _motor_armed, _drive_owner, _drive_owner_client
    _motor_armed = False
    _drive_owner = None
    _drive_owner_client = None
    _send_drive(0.0, 0.0, False)
    _ros_node.set_motor_arm(False)
    _ros_node.publish_cmd_vel(0.0, 0.0)
    _emit_motor_state()



# ══════════════════════════════════════════════════════════════════════
# 建图状态 (数据流判定) + 启停
# ══════════════════════════════════════════════════════════════════════

# 状态判据。**这是整个按钮唯一被允许说谎的地方,所以判据要尽可能硬**:
#   mapping    = 地图在更新。这是**唯一**的端到端证据 —— 要出图,雷达 + 激光里程计
#                + 静态 TF + slam_toolbox 必须**全部**活着,缺一个都不出图。
#                所以它不可能像「进程起来了」那样假装在跑。
#   lidar_only = 雷达在出数据但没在出图。半启动/中途崩的样子,也正是最容易被人
#                误当成「建图中」的那个状态。
#   off        = 两个都没有。
SCAN_FRESH_S = 2.0    # 雷达 10Hz,2 秒够容忍一次卡顿/丢包
MAP_FRESH_S = 10.0    # mapping_no_odom.yaml 里 map_update_interval=3.0,给 3 倍余量


def mapping_status(scan_age, map_age, odom_age=None) -> str:
    """age 单位秒;None = 从没收到过。纯函数,可 headless 单测。"""
    if (map_age is not None and 0 <= map_age < MAP_FRESH_S
            and scan_age is not None and 0 <= scan_age < SCAN_FRESH_S
            and odom_age is not None and 0 <= odom_age < 1.0):
        return "mapping"
    if scan_age is not None and scan_age < SCAN_FRESH_S:
        return "lidar_only"
    return "off"


MAPPING_CTL = os.environ.get("MAPPING_CTL", os.path.expanduser("~/mapping_ctl.sh"))
MAPPING_TIMEOUT_S = 45

_map_lock = threading.Lock()
_map_busy = False       # 一次只允许一个 start/stop 在路上
_map_action = None      # 正在进行的是什么,给 UI 显示「启动中…」用
_map_last = {"scan_age": None, "map_age": None, "odom_age": None, "imu_age": None,
             "imu_acceleration": None, "imu_frame": None, "odometry": None}


def _map_state_payload():
    managed = _navigation is not None and _navigation.running() and getattr(_navigation,'mode',None)=='mapping'
    status = ("localization" if _navigation is not None and _navigation.running() and not managed
              else mapping_status(_map_last["scan_age"], _map_last["map_age"], _map_last["odom_age"]))
    return {
        "status": status,
        "scan_age": _map_last["scan_age"],
        "map_age": _map_last["map_age"],
        "odom_age": _map_last["odom_age"],
        "odometry": _map_last.get("odometry"),
        "imu_age": _map_last["imu_age"],
        "imu_acceleration": _map_last.get("imu_acceleration"),
        "imu_frame": _map_last.get("imu_frame"),
        "busy": _map_busy,
        "action": _map_action,
        "managed": managed,
    }


def _emit_mapping_state():
    socketio.emit("mapping_state", _map_state_payload())


def _mapping_run(action: str):
    """跑 mapping_ctl.sh。在**后台线程**里跑:handler 阻塞会卡住那次长轮询请求
    (传输是 polling,每个事件一次 HTTP 往返),一个几秒的 start 会让整个页面顿住。

    脚本的输出**逐行进日志窗口** —— 于是「雷达没插」「motor_driver 在跑」这类
    失败直接显在网页上,不用去 ssh 看。
    """
    global _map_busy, _map_action
    try:
        if not os.path.exists(MAPPING_CTL):
            _push_log("slam", "[ERROR] 找不到 %s" % MAPPING_CTL)
            return
        _push_log("slam", "%s mapping_ctl.sh %s ..." % (
            "==>" if action == "start" else "<==", action))
        r = subprocess.run(
            ["bash", MAPPING_CTL, action],
            capture_output=True, text=True, timeout=MAPPING_TIMEOUT_S,
        )
        for stream in (r.stdout, r.stderr):
            for line in (stream or "").splitlines():
                if line.strip():
                    _push_log("slam", line)
        if r.returncode != 0:
            log.warning("mapping_ctl %s 退出码 %d", action, r.returncode)
    except subprocess.TimeoutExpired:
        _push_log("slam", "[ERROR] mapping_ctl.sh %s 超时(%ds)" % (action, MAPPING_TIMEOUT_S))
        log.warning("mapping_ctl %s 超时", action)
    except Exception as e:
        _push_log("slam", "[ERROR] mapping_ctl.sh %s 异常: %s" % (action, e))
        log.exception("mapping_ctl failed")
    finally:
        with _map_lock:
            _map_busy = False
            _map_action = None
        # 立刻推一次。注意这里**不猜**结果 —— 状态仍然由数据流算出来,
        # 下面那一发只是让按钮尽快从「启动中…」回来,值本身还是数据流说了算。
        _emit_mapping_state()


def _mapping_request(action: str) -> bool:
    """取锁 → 起后台线程。已在忙时返回 False(双击不能起两份:两个 slam_toolbox
    会同时往 /map 和 TF 上写,是那种**不报错**的坏)。"""
    global _map_busy, _map_action
    with _map_lock:
        if _map_busy:
            return False
        if _navigation is not None and (_navigation.running() or _navigation.busy):
            return False
        _map_busy = True
        _map_action = action
    threading.Thread(target=_mapping_run, args=(action,), daemon=True).start()
    return True


# ══════════════════════════════════════════════════════════════════════
# Broadcast Loop (SocketIO background task)
# ══════════════════════════════════════════════════════════════════════

_ros_node: SlamVizNode = None  # set by main()
_navigation = None
_navigation_error = None


def broadcast_loop():
    """SocketIO background task: poll ROS state → WebSocket emit."""
    last_map_emit = 0.0
    last_scan_seq = None
    last_pose_key = None
    last_map_status = None
    last_health_emit = 0.0
    log.info("Broadcast loop started")
    while True:
        try:
            state = _ros_node.get_state_snapshot()

            # 建图状态:每轮都重算(状态会**在没人发包的时候**变陈旧,所以不能只在
            # 收到数据时算)。状态变更立即推送，数据年龄最多每 250ms 更新一次。
            feed = state.get("feed")
            if feed:
                _map_last["scan_age"] = feed["scan_age"]
                _map_last["map_age"] = feed["map_age"]
                _map_last["odom_age"] = feed["odom_age"]
                _map_last["odometry"] = feed.get("odometry")
                _map_last["imu_age"] = feed["imu_age"]
                _map_last["imu_acceleration"] = feed.get("imu_acceleration")
                _map_last["imu_frame"] = feed.get("imu_frame")
            p = _map_state_payload()
            key = (p["status"], p["busy"], p["action"],p['managed'])
            if key != last_map_status or time.monotonic() - last_health_emit >= 0.25:
                socketio.emit("mapping_state", p)
                last_health_emit = time.monotonic()
            if key != last_map_status:
                last_map_status = key
                log.info("建图状态 → %s (scan_age=%s map_age=%s busy=%s)",
                         p["status"],
                         "—" if p["scan_age"] is None else "%.1fs" % p["scan_age"],
                         "—" if p["map_age"] is None else "%.1fs" % p["map_age"],
                         p["busy"])

            # Emit map when dirty AND min interval passed
            if "map" in state:
                # Always emit if we have a map and it's fresh
                # (map_dirty flag means we just got a new one)
                now = time.time()
                if now - last_map_emit > 0.5:
                    socketio.emit("map", state["map"], to='map_view')
                    last_map_emit = now
                    log.debug(
                        "map emitted: %dx%d @ %.2fm",
                        state["map"]["width"],
                        state["map"]["height"],
                        state["map"]["resolution"],
                    )

            # Emit scan when a new /scan frame arrived (seq 由 _scan_cb 递增)。
            # 之前用 hash(len(points)) 判变 —— 点数不变时整个激光叠加层冻结,是 bug。
            if "scan" in state:
                if state["scan"]["seq"] != last_scan_seq:
                    socketio.emit("scan", state["scan"], to='map_view')
                    last_scan_seq = state["scan"]["seq"]

            # 轨迹:state 里只在**真的有新点**时才带这个 key,所以这里是真·增量
            if "trajectory" in state:
                socketio.emit("trajectory", state["trajectory"], to='map_view')

            # Emit pose when changed
            if "pose" in state:
                p = state["pose"]
                key = (p["x"], p["y"], p["yaw"])
                if key != last_pose_key:
                    socketio.emit("pose", p, to='map_view')
                    last_pose_key = key
        except Exception:
            log.exception("broadcast error")

        socketio.sleep(0.05)  # 50ms → 20Hz


# ══════════════════════════════════════════════════════════════════════
# Camera Stream (USB 相机 → SocketIO frame)
# ══════════════════════════════════════════════════════════════════════

_cam_enabled = threading.Event()  # set while ≥1 client wants the stream
_cam_lock = threading.Lock()
_cam_clients = set()  # sids that requested the stream
_cam_flow = {}  # sid -> outstanding frame tokens, bounded per connection
_cam_limits = {}  # legacy flow clients:1; dedicated video connection:at most2
_cam_binary = set()  # opted-in bounded previews; legacy pages keep base64 JPEG
_cam_status = {"running": False, "w": 0, "h": 0, "fps": 0.0, "error": None}
CAMERA_ROOM = "camera_preview"


def _cam_publish_state():
    with _cam_lock:
        state = dict(_cam_status)
    socketio.emit("camera_state", state)


def _cam_preview(jpeg):
    # Forward the original JPEG: preserve its resolution and image quality.
    with Image.open(io.BytesIO(jpeg)) as picture:
        return jpeg, picture.width, picture.height


def _cam_publish_frame(image, width, height, sequence=None, captured_at=None, jpeg=None):
    payload = {"img": image, "w": width, "h": height,
               "sequence": sequence, "captured_at": captured_at}
    # Old clients retain the room contract. Updated pages allow at most two
    # in-flight frame each; slow decoding/network skips frames rather than queues.
    socketio.emit("frame", payload, to=CAMERA_ROOM)
    ready = []
    with _cam_lock:
        for sid, pending in _cam_flow.items():
            if len(pending) < _cam_limits.get(sid, 1):
                token = object()
                pending[token] = True
                ready.append((sid, token, pending, sid in _cam_binary))
    binary_payload = None
    if jpeg is not None and any(item[3] for item in ready):
        try:
            preview, preview_w, preview_h = _cam_preview(jpeg)
        except Exception:
            # Preserve delivery/ACKs if preview conversion fails. Otherwise the
            # reserved tokens fill up and this subscriber cannot recover.
            log.exception('Preview conversion failed; forwarding original JPEG')
            preview, preview_w, preview_h = jpeg, width, height
        binary_payload = {**payload, 'jpeg': preview, 'w': preview_w, 'h': preview_h}
        binary_payload.pop('img')
    for sid, token, pending, binary in ready:
        def decoded(*args, sid=sid, token=token, pending=pending):
            with _cam_lock:
                if _cam_flow.get(sid) is pending:
                    pending.pop(token, None)
        socketio.emit("frame", binary_payload if binary and binary_payload else payload,
                      to=sid, callback=decoded)


def camera_loop():
    """Pull existing fire_detect JPEGs; never opens the camera device itself."""
    log.info("Camera stream thread started (url=%s)", CAMERA_STREAM_URL)
    frame_budget = 1.0 / CAMERA_FPS
    fps_n, fps_t0 = 0, time.time()
    last_sequence = None

    while True:
        try:
            if not _cam_enabled.is_set():
                # Idle: fire_detect remains the sole camera-device owner.
                if _cam_status.get("running"):
                    _cam_status.update(running=False)
                    _cam_publish_state()
                time.sleep(0.2)
                continue

            started = time.monotonic()
            # Pull the latest frame from the standalone camera process
            with urllib.request.urlopen(CAMERA_STREAM_URL, timeout=2.0) as resp:
                jpg = resp.read()
                w = int(resp.headers.get("X-Frame-W", 0))
                h = int(resp.headers.get("X-Frame-H", 0))
                sequence = resp.headers.get("X-Frame-Seq")
                captured_at = float(resp.headers.get("X-Frame-Captured-At", 0)) or None

            if not jpg:
                time.sleep(frame_budget)
                continue
            if sequence is not None and sequence == last_sequence:
                time.sleep(max(0, frame_budget-(time.monotonic()-started)))
                continue
            last_sequence = sequence

            b64 = base64.b64encode(jpg).decode("ascii")
            _cam_publish_frame(b64, w, h, sequence, captured_at, jpg)

            if not _cam_status.get("running"):
                _cam_status.update(running=True, error=None)
                _cam_publish_state()

            fps_n += 1
            now = time.time()
            if now - fps_t0 >= 1.0:
                _cam_status.update(fps=round(fps_n / (now - fps_t0), 1))
                fps_n, fps_t0 = 0, now
                _cam_publish_state()

            # Throttle toward target fps
            time.sleep(max(0.0, frame_budget - (time.monotonic() - started)))

        except Exception as e:
            log.warning("camera error: %s — retrying in 1s", e)
            _cam_status.update(running=False, error=str(e))
            _cam_publish_state()
            time.sleep(1.0)


# ══════════════════════════════════════════════════════════════════════
# Fire Stream (火情标注帧 + 状态 → SocketIO)
# ══════════════════════════════════════════════════════════════════════

_fire_enabled = threading.Event()  # set while ≥1 client wants the stream
_fire_lock = threading.Lock()
_fire_clients = set()  # sids that requested the stream
_fire_status = {
    "running": False,
    "fire": False,
    "confidence": 0.0,
    "reason": "等待首次判定",
    "latency_s": 0.0,
    "samples": 0,
    "error": None,
}


def _fire_publish_state():
    with _fire_lock:
        state = dict(_fire_status)
    socketio.emit("fire_status", state)


def fire_loop():
    """Daemon thread: pull annotated frame + status from fire_detect.py."""
    log.info("Fire stream thread started (url=%s)", FIRE_STREAM_URL)
    last_status = 0.0
    while True:
        try:
            if not _fire_enabled.is_set():
                # Idle: just poll the event (fire_detect handles the device)
                if _fire_status.get("running"):
                    _fire_status.update(running=False)
                    _fire_publish_state()
                time.sleep(0.2)
                continue

            # Annotated frame from fire_detect.py
            with urllib.request.urlopen(FIRE_STREAM_URL, timeout=2.0) as resp:
                jpg = resp.read()
            if jpg:
                b64 = base64.b64encode(jpg).decode("ascii")
                socketio.emit("fire_frame", {"img": b64})
                if not _fire_status.get("running"):
                    _fire_status.update(running=True, error=None)
                    _fire_publish_state()

            # Status JSON (throttled)
            now = time.time()
            if now - last_status >= FIRE_STATUS_INTERVAL:
                last_status = now
                with urllib.request.urlopen(FIRE_STATUS_URL, timeout=2.0) as resp:
                    st = json.loads(resp.read().decode())
                _fire_status.update(
                    fire=bool(st.get("fire")),
                    confidence=round(float(st.get("confidence", 0.0)), 3),
                    reason=str(st.get("reason", "")),
                    latency_s=round(float(st.get("latency_s", 0.0)), 2),
                    samples=int(st.get("samples", 0)),
                    error=None,
                )
                _fire_publish_state()

            time.sleep(0.15)
        except Exception as e:
            log.warning("fire stream error: %s — retrying in 1s", e)
            _fire_status.update(running=False, error=str(e))
            _fire_publish_state()
            time.sleep(1.0)


# ── Motor control state (shared with frontend) ────────────────────
_motor_armed = False

# 当前「驾驶者」的 SocketIO sid。同一份页面经常被两台设备同时打开(PC + 手机 /
# 两个标签页),两边的定时器都会持续发速度 —— 没握摇杆的那台发的是 0(那是死人
# 手柄的正常输出,不是故障),但它是**另一个客户端**的零,会把真值一次次打断,
# 目标速度在 0 和真值之间跳,电机就来回抖。
# 2026-09-15 实测:两台设备合计约 39.5Hz,真值中间被硬插零 —— 当时前端定时器还是
# 20Hz,现在已提到 60Hz(见文件末尾的 setInterval),机制不变、只是更密。
# 规则:只有按下「启用电机」的那个客户端能发速度指令,其余一律丢弃。
_drive_owner = None
_drive_owner_client = None
_drive_owner_note_ts = 0.0
DISARM_GRACE_S = 6.0

# 云台「操作者」的 sid。**与 _drive_owner 完全独立** —— 不复用它的理由:
# _drive_owner 只在按「启用电机」时抢占,复用就会把云台重新绑回 armed 门上,
# 而「云台不受 armed 管」是这次的设计决定(转相机 ≠ 驱动轮子)。
#
# 抢占规则(**与 _drive_owner 不同,因为云台没有「启用」这个动作**):
#   * 非操作者发**零**(dx=dy=0) → **丢弃**。这就是要防的那件事:另一台设备
#     开着页面、死人摇杆持续发零,会把正在操作的指令一次次切断 ⇒ 云台抖。
#   * 非操作者发**非零** → **抢占**。一个人正在推杆是明确的意图表达,而云台上
#     没有别的「接管」入口;若改成先到先得,一台开着的闲页面就能把云台**永久**
#     锁死,而 UI 上没有任何办法解开。
#   * ⚠️ 残留歧义:两个人**同时**推杆会互相抢占(交替)。这是「无接管按钮」这个
#     前提的必然代价,写在这里备查 —— 不假装它不存在。
_gimbal_owner = None
_gimbal_note_ts = 0.0

# ── 已删除的临时诊断:一段判据是错的包率探针(2026-09-15 删) ──────────
# 这里原本有个 _vel_stat():每 2 秒统计包率 / 非零计数 / SocketIO 传输方式,并据此
# 推断「瓶颈在浏览器 → 服务端这一段,之前修 PWM/GIL 修错了方向」。**结论是反的**,
# 所以整块删掉。把它的错法记在这里,免得下次再照着它查一遍:
#
#   * 「l/a 全是 0」被判成故障 —— 实为**死人手柄的设计行为**:松杆/松键时前端持续
#     发零(见文件末尾 setInterval 处),零就是「当前没有输入」的正确表达。
#   * 「传输 = polling,所以传输层慢」—— werkzeug 开发服务器**只能**提供长轮询,
#     websocket 根本协商不出来;而长轮询实测端到端满速(diag_client.py 灌 60Hz,
#     pwm_proc 侧收到 59.9~60.2 包/s)。**不要改成 transports:['websocket']**,
#     那会把这条本来通的链路直接弄断。
#   * 当时读到的 18.4/s 是**瞬态**,不是传输层上限:werkzeug 访问日志的自持循环
#     (见本文件开头 69-77 行,已修)把客户端定时器饿到了十几 Hz。
#   * 真凶在更下游,而且在同一进程内:软 PWM 线程对整个 PWM 周期纯自旋、死占 GIL,
#     把 pwm_proc 自己的 200Hz 主循环饿到 1~2Hz,同时撑爆 UDP 接收缓冲(发 60 收 24)。
#     完整记录见 motor_hw.py 的 DualSoftPwm._busy_until。


def _emit_motor_state():
    socketio.emit("motor_state", {"armed": _motor_armed, "driver": _drive_owner})


_drive_io_lock = threading.RLock()

# 2026-10-24: 手动驾驶延迟统计（每 5 秒一行，p50/p95/max）
_lat_samples = []
_lat_last_log = 0.0


def _send_drive(l, a, armed, rot=0.0):
    """把归一化摇杆值直发 pwm_proc(绕开 ROS)。

    尽力而为:没起 pwm_proc 时只是丢包,绝不能让这条路把网页的按键回调搞崩,
    所以这里吞掉 OSError 只记 debug。

    armed 一并带上,是因为 pwm_proc 不跑 ROS、拿不到 /motor_arm —— 它靠这个
    字段决定动不动电机,上电默认 false(必须网页点「启用电机」)。
    """
    try:
        with _drive_io_lock:
            _pwm_sock.sendto(
                json.dumps({"l": float(l), "a": float(a), "rot": float(rot or 0.0),
                            "armed": bool(armed and _motor_armed)}).encode(),
                (PWM_UDP_HOST, PWM_UDP_PORT))
    except OSError as e:
        log.debug("pwm udp send failed: %s", e)


def _stage1_drive_packet(data):
    """Loopback latest-value input; never grants ownership or enables motors."""
    with _drive_io_lock:
        if not isinstance(data, dict) or not _motor_armed or data.get('sid') != _drive_owner:
            return
        try:
            l,a=float(data['l']),float(data['a'])
            valid=all(math.isfinite(x) and abs(x)<=1 for x in (l,a))
        except (KeyError,ValueError,TypeError):
            valid=False
        if not valid or not drive_packet_fresh(data,time.monotonic()):
            _navigation_disarm()
            return
        _send_drive(l,a,True)


def stage1_drive_loop():
    sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sock.bind(('127.0.0.1',5818))
    sock.settimeout(.1)
    while True:
        try:
            packet,_=sock.recvfrom(1024)
            sock.setblocking(False)
            try:
                while True:packet,_=sock.recvfrom(1024)
            except BlockingIOError:pass
            finally:sock.settimeout(.1)
            _stage1_drive_packet(json.loads(packet))
        except (socket.timeout,ValueError,UnicodeError):pass
        except Exception:log.exception('stage1 UDP input error')


def _emit_gimbal_state():
    """只在**操作者变化时**推 —— 绝不每包推。

    报文里带个字段其实更省事,但那是给 socketio.emit 消防水管 —— 本文件开头
    记着自持 emit 循环把客户端定时器饿到十几 Hz 的事故,别再开一口。
    """
    socketio.emit("gimbal_state", {"owner": _gimbal_owner})


def _send_gimbal(dx, dy):
    """把**协议单位**(±100)的 dx/dy 直发 gimbal_proc。

    ⚠️ `dx/dy` 是**速度**,不是角度,也不是位置。`(0,0)` = **速度 0 = 停**,
       **不是**「回中」—— 收端不会去找中位。见 `gimbal_proto.py` 模块 docstring。

    ⚠️ 单位就是**线上单位**:前端、这里、UDP、串口帧全程都是 ±100,中间不做
       -1..1 的归一化换算。这样界面读数可以**原样**显示真正会上线的那个数 ——
       少一次换算就少一处「界面说 A、线上是 B」的可能。
       (±100 是**本实现选定的**满量程,不是协议上限:协议是 int16 全域,
        |value| ≥ 250 才饱和。)
    ⚠️ **别把这里当成「线上最终值」**:`gimbal_proc` 还会对它做**斜坡限幅**
       (`gimbal_slew_per_s`)。所以界面读数 = **目标**,线上是**斜坡之后的已发值**;
       两者会短暂不同 —— 进程的状态行两个都报,别只信界面。

    ⚠️ 这里"尽力而为、吞掉 OSError"是**对的**,理由和 `_send_drive` 一样:
       UDP 是**最新值语义**,丢一个立刻被下一个覆盖。**别把这条结论用到串口上**
       —— gimbal_proc 里那条字节流相反,短写必须补齐(见那边 docstring【2】)。

    ⚠️ 钳位在这里**就地写**,刻意**不** `from gimbal_proto import clamp_cmd100`:
       那会让「gimbal_proto.py 缺失/语法错」变成「整个网页起不来」——
       地图、相机、驾驶全跟着死,只因为云台。这与本项目「爆炸半径要恰好是
       自己」的纪律相反。**权威实现仍在 gimbal_proto.clamp_cmd100**(有单测),
       这里只是入口处的第二道,防的是前端塞进来的野值。
    """
    try:
        dxi = max(-100, min(100, int(float(dx))))
    except (TypeError, ValueError):
        dxi = 0
    try:
        dyi = max(-100, min(100, int(float(dy))))
    except (TypeError, ValueError):
        dyi = 0
    try:
        _gimbal_sock.sendto(json.dumps({"dx": dxi, "dy": dyi}).encode(),
                            (GIMBAL_UDP_HOST, GIMBAL_UDP_PORT))
    except OSError as e:
        log.debug("gimbal udp send failed: %s", e)


# ── SocketIO camera events ────────────────────────────────────────

@socketio.on("disconnect")
def _on_disconnect():
    global _motor_armed, _drive_owner, _gimbal_owner
    patrol = getattr(_navigation, 'patrol_runner', None)
    navigation_owned = _navigation is not None and (request.sid == _navigation.nav_owner
        or (patrol is not None and patrol.active and request.sid == patrol.sid))
    # Disconnect must stop hardware before acquiring navigation locks/RPCs.
    # An observer disconnect must leave the actual driver's task untouched.
    if _motor_armed and request.sid == _drive_owner:
        # 2026-10-27: 不再立即 disarm —— 页面 socket 会周期性重连（polling），
        # 每次旧 sid 断开都会把电机收掉。给 6 秒宽限：同一浏览器重连会自动
        # 把驾驶权还给新 sid；真走了（关页面/断网）才 disarm。
        lost_sid = request.sid
        def _late_disarm():
            global _motor_armed, _drive_owner, _drive_owner_client
            if _motor_armed and _drive_owner == lost_sid:
                _motor_armed = False
                _drive_owner = None
                _drive_owner_client = None
                _send_drive(0.0, 0.0, False)
                _ros_node.publish_cmd_vel(0.0, 0.0)
                _ros_node.set_motor_arm(False)
                _emit_motor_state()
                log.warning('Driver disconnected — motors disarmed (%.0fs grace)', DISARM_GRACE_S)
        threading.Timer(DISARM_GRACE_S, _late_disarm).start()
        log.warning('Driver connection lost — %.0fs grace before disarm', DISARM_GRACE_S)
    if navigation_owned:
        try:
            _navigation.cancel('操作者断线，导航已取消', disarm=True)
        except Exception:
            log.exception('Navigation cancellation failed after disconnect output was disarmed')
    with _cam_lock:
        _cam_clients.discard(request.sid)
        _cam_flow.pop(request.sid, None)
        _cam_limits.pop(request.sid, None)
        _cam_binary.discard(request.sid)
        if not _cam_clients:
            _cam_enabled.clear()
            log.info("All clients left — camera stream OFF")
    _fire_clients.discard(request.sid)
    if not _fire_clients:
        _fire_enabled.clear()
        log.info("All clients left — fire stream OFF")
    # 云台操作者断开 → 释放操作权,**并把云台停住(发零速)**。
    # ⚠️ 措辞是「停」不是「回中」:`0xAA` 是**速度帧**,`(0,0)` = 速度 0 = **停**。
    #    收端不会去找中位 —— 回中只有 `0xBB cmd=1/2` 能做,我们没实现。
    #    写成「回中」会让人在排查时往完全错的方向查。
    # ⚠️ 这里和上面**刻意不对称**:电机断开要「急停」,云台断开只是「停住」。
    #    理由:发 (0,0) 是一次**明确的、由操作者离开触发的**动作,而「链路丢失」
    #    走的是 gimbal_proc 的 deadman(那边**也**是发零速,不是停发 —— 速度锁存,
    #    停发 = 云台一直转)。防的是「关掉标签页,云台一直按最后的速度转下去」。
    # ⚠️ 只有**操作者**断开才动;另一台设备刷新页面不该把云台拽停。
    if request.sid == _gimbal_owner:
        _gimbal_owner = None
        _send_gimbal(0, 0)
        _emit_gimbal_state()
        log.info("云台操作者断开 — 已发零速并释放操作权")


@socketio.on("camera_control")
def _on_camera_control(data):
    if not isinstance(data, dict):
        return
    action = (data or {}).get("action")
    if action == "start":
        flow = data.get('flow_control') is True
        if flow:
            leave_room(CAMERA_ROOM)
        else:
            join_room(CAMERA_ROOM)
        with _cam_lock:
            _cam_clients.add(request.sid)
            if flow:
                _cam_flow.setdefault(request.sid, {})
                _cam_limits[request.sid] = 2 if data.get('flow_window') == 2 else 1
                if data.get('binary') is True:
                    _cam_binary.add(request.sid)
                else:
                    _cam_binary.discard(request.sid)
            else:
                _cam_flow.pop(request.sid, None)
                _cam_limits.pop(request.sid, None)
                _cam_binary.discard(request.sid)
            _cam_enabled.set()
            _cam_status.update(error=None)
        log.info("Camera stream ON (sid=%s)", request.sid)
    elif action == "stop":
        leave_room(CAMERA_ROOM)
        with _cam_lock:
            _cam_clients.discard(request.sid)
            _cam_flow.pop(request.sid, None)
            _cam_limits.pop(request.sid, None)
            _cam_binary.discard(request.sid)
            if not _cam_clients:
                _cam_enabled.clear()
                log.info("Camera stream OFF (sid=%s)", request.sid)
    _cam_publish_state()


@socketio.on("camera_sync")
def _on_camera_sync():
    # Clock alignment for estimated readout-to-browser age; never actuates hardware.
    return time.time()


@socketio.on("drive_sync")
def _on_drive_sync():
    # Driver timestamps share this process's monotonic clock, never wall time.
    return time.monotonic()


@socketio.on("fire_control")
def _on_fire_control(data):
    action = (data or {}).get("action")
    if action == "start":
        _fire_clients.add(request.sid)
        _fire_enabled.set()
        _fire_status.update(error=None)
        log.info("Fire stream ON (sid=%s)", request.sid)
    elif action == "stop":
        _fire_clients.discard(request.sid)
        if not _fire_clients:
            _fire_enabled.clear()
            log.info("Fire stream OFF (sid=%s)", request.sid)
    _fire_publish_state()


def drive_packet_fresh(data, now):
    """Reject queued browser or local gamepad commands after a transport stall."""
    if 'sent_at' in data:
        try:
            age = time.time()-float(data['sent_at'])
            return math.isfinite(age) and -0.05 <= age <= 0.20
        except (TypeError, ValueError):
            return False
    if 'sent_monotonic' not in data:
        return True
    try:
        age = now - float(data['sent_monotonic'])
        return math.isfinite(age) and -0.05 <= age <= 0.20
    except (TypeError, ValueError):
        return False


@socketio.on("motor_control")
def _on_motor_control(data):
    global _motor_armed, _drive_owner, _drive_owner_client, _drive_owner_note_ts, _gimbal_owner, _lat_last_log
    action = (data or {}).get("action")
    if action == "vel":
        _t0 = time.monotonic()
        # 只认「已经启用电机」的那个驾驶者的指令:另一台设备发来的(通常是持续 0)
        # 直接丢弃,否则会把真值切断成锯齿,电机来回抖。见 _drive_owner 的注释。
        #
        # ⚠️ 2026-09-15 修掉一个洞:原判据是
        #         `_drive_owner is not None and request.sid != _drive_owner`
        #    —— 当 _drive_owner 为 None 时左半边为假,整个条件**短路成 False**,
        #    于是任何人都能走到下面、并带着 armed=true 转发给 pwm_proc:
        #    **不用点「启用电机」就能驱动电机**。而 _drive_owner 的初值正是 None,
        #    estop 和驾驶者断开又都把它清回 None —— 也就是说「未使能」恰好就是
        #    让这道检查失效的那个状态。
        #    正确写法是要求 sid 就是 _drive_owner:`None != sid` 恒真,未使能时
        #    自然丢弃。再显式要求服务端确实处在 armed,不再靠调用点多传一个 True。
        if request.sid != _drive_owner or not _motor_armed:
            now = time.monotonic()
            if now - _drive_owner_note_ts > 5.0:      # 限频,别刷屏
                _drive_owner_note_ts = now
                log.info("Ignoring vel from non-driver/disarmed client %s",
                         request.sid)
            return
        if not drive_packet_fresh(data, time.monotonic()):
            _motor_armed = False
            _drive_owner = None
            _send_drive(0.0, 0.0, False)
            _ros_node.publish_cmd_vel(0.0, 0.0)
            _ros_node.set_motor_arm(False)
            _emit_motor_state()
            log.warning('Expired drive command: motors disarmed')
            return
        # 摇杆/键盘发来已归一化 -1..1 的速度
        lin = float(data.get("linear", 0.0))
        ang = float(data.get("angular", 0.0))
        if _navigation is not None and (_navigation.navigating or _automatic_active()):
            if abs(lin) < 0.001 and abs(ang) < 0.001 and not data.get('takeover'):
                return  # Idle manual heartbeats must not overwrite autonomous commands.
            log.warning('Manual takeover: sid=%s owner=%s lin=%.3f ang=%.3f takeover=%s',
                        request.sid, _drive_owner, lin, ang, bool(data.get('takeover')))
            _navigation.cancel('手动接管，导航已取消')
        # A loopback ROS bridge has already consumed /cmd_vel. Mirroring its
        # normalized output back to that topic would create a feedback loop.
        ros_input = data.get('source') == 'ros_stage1' and request.remote_addr in ('127.0.0.1', '::1')
        if not ros_input:
            _ros_node.publish_cmd_vel(lin, ang)
        _send_drive(lin, ang, _motor_armed)   # 别硬编码 True;见上
        try:
            _age = (time.monotonic()-float(data['sent_monotonic']) if 'sent_monotonic' in data
                    else time.time()-float(data['sent_at']))
        except (KeyError, TypeError, ValueError):
            _age = None
        if _age is not None:
            _lat_samples.append((_age*1000.0, (time.monotonic()-_t0)*1000.0))
            if len(_lat_samples) >= 10 and time.monotonic()-_lat_last_log > 5.0:
                _lat_last_log = time.monotonic()
                _a = sorted(x[0] for x in _lat_samples)
                _c = sorted(x[1] for x in _lat_samples)
                log.info('drive latency n=%d age p50=%.0f p95=%.0f max=%.0f ms | handler p95=%.1f ms',
                         len(_a), _a[len(_a)//2], _a[int(len(_a)*0.95)-1], _a[-1],
                         _c[int(len(_c)*0.95)-1])
                _lat_samples.clear()
    elif action == "arm":
        if data.get('claim_if_disarmed') and _motor_armed and _drive_owner != request.sid:
            _emit_motor_state()
            return
        if _navigation is not None and (_navigation.navigating or _navigation.pending or _automatic_active()):
            _navigation.cancel('驾驶使能改变，导航已取消')
        # 谁按的「启用电机」谁就是驾驶者;解除时让出控制权
        _motor_armed = bool(data.get("armed", False))
        _drive_owner = request.sid if _motor_armed else None
        _drive_owner_client = (request.args.get('client') or None) if _motor_armed else None
        _ros_node.set_motor_arm(_motor_armed)
        # 立刻同步给直连通道:解除时这一包就是急停,pwm_proc 不必等 deadman
        _send_drive(0.0, 0.0, _motor_armed)
        _emit_motor_state()
        log.info("Motor %s (driver=%s)",
                 "ARMED" if _motor_armed else "disarmed", _drive_owner or "-")
    elif action == "estop":
        _motor_armed = False
        _drive_owner = None
        _drive_owner_client = None
        # Hardware stop must precede any Nav2 lock/service cancellation.
        _send_drive(0.0, 0.0, False)
        _ros_node.publish_cmd_vel(0.0, 0.0)
        _ros_node.set_motor_arm(False)
        # ⚠️ 顺手把云台也**停住**(发零速)。**不改这个的话**:一个叫「急停」的按钮
        #    按下去,相机云台还在动 —— 那是**模糊的**,而模糊的急停比没有急停更坏。
        #    这一行让按钮的标签变诚实。
        # ⚠️ 是「停」不是「回中」:`(0,0)` 是速度 0,云台不会去找中位。
        # ⚠️ 代价(接受):急停会顺带松开云台操作权。对一个**停止一切**的按钮,
        #    这是对的方向,不是副作用。
        _gimbal_owner = None
        _send_gimbal(0, 0)
        _emit_gimbal_state()
        _emit_motor_state()
        if _navigation is not None:
            try:
                _navigation.cancel('急停，导航已取消')
            except Exception:
                log.exception('Navigation cancellation failed after ESTOP output was disarmed')
        log.warning("ESTOP pressed — motors disarmed, gimbal stopped (zero velocity)")


@socketio.on("gimbal_control")
def _on_gimbal_control(data):
    """网页云台摇杆 → gimbal_proc(串口)。

    ⚠️⚠️ **这里没有 armed 检查,这是刻意的。** 云台转的是相机,不是轮子。
       别照着 `_on_motor_control` 的样子补一个 —— 也别把它和「不点启用就能
       驱动**电机**」那个已修的洞混为一谈:那是电机门控的漏洞,这是云台**本来
       就没有门**。判据的差别在于动作是什么,不在于代码长得像不像。

    ⚠️ 抢占判据必须写成 `request.sid != _gimbal_owner`(**不是**
       `_gimbal_owner is not None and ...`)。后者在 owner 为 None 时左半为假、
       整个条件短路成 False —— 那正是 2026-09-15 在 `_on_motor_control` 里修掉的
       那个洞的形状。**同一个坑别踩第二次**,这里是刻意用同样的写法。
    """
    global _gimbal_owner, _gimbal_note_ts
    action = (data or {}).get("action")
    if action != "vel":
        log.warning("未知 gimbal_control action=%r (sid=%s)", action, request.sid)
        return
    # ⚠️ 「停」走的就是 vel(0,0),**没有单独的 "stop" action**。理由:非操作者的
    #    (0,0) 本来就被下面那条「非操作者发零 ⇒ 丢弃」挡住,而操作者的 (0,0)
    #    走正常路径 — 两条合起来已经覆盖了「松手/失焦/页面隐藏要停住」的全部情形。
    #    再开一个 stop 分支就是一条**从任何地方都到不了**的路(死代码)。
    # ⚠️ `(0,0)` 是**零速**,不是「回中」—— 协议里 dx/dy 是速度(锁存),收端不会
    #    去找中位。见 gimbal_proto.py 的模块 docstring。

    dx = data.get("dx", 0)
    dy = data.get("dy", 0)
    # 先判「是不是零」再判归属:非操作者的零要**丢弃**(见 _gimbal_owner 注释)。
    try:
        nonzero = bool(int(float(dx))) or bool(int(float(dy)))
    except (TypeError, ValueError):
        nonzero = False

    if request.sid != _gimbal_owner:
        if not nonzero:
            now = time.monotonic()
            if now - _gimbal_note_ts > 5.0:          # 限频,否则 62.5 行/秒
                _gimbal_note_ts = now
                log.info("Ignoring gimbal zero from non-operator %s", request.sid)
            return
        _gimbal_owner = request.sid
        _emit_gimbal_state()
        log.info("Gimbal operator → %s (抢占了上一个操作者)", request.sid)

    _send_gimbal(dx, dy)


@socketio.on("mapping_control")
def _on_mapping_control(data):
    """启动/停止建图管线。形状照 motor_control:`(data or {}).get("action")` + if/elif。

    ⚠️ 这是这个 Web UI 里**第一个进程控制口子**,边界要明确:
       - 只经 `mapping_ctl.sh` 碰那 4 个 SLAM 进程(雷达/激光里程计/静态TF/slam_toolbox)。
       - **不碰电机**:start 不会 arm,stop 不是 estop。`pwm_proc` 的 armed 状态和
         deadman 完全不受影响,所以按下这个按钮车不会动。
    """
    action = (data or {}).get("action")
    if action not in ("start", "stop"):
        log.warning("未知 mapping_control action=%r (sid=%s)", action, request.sid)
        return
    if _navigation is not None and (_navigation.running() or _navigation.busy):
        _push_log('slam', '[WARNING] 请先停止定位，再操作建图')
        return
    if _mapping_request(action):
        log.info("建图控制: %s (sid=%s)", action, request.sid)
    else:
        log.info("建图控制: 忽略重复的 %s (sid=%s)", action, request.sid)
    _emit_mapping_state()   # 让所有客户端立刻看到 busy,按钮进入「启动中…」


# ══════════════════════════════════════════════════════════════════════
# Log tail (管线进程日志 → SocketIO log)
# ══════════════════════════════════════════════════════════════════════

_log_buffer = deque(maxlen=LOG_BUFFER_SIZE)
_log_lock = threading.Lock()


# HTTP 访问日志的指纹:... "GET /socket.io/?... HTTP/1.1" 200 -
# 见到就直接丢掉 —— 这类行的产生量正比于 HTTP 请求量,而投递一条事件又需要
# 一次 HTTP 请求,放行任何一条都会把 _push_log 变成自持循环(原因见文件头部
# werkzeug 那段)。werkzeug 已经被压到 ERROR,这里是第二道闸,防止日志配置
# 以后被人改回 INFO 时这个坑重新长出来。
_ACCESS_LOG_RE = re.compile(r'"\s*[A-Z]{3,7}\s+\S+\s+HTTP/[\d.]+"')
_ACCESS_LOG_WARNED = False


def _push_log(tag: str, line: str):
    global _ACCESS_LOG_WARNED
    if _ACCESS_LOG_RE.search(line):
        # 只提醒一次:这条警告本身也是条日志,限频才是有限的一跳。
        if not _ACCESS_LOG_WARNED:
            _ACCESS_LOG_WARNED = True
            log.warning("日志里出现 HTTP 访问记录 —— werkzeug 的日志级别又被放开了?"
                        "(放行它会形成自持循环,详见文件头部说明)")
        return
    line = re.sub(r"\x1b\[[0-9;]*m", "", line)  # strip ANSI color codes
    level = "INFO"
    m = re.search(r"\[(DEBUG|INFO|WARNING|WARN|ERROR|CRITICAL)\]", line)
    if m:
        level = m.group(1)
    entry = {
        "t": time.strftime("%H:%M:%S"),
        "tag": tag,
        "level": level,
        "msg": line,
    }
    with _log_lock:
        _log_buffer.append(entry)
    socketio.emit("log", entry, to='log_view')


def log_tail_loop():
    """Tail pipeline log files → socketio 'log' events."""
    log.info("Log tail thread started (%d files)", len(LOG_FILES))
    handles = {}  # path -> (file handle, tag, 上次看到的字节数)
    while True:
        for path, tag in LOG_FILES:
            try:
                entry = handles.get(path)
                if entry is None:
                    if not os.path.exists(path):
                        continue
                    fh = open(path, "r", errors="replace")
                    # 首次见到:只读之后新增的行,别把历史全灌给客户端
                    size = os.fstat(fh.fileno()).st_size
                    fh.seek(0, os.SEEK_END)
                    handles[path] = (fh, tag, size)
                    continue
                fh, _, size = entry
                cur = os.fstat(fh.fileno()).st_size
                # ★ 文件被**原地截断**了(新一轮建图用 `>` 重定向,见 mapping_ctl.sh):
                #   句柄的偏移已经越过 EOF,不重开就会一直读到空;更坏的是等新内容
                #   长过旧长度之后,它会从**半行中间**开始读,前面那段日志静默丢失
                #   —— 而「启动失败的原因」恰好就在那一段里。
                if cur < size:
                    fh.close()
                    fh = open(path, "r", errors="replace")
                    cur = 0
                elif cur == size:
                    continue
                for line in fh:
                    line = line.rstrip("\n")
                    if line.strip():
                        _push_log(tag, line)
                handles[path] = (fh, tag, os.fstat(fh.fileno()).st_size)
            except Exception:
                handles.pop(path, None)  # reopen next round
        time.sleep(0.5)


@socketio.on("connect")
def _on_connect(auth=None):
    global _drive_owner
    # 之前这里没有任何日志,连上来用的是 websocket 还是 long-polling 完全看不见
    # —— 而这两者的延迟差一个数量级(polling 每个包都要一次 HTTP 往返)。
    _up = str(request.environ.get("HTTP_UPGRADE", "")).lower()
    log.info("Client %s connected (transport=%s)",
             request.sid, "websocket" if _up == "websocket" else "polling")
    view = request.args.get('view', 'console')
    map_view = view not in ('control', 'camera')
    if map_view:
        join_room('map_view')
        join_room('log_view')
    # Send a snapshot of recent logs to the newly connected client
    with _log_lock:
        snapshot = list(_log_buffer)
    if snapshot and map_view:
        socketio.emit("log_snapshot", snapshot, to=request.sid)
    # Push current motor armed state (default disarmed)
    socketio.emit("motor_state",
                  {"armed": _motor_armed, "driver": _drive_owner}, to=request.sid)
    # 2026-10-27: 同一个浏览器重连 => 把驾驶权还给新 sid，不必重新点「启用电机」
    _client_id = request.args.get('client')
    if _motor_armed and _drive_owner_client and _client_id == _drive_owner_client:
        _drive_owner = request.sid
        socketio.emit('motor_state', {'armed': _motor_armed, 'driver': _drive_owner}, to=request.sid)
        log.info('Driver re-attached after reconnect (sid=%s)', request.sid)
    # 建图状态也一样要补发:边缘触发的广播循环只会在**变化时**发,不补这一发,
    # 刷新页面后按钮会一直停在 HTML 里的初始文案,直到下次状态变化才跟上。
    socketio.emit("mapping_state", _map_state_payload(), to=request.sid)
    # 让客户端知道自己的 sid,才能判断「我是不是驾驶者」
    socketio.emit("you", {"sid": request.sid}, to=request.sid)
    if _navigation is not None:
        socketio.emit('navigation_state', _navigation.payload(), to=request.sid)
    elif _navigation_error:
        socketio.emit('navigation_error', {'message': _navigation_error}, to=request.sid)

    # 轨迹走的是**增量**,新客户端手里没有历史 —— 不补这一发,它就只会从
    # 「连上的那一刻」开始画,之前走过的路是空的(刷新页面、开第二个标签页
    # 尤其明显)。只发给这个 sid,不影响已经在线的客户端。
    if _ros_node is not None and map_view:
        # Map servers publish once; a new browser must receive the cached
        # static map even if no future /map callback marks it dirty again.
        with _ros_node._lock:
            map_msg = _ros_node._latest_map_msg
        if map_msg is not None:
            map_data = occgrid_to_png_base64(map_msg)
            if map_data:
                socketio.emit('map', map_data, to=request.sid)
        traj = _ros_node.full_trajectory()
        if traj:
            socketio.emit("trajectory", traj, to=request.sid)


@socketio.on("log_clear")
def _on_log_clear():
    with _log_lock:
        _log_buffer.clear()


# ══════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════

def _shutdown(signum, frame):
    """SIGTERM/SIGINT → 必须真的退出。

    2026-10-18 实测事故：旧实现里 _navigation.stop() / rclpy.shutdown() 任何一步
    抛异常（典型："rcl_shutdown already called" / "Context must be initialized
    before it can be shutdown"），后面的 sys.exit(0) 就永远执行不到 ——
    进程变成僵尸：Flask 继续服务 5800，ROS 上下文已死，网页显示的是假状态；
    更糟的是 SIGTERM 从此杀不掉它，systemd 重起的实例全卡在
    "Address already in use"，服务 CrashLoop 而僵尸继续对外发旧数据。
    ⇒ 三步各自 try/except，最后 os._exit(0) 让内核立刻收回进程和端口。
    """
    log.info("Received signal %s, shutting down...", signum)
    try:
        if _navigation is not None:
            _navigation.stop()
    except Exception:
        log.exception("navigation.stop() failed during shutdown")
    try:
        rclpy.shutdown()
    except Exception:
        log.exception("rclpy.shutdown() failed during shutdown")
    log.info("shutdown complete — exiting")
    os._exit(0)


def main():
    global _ros_node, _navigation, _navigation_error

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Verify ROS2 env
    if "AMENT_PREFIX_PATH" not in os.environ:
        log.warning("ROS2 environment not detected!")
        log.warning("Run: source /opt/ros/humble/setup.bash")
        log.warning("     source ~/lidar_ws/install/setup.bash")

    rclpy.init(args=sys.argv[1:])
    _ros_node = SlamVizNode()
    try:
        sys.path.insert(0, os.path.expanduser('~/robot/navigation'))
        from navigation_runtime import NavigationRuntime
        _navigation = NavigationRuntime(_ros_node, socketio, _send_drive,
            lambda: _motor_armed, lambda: _drive_owner, _navigation_disarm)
    except Exception as e:
        _navigation_error = '导航组件不可用：'+str(e)
        log.exception(_navigation_error)

    # Inspection consumes the existing camera owner's HTTP frames; never opens a camera.
    sys.path.insert(0, os.path.expanduser('~/robot/inspection'))
    from inspection_service import InspectionService, register_inspection
    from pathlib import Path
    def inspection_task(name):
        if _navigation is None:
            raise ValueError('任务模块尚未就绪；可以不绑定路线做定点检测')
        return _navigation.patrols.get(name)
    def inspection_pose():
        if _navigation is not None and _navigation.localized(time.monotonic()):
            return dict(_navigation.last_pose, map=_navigation.active_map, frame='map')
        return None
    inspection = InspectionService(Path.home()/'robot'/'data'/'inspections',
        task_list=lambda: _navigation.patrols.list() if _navigation is not None else [],
        task_get=inspection_task, pose=inspection_pose)
    register_inspection(app, inspection, Path.home()/'robot'/'web')
    if _navigation is not None:
        _navigation.patrol_runner.inspections = inspection
    threading.Thread(target=inspection.health_loop, daemon=True).start()
    if _navigation is not None:
        from autonomy_runtime import AutonomyRuntime, register_autonomy
        _navigation.autonomy = AutonomyRuntime(_navigation, inspection)
        register_autonomy(app, _navigation.autonomy)
        _navigation.autonomy.start_threads()

    # Spin rclpy in daemon thread
    threading.Thread(target=stage1_drive_loop, daemon=True).start()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(_ros_node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    log.info("rclpy spinning in daemon thread")

    # Start broadcast as SocketIO background task
    socketio.start_background_task(broadcast_loop)

    # Start camera stream thread (idle until a client requests it)
    threading.Thread(target=camera_loop, daemon=True).start()

    # Start fire stream thread (idle until a client requests it)
    threading.Thread(target=fire_loop, daemon=True).start()

    # Start log tail thread
    threading.Thread(target=log_tail_loop, daemon=True).start()

    log.info("Starting Flask-SocketIO on 0.0.0.0:%d", PORT)
    log.info("Open http://<LAN-IP>:%d in your browser", PORT)

    try:
        socketio.run(app, host="0.0.0.0", port=PORT, request_handler=RealtimeRequestHandler)
    finally:
        log.info("Cleaning up...")
        try:
            if _navigation is not None:
                _navigation.stop()
        except Exception:
            log.exception("navigation.stop() failed during cleanup")
        try:
            _ros_node.destroy_node()
            rclpy.shutdown()
        except Exception:
            log.exception("ROS cleanup failed")


# ══════════════════════════════════════════════════════════════════════
# HTML + JS 前端 (嵌入) — 平铺三窗口: 地图 / 相机 / 日志
# ══════════════════════════════════════════════════════════════════════

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>犀牛一号 · 建图工作台</title>
<script src="/vendor/socket.io.min.js"></script>
<style>
  :root {
    --bg:#1c1c24; --bg-2:#16161d;
    --panel:rgba(255,255,255,0.055); --panel2:rgba(255,255,255,0.09);
    --line:rgba(255,255,255,0.10);
    --fg:#e7e9f0; --dim:#8a90a2; --accent:#7c8cff;
    --accent-soft:rgba(124,140,255,0.16);
    --shadow:0 10px 34px rgba(0,0,0,0.38);
    --radius:12px;
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  html,body { height:100%; }
  body {
    background:
      radial-gradient(1100px 700px at 12% -10%, rgba(124,140,255,0.14), transparent 60%),
      radial-gradient(900px 620px at 105% 110%, rgba(168,120,255,0.12), transparent 60%),
      var(--bg);
    color:var(--fg); overflow:hidden;
    font-family:'Segoe UI',system-ui,-apple-system,'Helvetica Neue',sans-serif;
    overscroll-behavior:none;   /* 手机上抑制下拉刷新抢触摸 */
  }

  /* ── 顶栏 ─────────────────────────────── */
  #status-bar {
    position:fixed; top:0; left:0; right:0; height:34px; z-index:30;
    display:flex; align-items:center; gap:18px; padding:0 14px; font-size:12px;
    background:rgba(28,28,36,0.62); backdrop-filter:blur(12px); -webkit-backdrop-filter:blur(12px);
    border-bottom:1px solid var(--line);
  }
  #status-conn { display:flex; align-items:center; gap:6px; }
  #conn-dot { width:8px; height:8px; border-radius:50%; background:#f45; }
  #conn-dot.live { background:#4ade80; box-shadow:0 0 8px rgba(74,222,128,.6); animation:pulse 2s infinite; }
  .stat-label { color:var(--dim); margin-right:4px; }
  .spacer { flex:1; }
  #s-clock { color:var(--dim); font-variant-numeric:tabular-nums; }

  /* ── 工作区平铺网格 ───────────────────── */
  #workspace {
    position:fixed; top:34px; left:0; right:0; bottom:0;
    display:grid; gap:8px; padding:8px;
    grid-template-columns:minmax(0,2fr) minmax(0,1fr);
    grid-template-rows:minmax(0,1fr) minmax(0,1fr) minmax(0,1fr);
  }
  /* 右列行高按各面板折叠状态由 JS updateGridClasses() 内联计算 */

  .win {
    background:var(--panel); border:1px solid var(--line); border-radius:var(--radius);
    backdrop-filter:blur(12px) saturate(140%); -webkit-backdrop-filter:blur(12px) saturate(140%);
    box-shadow:var(--shadow);
    display:flex; flex-direction:column; overflow:hidden; min-height:0; min-width:0;
    transition:border-color .15s;
  }
  .win:hover { border-color:rgba(255,255,255,0.18); }
  .win-title {
    display:flex; align-items:center; gap:10px;
    padding:5px 10px; font-size:12px; flex:0 0 auto;
    background:var(--panel2); border-bottom:1px solid var(--line); user-select:none;
  }
  .win-title .t { font-weight:600; color:var(--fg); white-space:nowrap; }
  .win-title .hud {
    font-family:Consolas,'Courier New',monospace; color:var(--accent);
    font-variant-numeric:tabular-nums; white-space:nowrap;
  }
  .win-title .sub { color:var(--dim); font-size:11px; white-space:nowrap; }
  .win-title .grp { margin-left:auto; display:flex; gap:2px; }
  .btn {
    cursor:pointer; color:var(--dim); background:transparent; border:none;
    font-size:13px; line-height:1; padding:4px 10px; border-radius:999px;
    font-family:inherit;
  }
  .btn:hover { background:rgba(255,255,255,0.08); color:#fff; }
  .btn.active { background:var(--accent); color:#14141c; }
  .btn:disabled { opacity:.35; cursor:default; }
  .win-body { flex:1 1 auto; position:relative; overflow:hidden; background:#000; }
  .win.collapsed .win-body { display:none; }

  /* ── 地图窗口 ─────────────────────────── */
  #win-map  { grid-column:1; grid-row:1 / span 3; }
  #win-map.full {
    position:fixed; inset:34px 0 0 0; z-index:25;
    border-radius:0; border:none;
  }
  #win-cam  { grid-column:2; grid-row:1; }
  #win-fire { grid-column:2; grid-row:2; }
  #win-log  { grid-column:2; grid-row:3; }
  #map-body { background:#0a0a14; }   /* 与 render() 里 canvas 清屏色 #0a0a14 一致 */
  #map-canvas { width:100%; height:100%; display:block; cursor:grab; touch-action:none; }
  #map-canvas:active { cursor:grabbing; }
  #legend {
    position:absolute; left:10px; bottom:10px; z-index:2;
    display:flex; gap:14px; font-size:11px; color:var(--dim);
    background:rgba(22,22,29,0.72); backdrop-filter:blur(8px); -webkit-backdrop-filter:blur(8px);
    padding:5px 12px; border-radius:999px;
    border:1px solid var(--line); pointer-events:none;
  }
  #legend .lg { display:flex; align-items:center; gap:5px; }
  #legend i { display:inline-block; width:10px; height:10px; border-radius:50%; }

  /* ── 驾驶面板 (地图窗口右下角) ───────────── */
  #drive {
    position:absolute; right:10px; bottom:10px; z-index:6;
    background:rgba(24,24,32,0.72); backdrop-filter:blur(12px); -webkit-backdrop-filter:blur(12px);
    border:1px solid var(--line); border-radius:var(--radius);
    box-shadow:var(--shadow);
    padding:8px; display:flex; flex-direction:column; gap:6px;
    font-size:12px; user-select:none;
  }
  #navigation-panel { position:absolute; left:10px; top:10px; z-index:7; max-width:min(360px,90%);
    background:rgba(24,24,32,.94); border:1px solid var(--line); border-radius:10px; padding:8px; font-size:12px; max-height:85%; overflow:auto; }
  #navigation-panel summary { cursor:pointer; }
  #navigation-panel .nav-row { display:flex; gap:5px; flex-wrap:wrap; margin-top:8px; }
  #navigation-panel input, #navigation-panel select { min-width:0; max-width:180px; padding:5px; }
  #nav-status, #nav-path { margin-top:8px; white-space:normal; line-height:1.5; }
  #drive-title { color:var(--dim); font-weight:600; text-align:center; }
  #drive-btns { display:flex; gap:6px; }
  #btn-arm {
    flex:1; padding:6px 8px; border:none; border-radius:999px; cursor:pointer;
    background:rgba(255,255,255,0.07); color:var(--dim); font-size:12px; font-family:inherit;
    transition:background .15s, color .15s;
  }
  #btn-arm.armed { background:var(--accent); color:#14141c; }
  #btn-arm:disabled { opacity:.4; cursor:default; }
  #btn-estop {
    flex:1; padding:6px 8px; border:none; border-radius:999px; cursor:pointer;
    background:rgba(255,90,90,0.14); color:#ff9090; font-size:12px; font-family:inherit;
  }
  #btn-estop:active { background:#e5484d; color:#fff; }
  /* 建图按钮:放在地图窗口标题栏、和视图 glyph **同一组**的首位。
     不另起一个 span.grp —— .grp 带 margin-left:auto,两个 auto 会把空白平分,
     按钮就飘到标题中间去了。配色沿用 #btn-arm 那套。 */
  #btn-map {
    padding:3px 12px; margin-right:6px; border:none; border-radius:999px; cursor:pointer;
    background:rgba(255,255,255,0.07); color:var(--dim); font-size:11px; font-family:inherit;
    transition:background .15s, color .15s; white-space:nowrap;
  }
  #btn-map:hover { color:#fff; }
  #btn-map.on      { background:var(--accent); color:#14141c; }           /* 建图中 */
  #btn-map.partial { background:rgba(255,179,64,0.18); color:#ffd479; }    /* 雷达在,SLAM 没起 */
  #btn-map:disabled { opacity:.4; cursor:default; }
  /* ── 双摇杆 (RC 车模式):左盘纵轴=油门,右盘横轴=转向 ──────────────
     原来是单个 240px 的 2D 盘,放大是为了「同样的手指位移对应的指令更小」,
     免得手抖一下就越过零点把方向翻过去 —— 那是因为 2D 盘得靠角度同时区分
     前后左右。拆成两个单轴盘之后不可能推错轴,那个理由就不成立了,于是缩到
     150px,两个并排也放得下。
     每个盘的活轴准线用 accent 色点亮、另一轴留暗 —— 免得人往死轴方向推。 */
  #joy-pair { display:flex; gap:12px; justify-content:center; }
  .joy-wrap { display:flex; flex-direction:column; align-items:center; gap:2px; }
  .joy-pad {
    width:150px; height:150px; border-radius:50%; position:relative;
    background:
      radial-gradient(circle at 50% 50%, transparent 0 32%,
        rgba(124,140,255,.24) 32% 32.7%, transparent 32.7% 64.5%,
        rgba(124,140,255,.24) 64.5% 65.2%, transparent 65.2%),
      radial-gradient(circle at 50% 50%, rgba(255,255,255,0.06), rgba(0,0,0,0.28));
    border:1px solid var(--line); cursor:grab; touch-action:none;
  }
  .joy-pad::before, .joy-pad::after {
    content:''; position:absolute; background:rgba(122,140,200,.22);
    pointer-events:none;
  }
  .joy-pad::before { left:50%; top:6%; bottom:6%; width:1px; margin-left:-.5px; }
  .joy-pad::after  { top:50%; left:6%; right:6%; height:1px; margin-top:-.5px; }
  .joy-pad.axis-y::before { background:var(--accent); opacity:.55; width:2px; margin-left:-1px; }
  .joy-pad.axis-x::after  { background:var(--accent); opacity:.55; height:2px; margin-top:-1px; }
  .joy-pad:active { cursor:grabbing; border-color:rgba(124,140,255,.5); }
  .joy-knob {
    position:absolute; left:50%; top:50%; width:52px; height:52px; z-index:2;
    margin:-26px 0 0 -26px; border-radius:50%;
    background:var(--accent); opacity:.9; pointer-events:none;
    box-shadow:0 0 16px rgba(124,140,255,.55), inset 0 0 6px rgba(255,255,255,.35);
  }
  .joy-lab { color:var(--dim); font-size:10px; white-space:nowrap; }
  /* ── 单杆模式:一个 2D 盘同时给两轴 ────────────────────────────────
     盘子放大到 180px —— 这里「放大」的理由**又成立了**:2D 盘得靠角度同时
     区分前后左右,盘子小了斜推容易推错角(这正是双杆当初缩到 150px 而不能再
     缩的原因的反面)。180 而不是老版那个 240:240 是按当时 240px 宽的面板定的,
     现在面板要同时容下「双杆 330px / 单杆 180px」两种形态,取中间。
     ⚠️ 别因为「双杆的注释说 240 不成立」就把这里也改小 —— 那条注释讲的
     是**单轴盘**,两者不是一回事。
     axis-y + axis-x 两个类同时给,于是两条准线都被 accent 点亮(两轴都活)。 */
  #joy-single { display:none; flex-direction:column; align-items:center; gap:2px; }
  #joy-2d { width:180px; height:180px; }
  #drive.mode-single #joy-pair   { display:none; }
  #drive.mode-single #joy-single { display:flex; }

  /* ── 摇杆模式切换 (双杆 / 单杆) ───────────── */
  #drive-head { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  #drive-head-right { display:flex; align-items:center; gap:8px; }
  #drive.collapsed > :not(#drive-head):not(#drive-btns) { display:none; } /* 保留急停 */
  #drive.collapsed #btn-arm { display:none; }
  #joy-mode { display:flex; border:1px solid var(--line); border-radius:999px; overflow:hidden; background:rgba(255,255,255,0.04); }
  #joy-mode button {
    padding:3px 11px; border:none; cursor:pointer; font-family:inherit; font-size:11px;
    background:transparent; color:var(--dim); transition:background .15s, color .15s;
  }
  #joy-mode button + button { border-left:1px solid var(--line); }
  #joy-mode button.on { background:var(--accent); color:#14141c; font-weight:600; }
  #drive-val {
    color:var(--accent); text-align:center; font-size:12px;
    font-variant-numeric:tabular-nums; white-space:pre;
  }
  #drive-hint { color:var(--dim); text-align:center; font-size:10px; }

  /* ── 云台 (二维云台,走串口 —— 和电机那条 GPIO 链零共享) ────────────
     放在驾驶面板里而不是相机窗口:相机窗口整个被 <img> 占满,叠一个盘上去会挡住
     画面;而这里本来就集中了「操作这个机器人」的全部控件。
     ⚠️ 盘径不写死 —— 用 `.joy-pad` 的 150px。**不是**单杆那个 180px:单杆放大
        是为了「靠角度区分前后左右」,而云台两轴是**独立的 pan/tilt**,没有方向
        可言,不需要那份精度。`web_drive_test.js` 的 PAD_R 里记 150,与这里一致。
     ⚠️ 圆门(斜推到角是 0.707/0.707,不是满舵)照旧对它生效 —— 见 padUpdate。
        v1 决定不为云台开分支:padUpdate 被第 11 节整段回归盯着,加分支会把那些
        用例的语义搞浑。要方形门就另开一次改动。 */
  #gimbal {
    border-top:1px solid var(--line); margin-top:2px; padding-top:6px;
    display:flex; flex-direction:column; align-items:center; gap:4px;
  }
  #gimbal-title { color:var(--dim); font-weight:600; font-size:11px; }
  #gimbal-val {
    color:var(--accent); text-align:center; font-size:12px;
    font-variant-numeric:tabular-nums; white-space:pre;
  }
  #gimbal-hint { color:var(--dim); text-align:center; font-size:10px; }

  /* ── 云台摇杆模式切换 (双杆 / 单杆) ───────────── */
  #gimbal-head { display:flex; align-items:center; justify-content:space-between; gap:10px; }
  #gimbal-pair { display:flex; gap:12px; justify-content:center; }
  #gimbal-single { display:none; flex-direction:column; align-items:center; gap:2px; }
  #gimbal.mode-single #gimbal-pair   { display:none; }
  #gimbal.mode-single #gimbal-single { display:flex; }
  #gimbal-mode { display:flex; border:1px solid var(--line); border-radius:999px; overflow:hidden; background:rgba(255,255,255,0.04); }
  #gimbal-mode button {
    padding:3px 11px; border:none; cursor:pointer; font-family:inherit; font-size:11px;
    background:transparent; color:var(--dim); transition:background .15s, color .15s;
  }
  #gimbal-mode button + button { border-left:1px solid var(--line); }
  #gimbal-mode button.on { background:var(--accent); color:#14141c; font-weight:600; }

  /* ── 相机窗口 ─────────────────────────── */
  #cam-img { width:100%; height:100%; object-fit:contain; display:block; background:#000; }
  #cam-err {
    position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
    color:var(--dim); font-size:13px; background:#000;
  }

  /* ── 火情窗口 ─────────────────────────── */
  #fire-img { width:100%; height:100%; object-fit:contain; display:block; background:#000; }
  #fire-err {
    position:absolute; inset:0; display:flex; align-items:center; justify-content:center;
    color:var(--dim); font-size:13px; background:#000;
  }
  #fire-hud.ok  { color:#4ade80; text-shadow:0 0 8px rgba(74,222,128,.5); }
  #fire-hud.alarm { color:#ff5a5a; text-shadow:0 0 10px rgba(255,90,90,.7); animation:alarm 1s ease-in-out infinite; }
  #fire-hud.off { color:var(--dim); }
  #s-fire.alarm { color:#ff5a5a; }
  #s-fire.ok { color:#4ade80; }

  /* ── 日志窗口 ─────────────────────────── */
  #log-list {
    height:100%; overflow-y:auto; padding:4px 0;
    background:#0e0e15; font-family:Consolas,'Courier New',monospace;
    font-size:11px; line-height:1.55;
  }
  .log-line {
    padding:0 8px; white-space:pre-wrap; word-break:break-all;
    color:var(--fg);
  }
  .log-line:hover { background:rgba(255,255,255,0.05); }
  .lt { color:#5a6070; margin-right:8px; font-variant-numeric:tabular-nums; }
  .l-tag { margin-right:8px; font-weight:600; }
  .lv-debug .l-msg { color:var(--dim); }
  .lv-warn .l-msg, .lv-warning .l-msg { color:#ffb454; }
  .lv-error .l-msg { color:#ff7a7a; }
  .lv-critical .l-msg { color:#ff5a5a; }
  #log-count { font-variant-numeric:tabular-nums; }

  /* ── 响应式 ───────────────────────────── */
  @media (max-width:760px) {
    #workspace { grid-template-columns:1fr; grid-template-rows:auto; }
    #win-map  { grid-column:1; grid-row:1; height:55vh; }
    #win-cam  { grid-column:1; grid-row:2; height:28vh; }
    #win-fire { grid-column:1; grid-row:3; height:28vh; }
    #win-log  { grid-column:1; grid-row:4; height:28vh; }
    /* 双摇杆在窄屏收一档:330px 的面板会把 55vh 的地图窗口占掉大半。
       收到 112px 后整块 ≈254px,与改动前那个 240px 单盘的面板相当。
       R 是在 padUpdate 里按 clientWidth 现算的,所以这里改尺寸不用动 JS。 */
    .joy-pad { width:112px; height:112px; }
    .joy-knob { width:40px; height:40px; margin:-20px 0 0 -20px; }
    #joy-2d { width:140px; height:140px; }
    /* 顶栏瘦身:Map 分辨率 / Path 点数对窄屏不重要,藏掉防溢出 */
    #status-bar .hide-narrow { display:none; }
  }

  /* ── 微动效 ─────────────────────────────── */
  @keyframes pulse {
    0%, 100% { box-shadow:0 0 0 0 rgba(74,222,128,.45); }
    50%      { box-shadow:0 0 0 6px rgba(74,222,128,0); }
  }
  @keyframes alarm {
    0%, 100% { opacity:1; }
    50%      { opacity:.55; }
  }

/* Task-focused console: viewing on the left, controls on the right. */
:root { --bg:#10191e; --bg-2:#132128; --accent:#77d5bd; --accent-soft:#23463f; --dim:#9aafb5; --line:#2b3d45; --radius:16px; }
body { background:#10191e; }
#status-bar { height:64px; gap:18px; padding:0 24px; background:#142128; font-size:12px; }
.brand { font-size:19px; color:#edf5f3; white-space:nowrap; }.brand small { font-size:12px; color:var(--dim); font-weight:400; margin-left:10px; }
#status-bar .hide-narrow,#status-bar span:has(>#s-fps) { display:none; }
#btn-estop { flex:0 0 auto; font-size:15px; font-weight:700; padding:11px 24px; border-radius:10px; background:#792f39; color:#fff; border:1px solid #b85864; }
#btn-estop:disabled { opacity:.5; cursor:not-allowed; }
#workspace { top:64px; padding:18px; gap:14px; grid-template-columns:minmax(0,1fr) minmax(0,1fr) 370px; grid-template-rows:minmax(260px,1fr) minmax(180px,.62fr) 42px; }
#win-map { grid-column:1 / span 2; grid-row:1; }#win-cam { grid-column:1; grid-row:2; }#win-fire { grid-column:2; grid-row:2; }#win-log { grid-column:1 / span 2; grid-row:3; }
#win-map.full { inset:64px 0 0; }
.win { background:#17262e; box-shadow:none; backdrop-filter:none; }
.win-title { min-height:42px; padding:9px 14px; background:#1b2c34; }.win-title .t { font-size:13px; }.win-title .hud { font-size:11px; }
#controls-panel { grid-column:3; grid-row:1 / span 3; overflow:auto; min-height:0; padding-right:4px; scrollbar-width:thin; scrollbar-color:#3c595f transparent; }
.control-intro { padding:4px 4px 18px; }.eyebrow { color:var(--accent); font-size:11px; letter-spacing:2px; }.control-intro h2 { font-size:20px; margin:8px 0; font-weight:600; }.control-intro p,.task-help { color:var(--dim); font-size:12px; line-height:1.8; }
#drive { position:static; padding:16px; background:#17262e; backdrop-filter:none; box-shadow:none; gap:12px; margin-bottom:14px; }
#drive-title { color:var(--fg); font-size:15px; }#drive-head { gap:8px; }#btn-arm { padding:12px; border-radius:10px; background:#24483f; color:#acebd8; font-size:14px; }#btn-arm.armed { background:#77d5bd; color:#10201c; }
#drive-hint,#gimbal-hint { white-space:normal; line-height:1.6; }#gimbal { padding-top:14px; }#gimbal-title { font-size:12px; }
#navigation-panel { position:static; max-width:none; max-height:none; overflow:visible; padding:0; border:0; background:none; }
.task-group,.task-state { border:1px solid var(--line); background:#17262e; border-radius:14px; margin-bottom:12px; padding:14px; }
.task-group>summary { font-size:14px; font-weight:600; padding:3px 0; }.step { color:var(--accent); margin:0 8px; font-size:12px; font-variant-numeric:tabular-nums; }
.task-group[open]>.task-help { margin:12px 0; }.task-editor,#autonomy-panel { border-top:1px solid var(--line); padding-top:12px; margin-top:14px; color:var(--dim); }
#navigation-panel .nav-row { gap:8px; margin-top:12px; }#navigation-panel .btn,#btn-map { border-radius:8px; background:#263b44; color:#e0eeeb; padding:10px 12px; font-size:12px; line-height:1.4; }
#navigation-panel .btn:disabled { opacity:.38; }#navigation-panel input,#navigation-panel select { background:#101e25; color:#e0eeeb; border:1px solid #3b535c; border-radius:7px; padding:9px; max-width:100%; font:inherit; }
#nav-map,#patrol-list { width:100%; }#nav-name,#patrol-name { flex:1; width:140px; }#nav-go,#auto-patrol { background:#285547!important; }#nav-cancel,#auto-stop { color:#ffc4bb!important; }
#nav-status,#auto-status,#patrol-status,#patrol-draft-status,#nav-path { font-size:12px; color:var(--dim); line-height:1.7; overflow-wrap:anywhere; }.task-state>strong { font-size:12px; }.report-link,#auto-report { display:inline-block; color:var(--accent); margin-top:14px; font-size:12px; }
[hidden] { display:none!important; }#autonomy-alert { top:72px!important; }
button:focus-visible,summary:focus-visible,a:focus-visible { outline:2px solid var(--accent); outline-offset:3px; }
@media(max-width:1100px) { #workspace { grid-template-columns:minmax(0,1fr) minmax(0,1fr) 340px; padding:10px; gap:10px; }#drive { padding:12px; }.joy-pad { width:140px; height:140px; }#status-bar { padding:0 16px; gap:12px; }#s-clock { display:none; } }
@media(max-width:760px) { body { overflow:auto; }#status-bar { height:60px; padding:0 12px; gap:10px; }.brand { font-size:16px; }.brand small,#status-bar span:has(>#s-scan),#status-bar span:has(>#s-fire) { display:none; }#btn-estop { margin-left:auto; padding:9px 16px; }#workspace { position:relative; top:auto; margin-top:60px; display:flex; flex-direction:column; padding:10px; }#win-map { order:0; height:48vh; min-height:300px; flex:none; }#controls-panel { order:1; overflow:visible; padding:0; }.control-intro { padding:14px 4px; }#win-cam { order:2; }#win-fire { order:3; }#win-log { order:4; }#win-cam,#win-fire { height:240px; flex:none; }#win-log { height:200px; flex:none; }#win-log.collapsed,#win-cam.collapsed,#win-fire.collapsed { height:42px; }#win-map.full { inset:60px 0 0; height:auto; }.joy-pad { width:140px; height:140px; }#hud { display:none; } }


#drive.collapsed #joy-mode { display:none; }
#drive.collapsed { gap:0; }#drive.collapsed #drive-btns { display:none; }
#map-empty { position:absolute; inset:0; display:flex; flex-direction:column; align-items:center; justify-content:center; gap:14px; pointer-events:none; text-align:center; color:#81959e; padding:24px; }
#map-empty strong { font-size:18px; font-weight:500; color:#c7d9dd; }#map-empty span { font-size:13px; line-height:1.9; }

#win-fire { grid-column:1 / span 2; }
#speed-gears { display:flex; align-items:center; gap:8px; }#speed-gears span { margin-right:auto; color:var(--dim); }
#speed-gears button { background:#253b44; color:var(--fg); border:1px solid var(--line); border-radius:8px; padding:8px 22px; cursor:pointer; }
#speed-gears button.on { background:var(--accent); color:#10201c; }#gear-hint { color:var(--dim); font-size:12px; line-height:1.6; }
  .mapping-only,.control-only,.mission-only { display:none; }
</style>
<link rel="stylesheet" href="/mapping.css?v=20261023-odom1">
<link rel="stylesheet" href="/control.css?v=20261021-drive3">
<link rel="stylesheet" href="/missions.css?v=20261023-navigation1">
</head>
<body class="{{ page_class }}">

<div id="status-bar">
  <strong class="brand">犀牛一号 <small>控制台</small></strong>
  <div class="mapping-only mapping-brand"><span class="brand-mark" aria-hidden="true">R</span><strong>犀牛一号</strong><span class="brand-divider"></span><span>建图工作台</span></div>
  <div class="control-only control-brand"><span class="brand-mark" aria-hidden="true">R</span><strong>犀牛一号</strong><span class="brand-divider"></span><span>遥控工作台</span></div>
  <div class="mission-only mission-brand"><span class="brand-mark" aria-hidden="true">R</span><strong>犀牛一号</strong></div>
  <nav class="page-links" aria-label="功能页面"><a href="/mapping">返回建图</a><a href="/control">遥控驾驶</a><details class="mission-menu"><summary>自动任务⌄</summary><div><a href="/navigation">定位与导航</a><a href="/explore">自动建图</a><a href="/patrol">路线巡检</a><a href="/inspection">检测与报告 ↗</a></div></details></nav>
  <span id="status-conn"><span id="conn-dot"></span><span id="conn-text">未连接</span></span>
  <span class="hide-narrow"><span class="stat-label">环境地图</span><span id="s-map">—</span></span>
  <span><span class="stat-label">雷达</span><span id="s-scan">—</span></span>
  <span class="hide-narrow"><span class="stat-label">轨迹</span><span id="s-trajectory">0</span></span>
  <span><span class="stat-label">帧率</span><span id="s-fps">0</span></span>
  <span><span class="stat-label">火情检测</span><span id="s-fire">—</span></span>
  <span class="spacer"></span>
  <span id="s-clock">--:--:--</span>
          <button id="btn-estop" disabled>急停</button>
</div>
<div id="autonomy-alert" hidden role="alert" style="position:fixed;top:48px;left:10%;right:10%;z-index:100;background:#642522;border:2px solid #f48772;border-radius:8px;padding:16px;color:#fff;white-space:normal"><span id="autonomy-alert-message"></span> <a id="autonomy-alert-evidence" hidden target="_blank" rel="noopener" style="text-decoration:underline">查看现场证据 ↗</a></div>

<div id="workspace">
<section class="mission-only mission-heading"><div><div class="page-eyebrow">SUPERVISED MISSIONS</div><h1 id="mission-title">自动任务</h1><p id="mission-description">准备、预览，再由现场操作者启动。</p></div><div class="mission-tabs"><a href="/explore">自动建图</a><a href="/patrol">路线巡检</a><a href="/inspection">检测与报告 ↗</a></div></section>
<section class="control-only control-heading"><div><div class="page-eyebrow">REMOTE CONTROL</div><h1>遥控工作台</h1><p>驾驶与云台独立控制，松开摇杆即停止。</p></div><span id="ctl-owner" class="control-owner">等待连接</span></section>
<section class="mapping-only mapping-heading"><div><div class="page-eyebrow">MAPPING WORKSPACE</div><h1>一步一步，画出环境。</h1><p>启动建图，缓慢驾驶，完成后保存地图。</p><div class="mobile-feed" id="mp-mobile-status">等待传感器数据</div></div><span class="mapping-session"><i></i><span id="mp-session">等待设备连接</span></span></section>
<aside id="controls-panel" aria-label="机器人操作">
<section class="mission-only mission-card" id="mission-health"><div class="card-heading"><h2>启动条件</h2><span id="mission-ready" class="status-pill">等待连接</span></div><ul id="mission-checks"></ul><details id="mission-calibration"><summary>实车标定缺项</summary><div id="mission-measurements">正在读取配置…</div><p>需要现场实测最大车轮转角，以及固定油门档位下的距离和耗时。请同时记录档位和地面条件；软件不会根据 PWM 比例推算实际车速。</p></details></section>
<section class="mission-only mission-card" id="mission-setup"><span class="mission-step">01 / 准备</span><h2 id="mission-setup-title">准备任务</h2><p id="mission-setup-note"></p><div id="mission-setup-controls"></div></section>
<section class="mission-only mission-card" id="mission-route"><span class="mission-step">02 / 预览</span><h2 id="mission-route-title">查看任务目标</h2><div id="mission-route-controls"></div><p id="mission-preview-note">配置和预览不会启用电机。</p></section>
<section class="mission-only mission-card" id="mission-run"><span class="mission-step">03 / 执行</span><h2>现场监督执行</h2><p id="mission-run-note">标定和定位就绪后，由当前页面启用电机，再明确启动任务。</p><div id="mission-run-controls"></div><div class="mission-progress"><span id="mission-stage">等待任务</span><strong id="mission-progress">0</strong><small>已完成目标 / 点位</small></div><div id="mission-runtime"></div><div id="mission-runtime-controls"></div></section>
<section class="mapping-only mapping-odometry" aria-label="实时里程计">
<div class="card-heading"><h2>实时里程计</h2><span id="mp-odometry-state" class="status-pill">等待数据</span><button type="button" class="odometry-collapse" id="btn-odometry-collapse" aria-expanded="true" title="折叠/展开实时里程计">收起</button></div>
<p class="odometry-source">雷达 + GY-521 · /odom</p>
<div class="odometry-toolbar"><span>相对测量起点</span><button type="button" id="mp-odometry-zero" disabled>设为测量起点</button></div>
<div class="odometry-grid">
<div><span>前进 / 后退 cm</span><strong id="mp-odometry-forward">—</strong></div>
<div><span>左 / 右偏移 cm</span><strong id="mp-odometry-lateral">—</strong></div>
<div><span>转角 °</span><strong id="mp-odometry-turn">—</strong></div>
<div><span>直线位移 cm</span><strong id="mp-odometry-distance">—</strong></div>
<div><span>前进速度 m/s</span><strong id="mp-odometry-speed">—</strong></div>
<div><span>转动速度 °/s</span><strong id="mp-odometry-rate">—</strong></div>
</div><p id="mp-odometry-note" class="odometry-note">停车后设置起点，再对照卷尺。前进、左偏、左转为正。</p>
<details class="odometry-raw"><summary>原始坐标与航向</summary><div>X <span id="mp-odometry-x">—</span> m · Y <span id="mp-odometry-y">—</span> m · θ <span id="mp-odometry-yaw">—</span>°</div><small id="mp-odometry-frame">等待坐标系</small></details>
</section>
<section class="mapping-only mapping-health" aria-label="建图状态"><div class="card-heading"><h2>当前状态</h2><span id="mp-status" class="status-pill">未连接</span></div><p id="mp-status-note">正在连接机器人…</p><div class="sensor-grid">
<div><span class="sensor-label"><i id="mp-scan-dot"></i>雷达</span><strong id="mp-scan">等待数据</strong><small id="mp-scan-age">—</small></div>
<div><span class="sensor-label"><i id="mp-odom-dot"></i>里程计</span><strong id="mp-odom">等待数据</strong><small id="mp-odom-age">—</small></div>
<div><span class="sensor-label"><i id="mp-imu-dot"></i>GY-521</span><strong id="mp-imu">等待数据</strong><small id="mp-imu-age">—</small></div>
</div><div class="motor-summary"><span>电机状态</span><strong id="mp-motor">未连接</strong></div></section><div class="control-intro"><span class="eyebrow">操作面板</span><h2>今天想让小犀做什么？</h2><p>手动驾驶，或按步骤建图、导航与巡检。</p></div>      <div id="drive" class="collapsed">
        <div id="drive-head">
          <div id="drive-title">手动驾驶</div>
          <div id="drive-head-right">
            <div id="joy-mode" role="group" aria-label="摇杆模式">
              <button type="button" data-mode="dual" title="左杆油门 / 右杆转向;两轴独立,可同时满舵">双杆</button>
              <button type="button" data-mode="single" title="一个盘同时给油门与转向;圆门,斜推时两轴各占一部分">单杆</button>
            </div>
            <button class="btn" id="btn-drive-collapse" aria-expanded="false" title="折叠/展开驾驶面板">展开</button>
          </div>
        </div>
        <p class="task-help">先启用电机，再按住摇杆驾驶。松手即停止。</p>
        <div id="speed-gears" role="group" aria-label="车速档位"><span>车速档位</span><button type="button" id="gear-low" aria-pressed="true">低速30%</button><button type="button" id="gear-high" aria-pressed="false">高速</button></div><div id="gear-hint">低速 · 油门上限 30%</div>
        <div id="drive-btns">
          <button id="btn-arm" disabled>启用电机</button>

        </div>
        <div id="joy-pair">
          <div class="joy-wrap">
            <div class="joy-pad axis-y" id="joy-thr"><div class="joy-knob" id="knob-thr"></div></div>
            <div class="joy-lab">油门 ↑前 ↓后</div>
          </div>
          <div class="joy-wrap">
            <div class="joy-pad axis-x" id="joy-steer"><div class="joy-knob" id="knob-steer"></div></div>
            <div class="joy-lab">转向 ←左 →右</div>
          </div>
        </div>
        <div id="joy-single">
          <div class="joy-pad axis-y axis-x" id="joy-2d"><div class="joy-knob" id="knob-2d"></div></div>
          <div class="joy-lab">↑前 ↓后　←左 →右</div>
        </div>
        <div id="drive-val">前进 +0.00　转向 +0.00</div>
        <div id="drive-hint">左杆油门 右杆转向,或 WASD/方向键</div>
        <div id="gimbal" class="mode-single">
          <div id="gimbal-head">
            <div id="gimbal-title">调整相机方向</div>
            <div id="gimbal-mode" role="group" aria-label="云台摇杆模式">
              <button type="button" data-mode="dual" title="水平 pan / 垂直 tilt;两轴独立,可同时满量程">双杆</button>
              <button type="button" data-mode="single" title="一个盘同时给 pan+tilt;圆门">单杆</button>
            </div>
          </div>
          <div id="gimbal-pair">
            <div class="joy-wrap">
              <div class="joy-pad axis-x" id="joy-gimbal-pan"><div class="joy-knob" id="knob-gimbal-pan"></div></div>
              <div class="joy-lab">水平 ←左 →右</div>
            </div>
            <div class="joy-wrap">
              <div class="joy-pad axis-y" id="joy-gimbal-tilt"><div class="joy-knob" id="knob-gimbal-tilt"></div></div>
              <div class="joy-lab">垂直 ↑上 ↓下</div>
            </div>
          </div>
          <div id="gimbal-single">
            <div class="joy-pad axis-y axis-x" id="joy-gimbal"><div class="joy-knob" id="knob-gimbal"></div></div>
            <div class="joy-lab">↑上 ↓下　←左 →右</div>
          </div>
          <div id="gimbal-val">dx +0　dy +0</div>
<div id="gimbal-hint">云台：Shift+WASD 或拖盘（不必「启用电机」）</div>
        </div>
      </div><div id="navigation-panel">
<details class="task-group" id="mapping-actions"><summary><span class="step">01</span> 建立地图</summary><h2 class="mapping-only">建图操作</h2>
<p class="task-help console-only">第一次使用：启动建图，用上方摇杆缓慢移动，完成后保存地图。</p><p class="mapping-only mapping-action-hint">启动后保持静止约 3 秒，等待陀螺仪校准。</p>
<div class="nav-row"><button id="btn-map" disabled>建图</button></div>
<label class="mapping-only save-label" for="nav-name">地图名称</label><div class="nav-row save-map-row"><input id="nav-name" placeholder="例如 room_01" maxlength="64" autocomplete="off" spellcheck="false" aria-label="地图名称"><button class="btn" id="nav-save" disabled>保存地图</button></div><p class="mapping-only save-feedback" id="mp-save-feedback" role="status" aria-live="polite">支持英文字母、数字、下划线和短横线。</p><a class="mapping-only map-editor-link" href="/map-editor">地图清理 · 玻璃边界与伪区域 →</a>
<details id="autonomy-panel"><summary>自动建图（需完成实车标定）</summary>
<p class="task-help">准备 → 查看探索目标 → 开始；未完成标定时不能自动行驶。</p>
<div class="nav-row"><button class="btn" id="auto-prepare">准备自动建图</button><button class="btn" id="auto-preview">查看探索目标</button></div><div class="nav-row"><button class="btn" id="auto-explore" disabled>开始自动建图</button><button class="btn" id="auto-finish" disabled>结束建图并保存</button></div>
</details></details>
<details class="task-group console-only"><summary><span class="step">02</span> 定位与导航</summary>
<p class="task-help">① 选地图并加载定位　② 在地图设置真实位置和朝向　③ 选择目标，预览后再开始导航。</p>
<div class="nav-row"><select id="nav-map" aria-label="已保存地图"></select><button class="btn" id="nav-load">加载定位</button><button class="btn" id="nav-stop" disabled>停止定位</button></div><div class="nav-row"><button class="btn" id="nav-initial" disabled>设置当前位置</button><button class="btn" id="nav-target" disabled>选择目标并预览</button></div><div class="nav-row"><button class="btn" id="nav-go" disabled>开始导航</button><button class="btn" id="nav-cancel">取消导航</button></div>
</details>
<details class="task-group console-only" id="patrol-panel"><summary><span class="step">03</span> 巡检任务</summary>
<p class="task-help">先完成定位，再选择任务。新任务需添加巡检点并保存；预览路线后再执行。</p>
<div class="nav-row"><select id="patrol-list" aria-label="已保存巡检任务"></select><button class="btn" id="patrol-load">载入任务</button><button class="btn" id="patrol-new">新建</button></div>
<details class="task-editor"><summary>新建 / 编辑巡检点</summary>
<div class="nav-row"><input id="patrol-name" placeholder="任务英文名" maxlength="64"><button class="btn" id="patrol-save">保存任务</button></div><div class="nav-row"><label>循环 <input id="patrol-cycles" type="number" value="1" min="1" max="100" style="width:60px"> 次</label><label>新增点停留 <input id="patrol-dwell" type="number" value="5" min="0" max="3600" style="width:65px"> 秒</label></div><div class="nav-row"><button class="btn" id="patrol-add">地图上添加点</button><button class="btn" id="patrol-preview">预览巡检路线</button></div><div id="patrol-map"></div><ol id="patrol-points" style="max-height:180px;overflow:auto;padding-left:22px"></ol>
</details>
<div class="nav-row"><button class="btn" id="auto-patrol" disabled>执行所选巡检任务</button></div>
<div id="patrol-draft-status">尚未配置巡检点</div><div id="patrol-status">仅配置和路线预览，不执行巡检</div>
<a href="/inspection" target="_blank" rel="noopener" class="report-link">查看巡检记录与报告 ↗</a>
</details>
<div class="task-state console-only"><strong>任务状态</strong>
<div id="nav-status">等待连接；先保存地图，再加载定位</div><div id="nav-path"></div><div id="auto-status" style="white-space:normal;line-height:1.6;margin-top:8px">等待连接；发现疑似火情即停车上报</div>
<div class="nav-row"><button class="btn" id="auto-stop">停止自动任务</button><button class="btn" id="auto-ack" disabled>已人工确认告警</button></div>
<a id="auto-report" hidden target="_blank" rel="noopener">查看本次巡检报告 ↗</a>
</div></div><section class="control-only control-card" id="control-gimbal-panel" aria-label="云台控制"></section></aside>

  <section class="control-only control-camera win" id="control-camera" aria-label="摄像头主画面"><header class="control-camera-head"><div><i id="ctl-camera-dot"></i><strong>摄像头画面</strong><span id="ctl-camera-state">等待连接</span></div><button type="button" id="ctl-video-toggle" aria-pressed="true" title="暂停或恢复摄像头画面">暂停画面</button></header><div class="control-video-body"><img id="ctl-video" alt="机器人摄像头实时画面" hidden><div id="ctl-video-placeholder" role="status"><span class="camera-placeholder-icon" aria-hidden="true">◉</span><strong id="ctl-video-message">正在连接摄像头…</strong><span id="ctl-video-help">画面就绪后可启用电机，使用摇杆驾驶。</span></div><span class="video-crosshair" aria-hidden="true"></span><span id="ctl-motor-badge" class="video-motor-badge">电机未启用</span></div><footer class="control-camera-foot"><span>松手即停 · 离开页面自动归零</span><span id="ctl-video-format">—</span><span id="ctl-video-latency" title="估计相机读出至网页解码的耗时；不含传感器曝光，网络按往返一半估算。">— ms</span><span id="ctl-video-fps">—</span></footer></section>

  <section class="win" id="win-map">
    <header class="win-title">
      <span class="t">环境地图</span><span class="mapping-only map-live-label" id="mp-map-caption">等待地图</span>
      <span class="hud" id="hud">x:— y:— θ:—</span>
      <span class="grp">
        <button type="button" class="btn mapping-only" id="mp-camera-toggle" aria-label="显示摄像头" aria-controls="mp-camera-window" aria-pressed="false" title="显示摄像头">相机</button>
        <button class="btn" id="btn-follow" title="跟随机器人">◉</button>
        <button class="btn" id="btn-zoomout" title="缩小">−</button>
        <button class="btn" id="btn-zoomin" title="放大">+</button>
        <button class="btn" id="btn-reset" title="复位视图">⌂</button>
        <button class="btn" id="btn-full" title="最大化/还原地图">⛶</button>
        <button class="btn mapping-only" id="btn-map-collapse" aria-expanded="true" title="折叠/展开实时建图">收起</button>
      </span>
    </header>
    <div class="win-body" id="map-body">
      <section class="mapping-only mapping-camera" id="mp-camera-window" aria-label="实时摄像头" hidden><header><strong>实时摄像头</strong><span id="mp-camera-caption">等待画面</span><button type="button" id="mp-camera-close" aria-label="关闭摄像头" title="关闭摄像头">×</button></header><div class="camera-preview-body"><img id="mp-camera-image" alt="机器人摄像头实时画面" hidden><div id="mp-camera-message" role="status">等待画面</div></div></section>
      <canvas id="c" aria-label="实时环境地图"></canvas><div class="mapping-only map-guide">拖动平移<span>·</span>滚轮缩放<span>·</span>双击跟随</div><section class="mapping-only map-position" aria-label="位置、航向与加速度"><div class="map-pose-row"><div><span>X / 米</span><strong id="mp-x">—</strong></div><div><span>Y / 米</span><strong id="mp-y">—</strong></div><div><span>航向角 / 地图坐标</span><strong id="mp-yaw">—</strong></div></div><div class="map-accel-heading"><span>三轴加速度 · m/s²</span><small id="mp-accel-state">等待数据</small></div><div class="map-accel-row"><div><span>X 轴</span><strong id="mp-accel-x">—</strong></div><div><span>Y 轴</span><strong id="mp-accel-y">—</strong></div><div><span>Z 轴</span><strong id="mp-accel-z">—</strong></div></div><div class="map-accel-note"><span>模块坐标 · 含重力</span><span>合值 <b id="mp-accel-total">—</b></span></div></section><div id="map-empty"><strong>等待地图数据</strong><span>首次使用，请展开“建立地图”启动建图。<br>已有地图，请在“定位与导航”中加载。</span></div>
      <div id="legend">
        <span class="lg"><i style="background:#999"></i>地图</span>
        <span class="lg"><i style="background:#00d2ff"></i>&lt;1m</span>
        <span class="lg"><i style="background:#7bed9f"></i>1–3m</span>
        <span class="lg"><i style="background:#ffa502"></i>&gt;3m</span>
      </div>

    </div>
  </section>

  <section class="win" id="win-fire">
    <header class="win-title">
      <span class="t">现场检测画面</span>
      <span class="hud off" id="fire-hud">—</span>
      <span class="sub" id="fire-info">—</span>
      <span class="grp">
        <button class="btn" id="btn-fire-collapse" title="折叠/展开火情">—</button>
      </span>
    </header>
    <div class="win-body" id="fire-body">
      <img id="fire-img" alt="fire" />
      <div id="fire-err">等待检测画面</div>
    </div>
  </section>

  <section class="win collapsed" id="win-log">
    <header class="win-title">
      <span class="t">运行日志</span>
      <span class="sub" id="log-count">0</span>
      <span class="grp">
        <button class="btn active" id="btn-log-scroll" title="自动滚动">⇓</button>
        <button class="btn" id="btn-log-clear" title="清空">✕</button>
        <button class="btn" id="btn-log-collapse" title="折叠/展开日志">展开</button>
      </span>
    </header>
    <div class="win-body" id="log-body">
      <div id="log-list"></div>
    </div>
  </section>

</div>

<script>
// ── 状态 ─────────────────────────────────────────────────────────
const mappingPage = document.body.classList.contains('mapping-page');
const controlPage = document.body.classList.contains('control-page');
const missionPage = document.body.classList.contains('mission-page');
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const mapBody = document.getElementById('map-body');

let mapImg = null, mapMeta = null;
let scanPoints = [];        // flat [x,y,...] in map frame
let trajectoryPoints = [];  // flat [x,y,...]
let trajHave = 0;           // 已收到的**累计**点数(裁剪不减少它,只用于对齐 from)
let robotPose = null;       // {x, y, yaw} in odom frame
let fps = 0, frameCount = 0, lastFpsTime = performance.now();

// 视图变换
let vScale = 40;        // px per meter
let vCx = 0, vCy = 0;   // world coords at canvas center
let mapOriginX = 0, mapOriginY = 0;

// 指针 / 跟随
let dragging = false, dragLast = null;
let follow = true;

// ── 画布尺寸 = 地图窗口大小 ─────────────────────────────────────
function resize() {
  const w = mapBody.clientWidth, h = mapBody.clientHeight;
  canvas.width  = Math.max(1, w);
  canvas.height = Math.max(1, h);
}
window.addEventListener('resize', resize);
new ResizeObserver(resize).observe(mapBody);
resize();

// ── SocketIO ────────────────────────────────────────────────────
const CLIENT_ID = (function(){try{var v=localStorage.getItem('rhinoDriveClient');if(!v){v=Math.random().toString(36).slice(2)+Date.now().toString(36);localStorage.setItem('rhinoDriveClient',v);}return v;}catch(e){return 'anon';}})();
const socket = io({query:{view:controlPage ? 'control' : mappingPage || missionPage ? 'mapping' : 'console', client: CLIENT_ID}});
// Video must not fill the driver's Engine.IO polling queue. Camera disconnects
// never release driving ownership: they belong to a separate, non-driver SID.
const cameraSocket = controlPage ? io({forceNew:true,query:{view:'camera'}}) : socket;
const connDot = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');

socket.on('connect', () => {
  connDot.className = 'live';
  connText.textContent = '已连接';
  // 只开启检测画面，避免重复传输原始相机流
  if (!mappingPage && !controlPage && !missionPage) socket.emit('fire_control', { action: 'start' });
});

socket.on('disconnect', () => {
  connDot.className = '';
  connText.textContent = '未连接';
});

socket.on('map', data => {
  mapMeta = data;
  mapOriginX = data.origin_x;
  mapOriginY = data.origin_y;
  document.getElementById('s-map').textContent =
    `${data.width}×${data.height} @ ${(data.resolution*100).toFixed(0)}cm`;
  const img = new Image();
  img.onload = () => { mapImg = img; document.getElementById('map-empty').hidden = true; };
  img.src = 'data:image/png;base64,' + data.png;
});

socket.on('scan', data => {
  scanPoints = data.points;
  document.getElementById('s-scan').textContent = `${data.count} pts`;
});

// 服务端只发增量(没有新点就不发)。reset=true → 整体重建。
// from 是这批点里第一个点的**累计序号**:连接时服务端会补发一次全量快照,
// 紧随其后的第一次增量必然和它有重叠,靠 from 把已有的那几条跳掉,
// 否则折线里会出现重复顶点。
socket.on('trajectory', data => {
  if (data.reset || trajectoryPoints.length === 0) {
    trajectoryPoints = data.points;
    trajHave = data.total ?? Math.floor(data.points.length / 2);
  } else {
    const from = data.from ?? 0;
    const skip = Math.max(0, trajHave - from) * 2;   // 一个点两个坐标
    if (skip < data.points.length) {
      trajectoryPoints = trajectoryPoints.concat(data.points.slice(skip));
    }
    trajHave = Math.max(trajHave, data.total ?? trajHave);
  }
  if (trajectoryPoints.length > 5000 * 2) {   // 和服务端 TRAJ_MAX 对齐
    trajectoryPoints = trajectoryPoints.slice(-5000 * 2);
  }
  document.getElementById('s-trajectory').textContent =
    `${data.total ?? Math.floor(trajectoryPoints.length/2)} pts`;
});

socket.on('pose', d => {
  robotPose = d;
  const deg = Math.round(d.yaw * 180 / Math.PI);
  document.getElementById('hud').textContent =
    `x:${d.x.toFixed(2)} y:${d.y.toFixed(2)} θ:${deg}°`;
});

// ── 检测画面 ─────────────────────────────────────────────
const fireImg = document.getElementById('fire-img');
const fireHud = document.getElementById('fire-hud');
const fireInfo = document.getElementById('fire-info');
const fireErr = document.getElementById('fire-err');
const sFire = document.getElementById('s-fire');

socket.on('fire_frame', data => {
  fireImg.src = 'data:image/jpeg;base64,' + data.img;
  fireErr.style.display = 'none';
});

socket.on('fire_status', st => {
  sFire.className = '';
  if (st.error) {
    fireHud.textContent = 'ERR';
    fireHud.className = 'hud off';
    fireInfo.textContent = '火情检测离线';
    sFire.textContent = 'off';
  } else if (st.fire) {
    fireHud.textContent = '🔥 ' + Math.round((st.confidence || 0) * 100) + '%';
    fireHud.className = 'hud alarm';
    sFire.textContent = 'FIRE';
    sFire.className = 'alarm';
  } else {
    fireHud.textContent = 'OK';
    fireHud.className = 'hud ok';
    sFire.textContent = 'ok';
    sFire.className = 'ok';
  }
  fireInfo.textContent = (st.reason && st.reason !== '等待首次判定')
    ? st.reason : '';
  if (st.samples) {
    fireInfo.textContent += (fireInfo.textContent ? ' · ' : '') +
      st.samples + '判';
  }
  if (st.latency_s) {
    fireInfo.textContent += (fireInfo.textContent ? ' · ' : '') +
      st.latency_s.toFixed(1) + 's';
  }
  if (!fireInfo.textContent) fireInfo.textContent = '—';
});

document.getElementById('btn-fire-collapse').addEventListener('click', () => {
  const collapsed = document.getElementById('win-fire').classList.toggle('collapsed');
  updateGridClasses();
  socket.emit('fire_control', { action: collapsed ? 'stop' : 'start' });
});

// ── 日志 ────────────────────────────────────────────────────────
const logBody = document.getElementById('log-body');
const logList = document.getElementById('log-list');
const TAG_COLORS = { viz:'#00d2ff', motor:'#ff6b6b', odom:'#7bed9f', slam:'#b088f5', lidar:'#ffa502', tf:'#5f6b8a' };
let autoScroll = true;

function makeLogEl(e) {
  const div = document.createElement('div');
  div.className = 'log-line lv-' + (e.level || 'INFO').toLowerCase();
  const time = document.createElement('span');
  time.className = 'lt'; time.textContent = e.t || '';
  const tag = document.createElement('span');
  tag.className = 'l-tag';
  tag.style.color = TAG_COLORS[e.tag] || '#8a93a8';
  tag.textContent = '[' + (e.tag || '?') + ']';
  const msg = document.createElement('span');
  msg.className = 'l-msg'; msg.textContent = e.msg || '';
  div.append(time, tag, msg);
  return div;
}
function addLog(e) {
  logList.appendChild(makeLogEl(e));
  while (logList.children.length > 800) logList.removeChild(logList.firstChild);
  document.getElementById('log-count').textContent = logList.children.length;
  if (autoScroll) logBody.scrollTop = logBody.scrollHeight;
}
socket.on('log', addLog);
socket.on('log_snapshot', arr => {
  logList.innerHTML = '';
  const frag = document.createDocumentFragment();
  for (const e of arr) frag.appendChild(makeLogEl(e));
  logList.appendChild(frag);
  document.getElementById('log-count').textContent = logList.children.length;
  if (autoScroll) logBody.scrollTop = logBody.scrollHeight;
});

logBody.addEventListener('scroll', () => {
  autoScroll = (logBody.scrollTop + logBody.clientHeight >= logBody.scrollHeight - 24);
  document.getElementById('btn-log-scroll').classList.toggle('active', autoScroll);
});
document.getElementById('btn-log-scroll').addEventListener('click', () => {
  autoScroll = true;
  document.getElementById('btn-log-scroll').classList.add('active');
  logBody.scrollTop = logBody.scrollHeight;
});
document.getElementById('btn-log-clear').addEventListener('click', () => {
  logList.innerHTML = '';
  document.getElementById('log-count').textContent = 0;
  socket.emit('log_clear');
});
document.getElementById('btn-log-collapse').addEventListener('click', () => {
  document.getElementById('win-log').classList.toggle('collapsed');
  updateGridClasses();
});

function updateGridClasses() {
  if (mappingPage || controlPage || missionPage) return;
  const collapsed = document.getElementById('win-log').classList.contains('collapsed');
  document.getElementById('workspace').style.gridTemplateRows =
    'minmax(260px,1fr) minmax(180px,.62fr) ' + (collapsed ? '42px' : '180px');
  document.getElementById('btn-log-collapse').textContent = collapsed ? '展开' : '收起';
}

// ── 驾驶 (电机控制) ──────────────────────────────────────────────
let armed = false;
let mySid = null;        // 本页面的 SocketIO sid (服务端 connect 时下发)
let driverSid = null;    // 当前驾驶者的 sid; null = 还没人被授权
// 云台当前操作者的 sid。**与 driverSid 完全独立** —— 服务端也是两套仲裁
// (_gimbal_owner / _drive_owner)。复用一个就会把云台重新绑回 armed 门上。
let gimbalOwner = null;
let driveClock = null; // Conservative server-monotonic anchor at ACK receipt.
let driveClockEpoch = 0, driveSyncPending = 0, driveSyncSerial = 0, driveClockSamples = [];
function syncDriveClock() {
  if (!controlPage || !socket.connected || document.hidden || driveSyncPending) return;
  const start = performance.now(), epoch = driveClockEpoch, token = ++driveSyncSerial;
  driveSyncPending = token;
  socket.emit('drive_sync', serverTime => {
    if (epoch !== driveClockEpoch || driveSyncPending !== token) return;
    driveSyncPending = 0;
    const received = performance.now(), rtt = received-start;
    if (!socket.connected || document.hidden || typeof serverTime !== 'number' || !Number.isFinite(serverTime) || rtt < 0 || rtt > 150) return;
    // Do not add half the RTT: the estimate must never date a command in the future.
    driveClockSamples.push({server:serverTime,local:received,rtt});
    driveClockSamples=driveClockSamples.filter(sample=>received-sample.local<=16000);
    // Keep the least delayed recent sample, rather than replacing a good clock
    // anchor with one delayed by a busy transport or image decode.
    driveClock=driveClockSamples.reduce((best,sample)=>sample.rtt<best.rtt?sample:best);
  });
  setTimeout(() => { if (driveSyncPending === token) driveSyncPending = 0; }, 1000);
}
socket.on('connect', () => { driveClockEpoch++; driveClock=null; driveClockSamples=[]; driveSyncPending=0; syncDriveClock(); });
socket.on('disconnect', () => { driveClockEpoch++; driveClock=null; driveClockSamples=[]; driveSyncPending=0; });
if (controlPage) setInterval(syncDriveClock, 2000);
const btnArm = document.getElementById('btn-arm');
const btnEstop = document.getElementById('btn-estop');
const valEl = document.getElementById('drive-val');
const hintEl = document.getElementById('drive-hint');
// ⚠️ 云台读数**必须是另一个元素**,绝不能写进 `drive-val`:
//    `web_drive_test.js` 的 read() 用 /前进 ([+-][\d.]+)\s+转向 ([+-][\d.]+)/
//    匹配 drive-val 的全文 —— 往里多写一段会让那个正则失配,**整套驾驶用例
//    一起红**,而原因看起来跟云台毫无关系。
const gimbalValEl = document.getElementById('gimbal-val');
const gimbalHintEl = document.getElementById('gimbal-hint');

// ── 摇杆 ────────────────────────────────────────────────────────
// 两种模式,**只是输入方式不同**:
//   双杆 —— 左盘只认纵轴 → 油门 l,右盘只认横轴 → 转向 a,两轴独立。
//   单杆 —— 一个 2D 盘同时给两轴(x → a,y → l)。
// UDP 契约、l/a 的符号约定、键盘、驾驶权仲裁、armed 门**一个字都没改**:
// 两种模式只是往同一个 driveTarget() 里填 l/a 的方式不同,出口完全一样。
//
// 每个盘都带 vx/vy 两个分量(单轴盘的另一轴恒 0),于是 driveTarget() 里
// 一个式子就能同时处理两种盘,不必按模式分支。
//
// ⚠️ 单轴盘**不能**做 hypot 归一化 —— 那会把手指在盘里的横向偏移悄悄折算进
// 纵向指令。只有单杆那个真的 2D 盘才归一化(见 padUpdate 里的「圆门」)。
const PADS = {
  thr:   { el: document.getElementById('joy-thr'),
           knob: document.getElementById('knob-thr'),   axes: 'y',  vx: 0, vy: 0, active: false, pid: null },
  steer: { el: document.getElementById('joy-steer'),
           knob: document.getElementById('knob-steer'), axes: 'x',  vx: 0, vy: 0, active: false, pid: null },
  stick: { el: document.getElementById('joy-2d'),
           knob: document.getElementById('knob-2d'),    axes: 'xy', vx: 0, vy: 0, active: false, pid: null },
  // 云台盘(二维云台,走串口)。
  // ⚠️ **刻意放进 padList**:它需要指针注册、setPointerCapture、releaseAll 清
  //    active、padsReset 归零 —— 这些都在 padList 的循环里,进 padList 才免费拿到。
  // ⚠️ 代价是**两处必须显式排除**,任漏一处「云台不受 armed 管」就失效:
  //      ① 不进 MODE_PADS ⇒ driveTarget() 结构上够不到它(这一点天然成立);
  //      ② driveGate() 的循环必须 `if (p.isGimbal) continue;` —— 见那边的长注释。
  gimbal: { el: document.getElementById('joy-gimbal'),
            knob: document.getElementById('knob-gimbal'), axes: 'xy', vx: 0, vy: 0,
            active: false, pid: null, isGimbal: true },
  // 双杆模式的两个单轴云台盘(水平 pan / 垂直 tilt)。和上面那个 2D 盘一样带
  // isGimbal:true,保证 driveGate 跳过它们、padList 循环里免费拿到指针注册/归零。
  gimbalPan:  { el: document.getElementById('joy-gimbal-pan'),
                knob: document.getElementById('knob-gimbal-pan'),  axes: 'x', vx: 0, vy: 0,
                active: false, pid: null, isGimbal: true },
  gimbalTilt: { el: document.getElementById('joy-gimbal-tilt'),
                knob: document.getElementById('knob-gimbal-tilt'), axes: 'y', vx: 0, vy: 0,
                active: false, pid: null, isGimbal: true },
};
const padList = [PADS.thr, PADS.steer, PADS.stick, PADS.gimbal, PADS.gimbalPan, PADS.gimbalTilt];

// 每种模式**读哪几个盘**。互斥地读是有意的:被隐藏的那个盘万一残留了 active
// (隐藏一个正持着 pointer capture 的元素时,pointerup 不保证还会送到它),
// 互斥能保证它绝不会偷偷继续出力。
const MODE_PADS = { dual: [PADS.thr, PADS.steer], single: [PADS.stick] };
let joyMode = 'dual';

// 云台自己的模式映射,与驾驶的 MODE_PADS 完全独立:双杆 = pan + tilt 两个单轴盘,
// 单杆 = 一个 2D 盘。默认单杆(保持云台一直是 2D 盘的历史行为)。
// ⚠️ 别动上面的 MODE_PADS —— web_drive_test.js 有精确正则钉死它。
const GIMBAL_MODE_PADS = { dual: [PADS.gimbalPan, PADS.gimbalTilt], single: [PADS.gimbal] };
let gimbalMode = 'single';
let gimbalWasMoving = false;

// 同一份页面开在两台设备上时,双边都会 20Hz 发 /cmd_vel,没握摇杆的那台
// 持续灌 0 → 真值被切断 → 电机抖。服务端只认驾驶者,这里同步禁掉非驾驶者
// 的发送与操作,免得用户对着没反应的摇杆使劲。
function isDriver() {
  return !armed || driverSid === null || driverSid === mySid;
}

function sendDriveTarget(linear, angular, takeover=false, periodic=false) {
  const payload = {action:'vel', linear, angular};
  if (takeover) payload.takeover = true;
  if (controlPage) {
    if (!socket.connected) return;
    const moving = linear !== 0 || angular !== 0;
    if (moving) {
      if (driveClock === null) return; // Wait for bounded clock alignment before motion.
      const elapsed = performance.now()-driveClock.local;
      if (elapsed < 0 || elapsed > 30000) {
        driveClock = null;
        socket.emit('motor_control', {action:'arm', armed:false});
        return; // Clock failure requires a new explicit arm; never resumes itself.
      }
      payload.sent_monotonic = driveClock.server+elapsed/1000;
    }
    // All periodic state (including neutral) is replaceable. Explicit release/stop
    // remains reliable and untimestamped: an old zero can never authorize motion.
    if (periodic && socket.volatile) {
      socket.volatile.emit('motor_control', payload);
      return;
    }
  }
  socket.emit('motor_control', payload);
}

function driveGate() {
  const ok = isDriver();
  // 所有盘都刷(含当前隐藏的):这样切换模式时,新露出来的那个盘**已经**带着
  // 正确的 opacity/pointerEvents,不会先闪一下可点状态。
  for (const p of padList) {
    // ⚠️⚠️ **云台盘必须跳过。** 这个门的判据是**电机驾驶权**,与云台无关。
    //   不跳过的三个后果,每一个都是静默的:
    //     ① 别人点了「启用电机」后 isDriver() 为假 ⇒ 云台盘被 pointerEvents:'none'
    //        锁死 ⇒ 从第二台设备**完全无法控制云台**,除非去点「启用电机」抢权。
    //        **这直接违反「云台不受 armed 管」这条设计决定。**
    //     ② 提示文案跟着说谎:下面写的是「已被其它设备接管 —— 点「启用电机」抢回」
    //        —— **为了拿回相机云台,UI 让你去给电机上电**。
    //     ③ 反方向同样坏:`isDriver()` 是 `!armed || ...`,armed=false 时
    //        `!armed` 短路为真 ⇒ 它对**所有**客户端返回 true ⇒ 两台设备都能同时
    //        灌云台,服务端那套 _gimbal_owner 仲裁在这层等于零防护。
    //   ⚠️ 这个 `!armed ||` 的形状,和 slam_viz._on_motor_control 里 2026-09-15
    //      已修掉的那个洞**一模一样** —— 同一个坑,这是第二次。
    if (p.isGimbal) continue;
    p.el.style.opacity = ok ? '1' : '.4';
    p.el.style.pointerEvents = ok ? 'auto' : 'none';
  }
  hintEl.textContent = ok
    ? (joyMode === 'single' ? '单杆推即走,斜推同时转向,或 WASD/方向键'
                            : '左杆油门 右杆转向,或 WASD/方向键')
    : '已被其它设备接管 —— 点「启用电机」抢回';
}

// ── 云台的门:只认**连接** ──────────────────────────────────────
// ⚠️ 判据里**没有 armed、也没有 isDriver()**,这是刻意的:转相机 ≠ 驱动轮子。
//    服务端的云台操作权(_gimbal_owner)是另一套仲裁,与电机驾驶权无关。
// ⚠️ 但"能不能动"和"我是不是当前操作者"是两回事,提示文案必须把后者说清楚 ——
//    否则「我推了云台却没反应(其实被别人抢走了)」会被当成云台坏了。
function gimbalGate() {
  const ok = !!socket.connected;
  // 三个云台盘(单杆 2D 盘 + 双杆两个单轴盘)都要刷,不是只有 PADS.gimbal 一个 ——
  // 否则切到双杆后那两个新盘会一直停在初始的 opacity/pointerEvents。
  for (const p of padList) {
    if (!p.isGimbal) continue;
    p.el.style.opacity = ok ? '1' : '.4';
    p.el.style.pointerEvents = ok ? 'auto' : 'none';
  }
  const other = ok && gimbalOwner !== null && mySid !== null && gimbalOwner !== mySid;
  gimbalHintEl.textContent = !ok ? '未连接 —— 云台指令发不出去'
    : other ? '云台正被另一台设备操作 —— 你一推杆就抢过来'
            : '云台:不必「启用电机」';
}

socket.on('you', m => { mySid = m.sid; driveGate(); gimbalGate(); });

// 服务端只在**操作者变化时**推这个事件(不随包推 —— 每包推会给 socketio.emit
// 开口消防水管,见文件开头记的自持 emit 循环把客户端定时器饿到十几 Hz 那次)。
socket.on('gimbal_state', st => {
  gimbalOwner = st.owner || null;
  gimbalGate();
});

socket.on('motor_state', st => {
  armed = !!st.armed;
  driverSid = st.driver || null;
  btnArm.classList.toggle('armed', armed);
  btnArm.textContent = armed ? '电机已启用' : '启用电机';
  driveGate();
});

function driveUi(connected) {
  btnArm.disabled = !connected;
  btnEstop.disabled = !connected;
  if (!connected) {
    armed = false;
    driverSid = null;
    btnArm.classList.remove('armed');
    btnArm.textContent = '启用电机';
  }
  driveGate();
  gimbalGate();
}
socket.on('connect', () => driveUi(true));
socket.on('disconnect', () => driveUi(false));

btnArm.addEventListener('click', () => {
  socket.emit('motor_control', { action: 'arm', armed: !armed });
});
btnEstop.addEventListener('click', () => {
  // ⚠️ 服务端在 estop 里会把云台**停住**(发零速,见 _on_motor_control)。这里同步把
  //    **盘和读数**清零 —— 否则界面还显示着「dx +50」而云台已经停了,那是个说谎的读数。
  //    服务端那一步管实际动作,这一步管界面,两边都要做。
  padReset(PADS.gimbal);
  socket.emit('motor_control', { action: 'estop' });
});

// 折叠会隐藏持有指针的盘:先清零并补发停止,再隐藏。急停始终可见。
document.getElementById('btn-drive-collapse').addEventListener('click', () => {
  const panel = document.getElementById('drive');
  const collapsed = !panel.classList.contains('collapsed');
  if (collapsed) releaseAll();
  panel.classList.toggle('collapsed', collapsed);
  const button = document.getElementById('btn-drive-collapse');
  button.textContent = collapsed ? '展开' : '收起';
  button.setAttribute('aria-expanded', String(!collapsed));
});

// ── 建图按钮 (地图窗口标题栏) ───────────────────────────────
// 状态是**服务端按 /scan 与 /map 的实际数据流**算出来的,不是「我按过了」。
// 所以别人从 ssh 起的管线,这里照样显示成建图中 —— 按钮不说谎是设计目标。
const btnMap = document.getElementById('btn-map');
let mapStatus = 'off';     // off | lidar_only | mapping  (服务端只发这三个)
let mapBusy  = false;      // 服务端有 start/stop 在路上
let mapAction = null;
let mapManaged = false;

// 三态 => 文案 / class / 鼠标悬停说明。文案说的是**当前状态**;
// 点下去做什么由 mapClick() 按状态决定。
const MAP_UI = {
  off:        { text: '启动建图', cls: null,
                tip: '点一下:拉起 雷达 + 激光里程计 + 静态TF + slam_toolbox' },
  lidar_only: { text: 'SLAM 未起 ⟳', cls: 'partial',
                tip: '雷达在出数据,但地图没有在更新 —— 雷达之后的某一环(里程计/TF/slam_toolbox)没起来或已崩。再点一次会重启整条链路' },
  localization: { text: '定位模式', cls: null, tip: '已加载保存的地图；请先停止定位再建图' },
  mapping:    { text: '建图中', cls: 'on',
                tip: '地图正在更新(雷达+里程计+TF+SLAM 全链路都活着)。点一下停止 —— ⚠️ 地图会丢,本项目暂不自动保存' },
};

function mapPaint() {
  if (mapBusy) {
    btnMap.disabled = true;
    btnMap.classList.remove('on', 'partial');
    btnMap.textContent = (mapAction === 'stop') ? '停止中…' : '启动中…';
    return;
  }
  const ui = MAP_UI[mapStatus] || MAP_UI.off;
  btnMap.disabled = !socket.connected || mapStatus === 'localization' || mapManaged;
  btnMap.classList.toggle('on',      ui.cls === 'on');
  btnMap.classList.toggle('partial', ui.cls === 'partial');
  btnMap.textContent = ui.text;
  btnMap.title = mapManaged ? '自动建图管线管理中，请在地图与导航中结束建图或停止管线' : ui.tip;
}

socket.on('mapping_state', st => {
  mapStatus = st.status || 'off';
  mapBusy   = !!st.busy;
  mapAction = st.action || null;
  mapManaged = !!st.managed;
  mapPaint();
});

btnMap.addEventListener('click', () => {
  if (mapBusy || mapStatus === 'localization' || mapManaged) return;
  if (mapStatus === 'mapping') {
    // 没有自动保存(slam_toolbox Humble 也没有 reset 服务),停了就没了 ⇒ 确认一次。
    if (!confirm('停止建图?当前地图会丢掉 —— 没有自动保存。')) return;
    socket.emit('mapping_control', { action: 'stop' });
  } else {
    socket.emit('mapping_control', { action: 'start' });
  }
});

// 连接态单独一个函数,不往 driveUi 里塞驾控以外的东西
function mapUi(connected) {
  if (!connected) { mapStatus = 'off'; mapBusy = false; mapAction = null; }
  mapPaint();
}
socket.on('connect',    () => mapUi(true));
socket.on('disconnect', () => mapUi(false));

// 键盘 WASD / 方向键 (页面聚焦时; 松开即回零)
const keys = {};
function editingText(target) {
  return target && (target.isContentEditable || ['INPUT','TEXTAREA','SELECT'].includes(target.tagName));
}
document.addEventListener('focusin', e => { if (editingText(e.target)) releaseAll(); });
document.addEventListener('keydown', e => {
  if (missionPage || editingText(e.target)) return;
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key)) e.preventDefault();
  if (e.repeat) return;
  keys[e.key] = true;
});
function sendReleasedDrive() {
  if (missionPage || !armed || !isDriver()) return;
  const target = driveTarget();
  sendDriveTarget(target.l, target.a);
}
document.addEventListener('keyup', e => {
  const wasPressed = keys[e.key] || (e.key.length === 1 && (keys[e.key.toLowerCase()] || keys[e.key.toUpperCase()]));
  keys[e.key] = false;
  // 2026-10-25: Shift 按住时 e.key 是大写，先松 Shift 再松 W 会留下 keys['W']=true
  // 卡在云台上；两个都清。
  if (e.key.length === 1) { keys[e.key.toLowerCase()] = false; keys[e.key.toUpperCase()] = false; }
  if (wasPressed && e.key !== 'Shift') sendReleasedDrive();
});

// 虚拟摇杆: 上=前进 下=后退,左=左转 右=右转。
// 松手回中(弹簧回中),与真车遥控器一致 —— 也是「死人手柄」在触屏上的自然形态。
function padUpdate(p, clientX, clientY) {
  const r = p.el.getBoundingClientRect();
  const R = p.el.clientWidth / 2;
  let vx = (clientX - (r.left + R)) / R;
  let vy = (clientY - (r.top + R)) / R;
  // 单轴盘把死轴摁成 0 —— 手指在盘里的横向偏移不该折进纵向指令。
  // ⚠️ 必须在下面归一化**之前**摁:摁完另一轴恒 0,模长就等于该轴自己的绝对值,
  //    于是「只钳模长」对单轴盘是个恒等变换(见下),横向偏移依然折不进来。
  if (p.axes === 'y') vx = 0;
  if (p.axes === 'x') vy = 0;
  // ★ 只钳**模长**,方向原样保留。
  // ⚠️ 顺序是承重的,别退回「先按轴各钳一次、再归一化」:那样「某一轴超出 1」的整片
  //    区域会**塌缩成同一个值**(指令冻死)。而手指滑出盘子时 setPointerCapture 会
  //    把 move 继续送进来,于是「推满前进再往右滑」只要滑过盘右边界就不再响应 ——
  //    现场表现就是「单杆转向有时没反应」。实测(指针在盘心外一个半径以上):
  //      先钳各轴 (1.5,-1.0) → (1,-1) → 0.707/-0.707,冻住;
  //      只钳模长 (1.5,-1.0) → 0.832/-0.555,方向仍在跟手指。
  //    顺带记一笔:(1.5,-1.5) 这类**按比例**推到盘外的位置,两种写法结果相同,
  //    所以老用例「拖出盘外仍钳在圆门」依然成立 —— 差异只在非比例的位置上。
  // 圆门本身照旧:推到 45° 角是 0.707/0.707 而不是 1/1。代价是单杆没法
  // 「满油门 + 同时满转向」,要那个就用双杆(那两轴互相独立)。
  // 单轴盘:另一轴恒 0 ⇒ m == |该轴|,除以 m 恰好得 ±1,与原来的按轴钳位**逐点等价**。
  // (旧注释说单轴盘「模长 = |该轴|,已经 ≤1」是**循环论证** —— "已经 ≤1" 只是因为
  //  前面刚按轴钳过。那句话正是让人以为这里可以分支处理的源头。)
  const m = Math.hypot(vx, vy);
  if (m > 1) { vx /= m; vy /= m; }
  p.vx = vx; p.vy = vy;
  p.knob.style.transform = `translate(${vx * R}px, ${vy * R}px)`;
}
function padReset(p) {
  p.vx = 0; p.vy = 0;
  p.knob.style.transform = 'translate(0,0)';
}
function padsReset() { padList.forEach(padReset); }

for (const p of padList) {
  p.el.addEventListener('pointerdown', e => {
    p.active = true;
    // 记下是哪根手指按的。blur 之后要靠它认出「这根手指其实还按着」(见 releaseAll
    // 与下面 pointermove 的判据)。
    p.pid = e.pointerId;
    try { p.el.setPointerCapture(e.pointerId); } catch (_) {}
    padUpdate(p, e.clientX, e.clientY);
  });
  // ⚠️ 判据不能只是 `p.active`。blur 会把 active 清掉(车必须停),但那时**手指往往
  //    还按在盘上**、pointerup 根本不会来 ⇒ 只认 active 就等于「失焦之后拖动全被丢弃,
  //    必须松手重按」,现场表现同样是「推摇杆没反应」。
  //    所以:还按着的那根手指(pid 匹配)允许把自己**重新激活**,取的是手指**当前
  //    位置**而不是残留值 —— blur 期间车确实是停的,恢复要靠用户真的又动了那根手指。
  //    松手时 pointerup 已把 pid 清成 null ⇒ 「松手后再拖」照旧无效,语义没变。
  p.el.addEventListener('pointermove', e => {
    if (p.active || p.pid === e.pointerId) {
      p.active = true;
      padUpdate(p, e.clientX, e.clientY);
    }
  });
  const release = () => {
    p.active = false;p.pid = null;padReset(p);
    if (!p.isGimbal) sendReleasedDrive();
  };
  p.el.addEventListener('pointerup', release);
  p.el.addEventListener('pointercancel', release);
}

// 失焦 / 切标签页时 keyup 与 pointerup 都不会触发,按键和摇杆状态会残留 →
// 机器人一路跑不停。强制清空并立刻补发一次零速指令。
// (注意区分:积分残留也会「停不下来」,那条在 motor_driver._control_tick 修)
// keepPointer:失焦/切标签页时传 true,**保留每根手指的 pid**。
// 那时手指往往还按在盘上、pointerup 不会来 —— 留着 pid,下一次 pointermove 才能让
// 这根手指接着用(否则就得松手重按,现象就是「推摇杆没反应」)。安全侧不受影响:
// 车已经在下面被立刻归零并补发零速了,恢复要靠用户真的又动了那根手指。
// 切模式则必须清掉:那个盘正在被隐藏,旧 pid 不该再把它激活。
function releaseAll(keepPointer) {
  for (const k in keys) keys[k] = false;
  for (const p of padList) {
    p.active = false;
    if (!keepPointer) p.pid = null;
  }
  padsReset();
  if (armed && isDriver()) {
    sendDriveTarget(0, 0, true);
  }
  // ⚠️⚠️ 云台零速帧 **无条件发**,两处刻意与上面那条不同:
  //   ① **不判 armed / isDriver()**。云台的典型工况恰恰是「没启用电机、只在转
  //      相机」—— 照抄上面那个条件的话,那种工况下零速帧根本不发,云台会**一直按
  //      最后那个速度转下去**(速度是锁存的!),而用户明明已经松手了。
  //   ② **在这里命令式直接 emit,不交给定时器**。visibilitychange 那条路是
  //      「先 releaseAll(true),**然后**才 setDriveRunning(false)」——
  //      靠定时器发的那个零速帧,在那一刻定时器已经被停,**永远出不去**。
  //      (上面那条 motor_control 也是出于同样的理由才写成命令式直接 emit。)
  // ⚠️ 措辞是「零速」不是「回中」:`0xAA` 是速度帧,`(0,0)` = 停,不是回中位。
  // ⚠️ 与 deadman 那条**现在的方向一致**:松手/失焦/切标签页是**操作者的明确动作**,
  //    走这条命令式路径;链路丢失走 gimbal_proc 的 deadman —— **那边也是发零速**,
  //    不是停发(2026-09-27 修掉的那个错)。
  // ⚠️ 未连接时 socket.emit 是空操作,不必额外判连接。
  // ⚠️ 不是操作者时服务端会把这包零丢掉(它只认操作者的零)—— 那正是我们要的:
  //    别人松手不该把云台拽停。
  if (!mappingPage && !missionPage) socket.emit('gimbal_control', { action: 'vel', dx: 0, dy: 0 });
}
window.addEventListener('blur', () => releaseAll(true));
window.addEventListener('pagehide', () => { releaseAll(); setDriveRunning(false); });
window.addEventListener('pageshow', () => setDriveRunning(!document.hidden));
document.addEventListener('visibilitychange', () => {
  // 不可见时**两件事都要做**:① 清掉残留的按键/摇杆并补发一次零速;② 停掉
  // 发送定时器 —— 光做 ① 是不够的,被钳到 1Hz 的定时器会继续发零速包,而
  // 1Hz 的包率正好落在 deadman 阈值以下(见下面 setDriveRunning 处的长注释)。
  if (document.hidden) releaseAll(true);
  setDriveRunning(!document.hidden);
});

// ── 摇杆模式切换 (双杆 / 单杆) ──────────────────────────────────
const driveEl = document.getElementById('drive');
const modeBtns = Array.from(document.querySelectorAll('#joy-mode button'));

function setJoyMode(m) {
  // ★ 顺序不能反:先把正在按的盘清干净,**再**把它藏起来。
  //   隐藏一个正持着 pointer capture 的元素时,pointerup 不保证还会送到它 ——
  //   于是 p.active 残留 → 盘看不见了但 l/a 还在按最后那个值发 → 车一直跑。
  //   releaseAll() 正是干这个的,而且会立刻补发一次零速。
  releaseAll();
  joyMode = (m === 'single') ? 'single' : 'dual';
  driveEl.classList.toggle('mode-single', joyMode === 'single');
  for (const b of modeBtns) b.classList.toggle('on', b.dataset.mode === joyMode);
  try { localStorage.setItem('joyMode', joyMode); } catch (_) {}
  driveGate();
}
for (const b of modeBtns) {
  b.addEventListener('click', () => setJoyMode(b.dataset.mode));
}
// 恢复上次选的模式。localStorage 在隐私模式/禁用 cookie 时会抛,整段包起来。
// ⚠️ 必须在 `let joyMode` 之后、且此时 padList/releaseAll 都已就绪。
try {
  if (localStorage.getItem('joyMode') === 'single') setJoyMode('single');
} catch (_) {}
// 初始化按钮高亮(没走到上面那行时也要把「双杆」点亮)。
for (const b of modeBtns) b.classList.toggle('on', b.dataset.mode === joyMode);

// Manual throttle range only; each page starts low, never restores high speed.
const LOW_GEAR_MAX = 0.30;
let speedGear = 'low';
function setSpeedGear(gear) {
  const next = gear === 'high' ? 'high' : 'low';
  if (next !== speedGear) releaseAll();
  speedGear = next;
  for (const [id, value] of [['gear-low','low'], ['gear-high','high']]) {
    const button = document.getElementById(id);
    button.classList.toggle('on', speedGear === value);
    button.setAttribute('aria-pressed', String(speedGear === value));
  }
  document.getElementById('gear-hint').textContent = speedGear === 'low'
    ? `低速 · 油门上限 ${Math.round(LOW_GEAR_MAX * 100)}%` : '高速 · 原有油门范围 · 注意周围环境';
}
document.getElementById('gear-low').addEventListener('click', () => setSpeedGear('low'));
document.getElementById('gear-high').addEventListener('click', () => setSpeedGear('high'));
setSpeedGear('low');

// ── 云台摇杆模式切换 (双杆 / 单杆) ──────────────────────────────
// 和 setJoyMode 同一套顺序:先清正在按的盘、再切 class(隐藏一个正持着 pointer
// capture 的元素时 pointerup 不保证送到它,盘藏了但残留 active 会继续发)。
// ⚠️ **不能**直接调 releaseAll():那会连驾驶盘一起清、并补发一包电机零速,
//    切云台模式不该打断正在驾驶的手。这里只清 isGimbal 盘、只补发云台零速。
const gimbalEl = document.getElementById('gimbal');
const gimbalModeBtns = Array.from(document.querySelectorAll('#gimbal-mode button'));

function setGimbalMode(m) {
  for (const p of padList) {
    if (!p.isGimbal) continue;
    p.active = false;
    p.pid = null;         // 切模式:被隐藏的盘,旧 pid 不该再把它激活
    padReset(p);
  }
  gimbalMode = (m === 'single') ? 'single' : 'dual';
  gimbalEl.classList.toggle('mode-single', gimbalMode === 'single');
  for (const b of gimbalModeBtns) b.classList.toggle('on', b.dataset.mode === gimbalMode);
  try { localStorage.setItem('gimbalMode', gimbalMode); } catch (_) {}
  // 立刻补发云台零速(与 releaseAll 里那条同样的理由:命令式直接 emit,别交给定时器)。
  if (!mappingPage && !missionPage) socket.emit('gimbal_control', { action: 'vel', dx: 0, dy: 0 });
  gimbalGate();
}
for (const b of gimbalModeBtns) {
  b.addEventListener('click', () => setGimbalMode(b.dataset.mode));
}
// 恢复上次选的模式(默认单杆;localStorage 在隐私模式/禁用 cookie 时会抛,整段包起来)。
try {
  if (localStorage.getItem('gimbalMode') === 'dual') setGimbalMode('dual');
} catch (_) {}
// 初始化按钮高亮(没走到上面那行时也要把「单杆」点亮)。
for (const b of gimbalModeBtns) b.classList.toggle('on', b.dataset.mode === gimbalMode);

// 当前目标速度:摇杆 + 键盘合并。松键/松杆即回零("死人手柄")。
function driveTarget() {
  let l = 0, a = 0;
  // 取负:往上/往右推时 clientY/clientX 是增大的,而「上=前进、右=右转」。
  // 与键盘那组(a 键 a+=1 = 左转)是同一套符号约定,别单独翻其中一个。
  // 只读当前模式的那几个盘(见 MODE_PADS);单轴盘的死轴恒 0,所以一个式子
  // 对两种盘都成立,不必按模式分支。
  for (const p of MODE_PADS[joyMode]) {
    if (!p.active) continue;
    l -= p.vy;
    a -= p.vx;
  }
  // 2026-10-25: Shift 按住时键盘让给云台（Shift+WASD），驾驶只认摇杆/方向键；
  // 否则两套指令同时生效，调相机会把车也带着走。
  if (!keys['Shift']) {
    if (keys['w'] || keys['W'] || keys['ArrowUp']) l += 1;
    if (keys['s'] || keys['S'] || keys['ArrowDown']) l -= 1;
    if (keys['a'] || keys['A'] || keys['ArrowLeft']) a += 1;
    if (keys['d'] || keys['D'] || keys['ArrowRight']) a -= 1;
  }
  return {
    l: Math.max(-1, Math.min(1, l)) * (speedGear === 'low' ? LOW_GEAR_MAX : 1),
    a: Math.max(-1, Math.min(1, a)),
  };
}
function fmtSigned(v) { return (v < 0 ? '-' : '+') + Math.abs(v).toFixed(2); }
// 云台按**整数**显示 —— 它就是线上那 6 个字节里的值,写成 +50.00 会让人以为
// 中间还有小数精度。显示的值 = 真正会编码进帧的值,一个比特都不多。
function fmtSignedInt(v) { return (v < 0 ? '-' : '+') + Math.abs(v); }

// 周期发送摇杆值 (60Hz) — "死人手柄": 松键/松杆即回零。
// 非驾驶者不发(服务端也会丢),但读数照常刷新,方便先看清数值再抢控制权。
//
// 60Hz 而不是原来的 20Hz:直连通道下 duty 严格等于摇杆比值,于是「摇杆动 →
// duty 变」之间就只剩这个间隔。50ms 会直接变成手感上的延迟;16ms 约等于屏幕
// 刷新率,已经感知不到。UDP 是「最新值」语义,多发不会积压(见 pwm_proc.py)。
//
// ⚠️⚠️ **页面不可见时必须停掉这个定时器**。原因不是省电:
//   浏览器会把不可见标签页的 setInterval 钳到约 1Hz —— 这是浏览器的行为,
//   绕不过去(Web Worker / 音频 hack 都不可靠)。1Hz = 包间隔约 400ms,而
//   pwm_proc 的 deadman 是 300ms,于是每一包都被判成一次「恢复」、300ms 后
//   又一次「跳闸」。电机和舵机走**同一个门**,舵机就跟着 park(完全不发脉冲、
//   松掉)↔ 全速转回目标角,**来回抖** —— 2026-09-15 实测到 61 次严格交替的
//   「中断/恢复」,用户报的现象就是「启用电机后舵机老是来回抖」。
//   pwm_proc 那侧现在加了滞回(见 motor_control.Deadman),不会再抖了;但让
//   这个包率继续发本身就是错的 —— 页面既然不可见,就没人能看住这台车,
//   「暂停发送」比「发一堆把链路拖到阈值以下的包」诚实,读数也照实说「已暂停」。
//   ⚠️ blur(窗口失焦但仍可见)**不**在此列:那种情况不被节流,不用停。
const DRIVE_PERIOD_MS = 16;
let driveTimer = null;

function driveTick() {
  // Mission pages use Nav2's owned control loop and separate heartbeat.
  // Hidden manual controls must not flood SocketIO/DDS with 60Hz zero commands.
  if (missionPage) return;
  const t = driveTarget();
  valEl.textContent = '前进 ' + fmtSigned(t.l) + '　转向 ' + fmtSigned(t.a);
  if (!armed || !isDriver()) return;
  sendDriveTarget(t.l, t.a, false, true);
}

// ── 云台 ────────────────────────────────────────────────────────
// ⚠️⚠️ 云台**必须有自己的 tick 和自己的门**,绝不能把 emit 追加到 driveTick 末尾:
//    上面 `if (!armed || !isDriver()) return;` 是**早退** —— 追加在它后面就等于
//    「云台只在电机已启用、且我是驾驶者时才动」,又一次违反「云台不受 armed 管」。
//
// 单位就是**线上单位**(±100):前端 → slam_viz → UDP → 串口帧全程一致,中间不做
// -1..1 的换算。少一次换算就少一处「界面说 A、线上是 B」的可能。
// ⚠️ `dx/dy` 是**速度**(不是角度、不是位置):`(0,0)` = 停,不是回中。
//    ⚠️ 加了斜坡之后,界面这个读数 = **目标**;`gimbal_proc` 发出去的会短暂小于它
//    (`gimbal_slew_per_s`,0→满量程约 0.4s)。要看**已发值**得看进程的状态行。
// ⚠️ ±100 是**本实现选定的**满量程,不是协议上限(int16 全域,|value| ≥ 250 才饱和)。
//
// 符号约定(**物理方向不在这里定**,由板上的 gimbal_invert_x / gimbal_invert_y 定,
// 那是唯一该改的地方 —— 和 motor_params.yaml 的 servo_invert 同一性质):
//     推上 = dy 正      推右 = dx 正
// 与驾驶摇杆的「上=前进、右=右转」是同一套直觉,别单独翻其中一个。
const GIMBAL_SCALE = 100;
function gimbalTick() {
  if (mappingPage || missionPage) return;
  // 只读当前云台模式的那几个盘(GIMBAL_MODE_PADS);单轴盘的死轴恒 0,
  // 所以一个式子对两种盘都成立,不必按模式分支(与 driveTarget 同理)。
  let dx = 0, dy = 0;
  for (const p of GIMBAL_MODE_PADS[gimbalMode]) {
    if (!p.active) continue;
    dx += Math.round(p.vx * GIMBAL_SCALE);
    dy += Math.round(-p.vy * GIMBAL_SCALE);
  }
  // 2026-10-25: Shift+WASD 叠加到云台（W 上 / S 下 / A 左 / D 右）。
  // 大写字母是 Shift 按住时 e.key 的真实值；CapsLock 的 'W' 没有 Shift，不触发。
  if (keys['Shift']) {
    if (keys['w'] || keys['W']) dy += GIMBAL_SCALE;
    if (keys['s'] || keys['S']) dy -= GIMBAL_SCALE;
    if (keys['a'] || keys['A']) dx -= GIMBAL_SCALE;
    if (keys['d'] || keys['D']) dx += GIMBAL_SCALE;
  }
  dx = Math.max(-GIMBAL_SCALE, Math.min(GIMBAL_SCALE, dx));
  dy = Math.max(-GIMBAL_SCALE, Math.min(GIMBAL_SCALE, dy));
  gimbalValEl.textContent = 'dx ' + fmtSignedInt(dx) + '　dy ' + fmtSignedInt(dy);
  // 非驾驶者的读数照常刷新(方便先看清数值再动),但没连接就别发 —— 发了也没人收,
  // 只会让 read() 之类的计数出现幻影包。
  if (!socket.connected) return;
  // Idle non-operators cannot affect the gimbal. Avoid 60 discarded requests/s
  // on the camera-first page; explicit release/blur still sends zero immediately.
  const moving = dx !== 0 || dy !== 0;
  if (controlPage && !moving && !gimbalWasMoving && gimbalOwner !== mySid) return;
  gimbalWasMoving = moving;
  socket.emit('gimbal_control', { action: 'vel', dx: dx, dy: dy });
}

// ⚠️⚠️ **一个 16ms 定时器,里面调两个 tick。** 绝不改成两个 `setInterval(..., 16)`:
//    `web_drive_test.js` 的 `liveTimer(ms)` 用的是 `timers.find(...)`(**不是
//    filter**)—— 两个同周期定时器时它会拿到**其中一个**,于是测试台重放的是云台
//    那个、`valEl` 没被刷新、`read()` 的正则匹配到**上一帧的陈旧文本** ⇒
//    **测试照样全绿,而驾驶链路已经断了**。那正是这个测试台存在的意义的反面。
//    共用一个定时器之后 `liveTimer(16)` / `read()` / `setDriveRunning` 全不用改,
//    第 10 节整段继续有效。
function tickAll() { driveTick(); gimbalTick(); }

function setDriveRunning(on) {
  if (on) {
    if (driveTimer === null) driveTimer = setInterval(tickAll, DRIVE_PERIOD_MS);
  } else if (driveTimer !== null) {
    clearInterval(driveTimer);
    driveTimer = null;
    // 定时器停了读数就不再刷新,留着最后一帧的「前进 +0.80」是**假**的 ——
    // 那个值此刻既没在发、车也不认。宁可写清楚它已经暂停。
    // ⚠️ 云台读数**同理**,不能只写一个:留着「dx +50」而网页其实已经不再发,
    //    是同一个谎。两个都要说清楚。
    //    ⚠️ 此刻云台并不一定停了 —— 进程的 deadman(500ms)之后才会把目标压成零速。
    //       所以这句说的是「**本页不再发**」,不是「云台已停」。别把措辞写强。
    valEl.textContent = '已暂停(页面不可见)';
    gimbalValEl.textContent = '已暂停(页面不可见)';
  }
}
// 判据用 !document.hidden 而不是直接 true:页面**一开始就在后台标签页**里
// 打开时不会触发 visibilitychange 事件,直接 true 等于让它带着 1Hz 的包率起跑
// —— 恰好是这个 bug 本身。回到前台时下面那个监听器会把它重新拉起来。
setDriveRunning(!document.hidden);

// ── 时钟 ────────────────────────────────────────────────────────
setInterval(() => {
  const d = new Date();
  document.getElementById('s-clock').textContent = d.toTimeString().slice(0, 8);
}, 1000);

// ── 坐标换算 ────────────────────────────────────────────────────
function w2s(wx, wy) {
  return {
    x: canvas.width/2  + (wx - vCx) * vScale,
    y: canvas.height/2 - (wy - vCy) * vScale,
  };
}
function s2w(sx, sy) {
  return {
    x: vCx + (sx - canvas.width/2)  / vScale,
    y: vCy - (sy - canvas.height/2) / vScale,
  };
}
function scanColor(dist) {
  if (dist < 1.0) return '#00d2ff';
  if (dist < 3.0) return '#7bed9f';
  return '#ffa502';
}

// ── 渲染 ────────────────────────────────────────────────────────
function drawGrid() {
  const minW = s2w(0, canvas.height);
  const maxW = s2w(canvas.width, 0);
  const x0 = Math.floor(minW.x), x1 = Math.ceil(maxW.x);
  const y0 = Math.floor(minW.y), y1 = Math.ceil(maxW.y);
  ctx.strokeStyle = '#161a2a';
  ctx.lineWidth = 1;
  ctx.setLineDash([4, 8]);
  for (let x = x0; x <= x1; x++) {
    const p = w2s(x, 0);
    ctx.beginPath(); ctx.moveTo(p.x, 0); ctx.lineTo(p.x, canvas.height); ctx.stroke();
  }
  for (let y = y0; y <= y1; y++) {
    const p = w2s(0, y);
    ctx.beginPath(); ctx.moveTo(0, p.y); ctx.lineTo(canvas.width, p.y); ctx.stroke();
  }
  ctx.setLineDash([]);
  const o = w2s(0, 0);
  ctx.strokeStyle = '#3a3d52';
  ctx.beginPath();
  ctx.moveTo(o.x - 12, o.y); ctx.lineTo(o.x + 12, o.y);
  ctx.moveTo(o.x, o.y - 12); ctx.lineTo(o.x, o.y + 12);
  ctx.stroke();
}

function drawMap() {
  if (!mapImg || !mapMeta) return;
  const mw = mapMeta.width  * mapMeta.resolution;
  const mh = mapMeta.height * mapMeta.resolution;
  const tl = w2s(mapOriginX, mapOriginY + mh);
  const br = w2s(mapOriginX + mw, mapOriginY);
  ctx.imageSmoothingEnabled = false;
  ctx.drawImage(mapImg, tl.x, tl.y, br.x - tl.x, br.y - tl.y);
  ctx.imageSmoothingEnabled = true;
}

function drawScan() {
  if (!scanPoints.length) return;
  for (let i = 0; i < scanPoints.length; i += 2) {
    const sx = scanPoints[i], sy = scanPoints[i + 1];
    const p = w2s(sx, sy);
    if (p.x < -5 || p.x > canvas.width + 5 || p.y < -5 || p.y > canvas.height + 5) continue;
    ctx.fillStyle = scanColor(Math.sqrt(sx*sx + sy*sy));
    ctx.beginPath(); ctx.arc(p.x, p.y, 2, 0, Math.PI * 2); ctx.fill();
  }
}

function drawTrajectory() {
  if (trajectoryPoints.length < 4) return;
  ctx.strokeStyle = 'rgba(255,255,255,0.35)';
  ctx.lineWidth = 2;
  ctx.lineCap = 'round';
  ctx.lineJoin = 'round';
  ctx.beginPath();
  const p0 = w2s(trajectoryPoints[0], trajectoryPoints[1]);
  ctx.moveTo(p0.x, p0.y);
  for (let i = 2; i < trajectoryPoints.length; i += 2) {
    const p = w2s(trajectoryPoints[i], trajectoryPoints[i + 1]);
    ctx.lineTo(p.x, p.y);
  }
  ctx.stroke();
}

function drawRobot() {
  const o = w2s(robotPose ? robotPose.x : 0, robotPose ? robotPose.y : 0);
  const yaw = robotPose ? robotPose.yaw : 0;
  ctx.save();
  ctx.translate(o.x, o.y);
  ctx.rotate(Math.PI/2-yaw);  // 屏幕 y 翻转,取负号
  // 车身
  ctx.fillStyle = '#e0e0e0';
  ctx.beginPath();
  ctx.moveTo(0, -9); ctx.lineTo(-5.5, 6); ctx.lineTo(5.5, 6);
  ctx.closePath(); ctx.fill();
  // 机头朝向点
  ctx.fillStyle = '#0f0';
  ctx.beginPath(); ctx.arc(0, -12, 2.4, 0, Math.PI*2); ctx.fill();
  ctx.restore();
}

function drawScaleBar() {
  const pxPerM = vScale;
  let barM = 1;
  if (pxPerM < 15) barM = 5;
  else if (pxPerM < 30) barM = 2;
  const barPx = barM * pxPerM;
  const x = 20, y = canvas.height - 26;
  ctx.strokeStyle = '#fff';
  ctx.lineWidth = 2;
  ctx.beginPath(); ctx.moveTo(x, y); ctx.lineTo(x + barPx, y); ctx.stroke();
  ctx.beginPath();
  ctx.moveTo(x, y - 5); ctx.lineTo(x, y + 5);
  ctx.moveTo(x + barPx, y - 5); ctx.lineTo(x + barPx, y + 5);
  ctx.stroke();
  ctx.fillStyle = '#fff';
  ctx.font = '12px system-ui';
  ctx.textAlign = 'center';
  ctx.fillText(barM >= 1 ? barM + 'm' : (barM*100) + 'cm', x + barPx/2, y - 8);
}

// 2026-10-26: 地图上叠加语义区域（客厅/卧室/厨房…），来自 navigation_state.regions
function drawRegions() {
  const regions = window.__navRegions;
  if (!regions || !regions.length) return;
  ctx.save();
  for (const r of regions) {
    const pts = r.points || [];
    if (pts.length < 3) continue;
    ctx.beginPath();
    pts.forEach((p, i) => { const s = w2s(p[0], p[1]); i ? ctx.lineTo(s.x, s.y) : ctx.moveTo(s.x, s.y); });
    ctx.closePath();
    ctx.fillStyle = (r.color || '#4f9dff') + '22';
    ctx.fill();
    ctx.strokeStyle = (r.color || '#4f9dff') + 'aa';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    let cx = 0, cy = 0;
    for (const p of pts) { cx += p[0]; cy += p[1]; }
    cx /= pts.length; cy /= pts.length;
    const c = w2s(cx, cy);
    ctx.fillStyle = '#dce8f5';
    ctx.font = '12px "Segoe UI", "Microsoft YaHei", sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText(r.name, c.x, c.y);
  }
  ctx.restore();
}

function render(ts) {
  if (controlPage) return;
  frameCount++;
  if (ts - lastFpsTime > 1000) {
    fps = Math.round(frameCount / ((ts - lastFpsTime) / 1000));
    frameCount = 0;
    lastFpsTime = ts;
    document.getElementById('s-fps').textContent = fps;
  }

  // 跟随模式:持续居中到机器人
  if (follow && robotPose) { vCx = robotPose.x; vCy = robotPose.y; }

  ctx.fillStyle = '#0a0a14';
  ctx.fillRect(0, 0, canvas.width, canvas.height);
  drawGrid();
  drawTrajectory();
  drawMap();
  drawRegions();
  drawScan();
  drawRobot();
  drawScaleBar();
  requestAnimationFrame(render);
}

// ── 工具栏 ──────────────────────────────────────────────────────
function setFollow(f) {
  follow = f;
  document.getElementById('btn-follow').classList.toggle('active', f);
  document.getElementById('btn-follow').textContent = f ? '◉' : '○';
}
setFollow(true);
document.getElementById('btn-follow').addEventListener('click', () => setFollow(!follow));

function zoomBy(f) {
  vScale = Math.max(5, Math.min(500, vScale * f));
}
document.getElementById('btn-zoomin').addEventListener('click', () => zoomBy(1.3));
document.getElementById('btn-zoomout').addEventListener('click', () => zoomBy(1/1.3));

function fitView() {
  if (mapMeta && mapMeta.width * mapMeta.resolution > 0) {
    const mw = mapMeta.width * mapMeta.resolution;
    const mh = mapMeta.height * mapMeta.resolution;
    vScale = Math.min(canvas.width * 0.85 / mw, canvas.height * 0.85 / mh, 500);
  }
  vCx = robotPose ? robotPose.x : 0;
  vCy = robotPose ? robotPose.y : 0;
}
document.getElementById('btn-reset').addEventListener('click', () => { fitView(); setFollow(true); });

document.getElementById('btn-full').addEventListener('click', () => {
  const full = document.getElementById('win-map').classList.toggle('full');
  document.getElementById('btn-full').classList.toggle('active', full);
});

// 2026-10-25: 实时建图窗口折叠（只改 mapping 页的网格行高，别影响控制台布局）
// 2026-10-25: 实时里程计卡片折叠（只藏内容，标题保留）
document.getElementById('btn-odometry-collapse').addEventListener('click', () => {
  const card = document.querySelector('.mapping-odometry');
  const collapsed = card.classList.toggle('collapsed');
  const button = document.getElementById('btn-odometry-collapse');
  button.textContent = collapsed ? '展开' : '收起';
  button.setAttribute('aria-expanded', String(!collapsed));
});

document.getElementById('btn-map-collapse').addEventListener('click', () => {
  const win = document.getElementById('win-map');
  const collapsed = win.classList.toggle('collapsed');
  document.body.classList.toggle('map-collapsed', collapsed);
  const button = document.getElementById('btn-map-collapse');
  button.textContent = collapsed ? '展开' : '收起';
  button.setAttribute('aria-expanded', String(!collapsed));
  requestAnimationFrame(() => window.dispatchEvent(new Event('resize')));
});

// ── 交互:平移 / 缩放 / 复位 ─────────────────────────────────────
canvas.addEventListener('wheel', e => {
  e.preventDefault();
  const factor = e.deltaY < 0 ? 1.15 : 1/1.15;
  const mouse = s2w(e.clientX, e.clientY);
  vScale = Math.max(5, Math.min(500, vScale * factor));
  vCx = mouse.x + (vCx - mouse.x) * (factor < 1 ? 1/factor : factor);
  vCy = mouse.y + (vCy - mouse.y) * (factor < 1 ? 1/factor : factor);
}, { passive: false });

function panStart(e) {
  dragging = true;
  dragLast = { x: e.clientX, y: e.clientY };
  if (follow) setFollow(false);  // 手动拖动时退出跟随
}
function panMove(e) {
  if (!dragging) return;
  const dx = (e.clientX - dragLast.x) / vScale;
  const dy = (e.clientY - dragLast.y) / vScale;
  vCx -= dx; vCy += dy;
  dragLast = { x: e.clientX, y: e.clientY };
}
function panEnd() { dragging = false; }

canvas.addEventListener('mousedown', panStart);
canvas.addEventListener('mousemove', panMove);
canvas.addEventListener('mouseup', panEnd);
canvas.addEventListener('mouseleave', panEnd);

canvas.addEventListener('touchstart', e => {
  if (e.touches.length === 1) panStart({ clientX: e.touches[0].clientX, clientY: e.touches[0].clientY });
}, { passive: false });
canvas.addEventListener('touchmove', e => {
  e.preventDefault();
  if (!dragging || e.touches.length !== 1) return;
  panMove({ clientX: e.touches[0].clientX, clientY: e.touches[0].clientY });
}, { passive: false });
canvas.addEventListener('touchend', panEnd);

canvas.addEventListener('dblclick', () => { fitView(); setFollow(true); });

// ── 启动 ─────────────────────────────────────────────────────────
socket.once('map', () => { fitView(); setFollow(true); });
requestAnimationFrame(render);
</script>
<script src="/navigation.js?v=20261026-regions3"></script>
<script src="/mapping.js?v=20261023-odom1"></script>
<script src="/control.js?v=20261026-video4"></script>
<script src="/autonomy.js?v=20261026-inspection1"></script>
<script src="/missions.js?v=20261026-inspection1"></script>
<script src="/navigation_page.js?v=20261023-navigation1"></script>
</body>
</html>"""


from map_editor import register_map_editor
register_map_editor(app, lambda: _ros_node, lambda: _navigation)

if __name__ == "__main__":
    main()
