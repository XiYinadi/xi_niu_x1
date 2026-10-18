#!/usr/bin/env python3
"""
slam_viz.py — 犀牛派 SLAM 实时 Web 可视化
===========================================
单文件方案: ROS2 订阅 + Flask-SocketIO WebSocket 推送 + 平铺多窗口前端
窗口: 地图 / 相机 / 运行日志 (三窗口平铺网格)

相机画面: cam_stream.py 独立进程捕获 USB 相机 → JPEG → slam_viz 转发 SocketIO
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
from pathlib import Path

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
from flask import Flask, render_template_string, request
from flask_socketio import SocketIO, emit

# ── ROS2 ───────────────────────────────────────────────────────────
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
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

# ── Camera config (env-overridable) ───────────────────────────────
# 相机帧来自独立进程 fire_detect.py 的原始帧端点(相机是单读设备,
# fire_detect 独占 /dev/video2,故不再用 cam_stream;ROS env 会破坏 cv2 的 V4L2)
CAMERA_STREAM_URL = os.environ.get(
    "SLAM_VIZ_CAM_URL", "http://127.0.0.1:5812/raw.jpg"
)
CAMERA_FPS = int(os.environ.get("SLAM_VIZ_CAM_FPS", 10))  # 目标帧率

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
    ("/tmp/slam_viz.log", "viz"),
    ("/tmp/motor_driver.log", "motor"),
    ("/tmp/laser_odom.log", "odom"),
    ("/tmp/slam.log", "slam"),
    ("/tmp/lidar.log", "lidar"),
    ("/tmp/tf_laser.log", "tf"),
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
    origin_x, origin_y, map_seq.
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
        self._map_dirty = False
        self._scan_count = 0
        self._map_count = 0

        # 「建图到底在不在跑」只能靠**数据流**判 —— 计数器只能证明「曾经收到过」,
        # 证明不了「现在还在收」。0.0 表示从没收到过(monotonic 单调,不会是 0)。
        self._last_scan_t = 0.0
        self._last_map_t = 0.0

        # QoS: /map needs transient_local to receive last map immediately
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        scan_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self._map_sub = self.create_subscription(
            OccupancyGrid, "/map", self._map_cb, map_qos,
        )
        self._scan_sub = self.create_subscription(
            LaserScan, "/scan", self._scan_cb, scan_qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_cb, 10,
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
            self._twist_pub.publish(t)

    def set_motor_arm(self, on):
        b = Bool()
        b.data = bool(on)
        with self._twist_lock:
            self._arm_pub.publish(b)

    def _map_cb(self, msg: OccupancyGrid):
        with self._lock:
            self._latest_map_msg = msg
            self._map_dirty = True
            self._map_count += 1
            self._last_map_t = time.monotonic()

    def _scan_cb(self, msg: LaserScan):
        # Convert outside lock (CPU work, no shared state access)
        xy = laser_scan_to_xy(msg)
        with self._lock:
            self._latest_scan_xy = xy
            self._scan_count += 1
            self._last_scan_t = time.monotonic()

    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # yaw from quaternion (ZYX convention)
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        with self._lock:
            # 存的时候就四舍五入到 3 位(1mm,远超地图精度):全精度浮点
            # 像 1.2345678901234567 每个 18 字节,是纯浪费 —— 单条轨迹本来
            # 能到 ~200KB,砍掉 2/3 是在这里而不是在发送端。
            self._trajectory.append((round(x, 3), round(y, 3)))
            self._traj_total += 1
            if len(self._trajectory) > TRAJ_MAX:
                self._trajectory = self._trajectory[-TRAJ_MAX:]
            self._latest_pose = (x, y, yaw)

    def full_trajectory(self) -> dict:
        """整条轨迹,只给新连上来的客户端补历史用。

        **不动 `_traj_sent`** —— 那个游标只属于广播循环的增量语义。这里如果
        也去推进它,广播那边就会以为新客户端已经拿过历史,后续增量发不全。
        """
        with self._lock:
            if not self._trajectory:
                return None
            return {
                "reset": True,          # 让客户端整体重建,而不是追加
                "from": self._traj_total - len(self._trajectory),
                "points": [c for pt in self._trajectory for c in pt],
                "total": self._traj_total,
            }

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
                    "count": len(self._latest_scan_xy) // 2,
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

            result["_stats"] = {
                "map_count": self._map_count,
                "scan_count": self._scan_count,
            }

            # 建图状态判据(见 mapping_status)。**另起一个键,不动 _stats** ——
            # _stats 连广播循环那关都过不去(只转发 map/scan/trajectory/pose),
            # 改它等于顺手改既有结构。
            now = time.monotonic()
            result["feed"] = {
                "scan_age": (now - self._last_scan_t) if self._last_scan_t else None,
                "map_age": (now - self._last_map_t) if self._last_map_t else None,
            }

        # Encode map PNG outside lock (CPU-heavy, no shared state)
        if "_map_msg" in result:
            map_data = occgrid_to_png_base64(result.pop("_map_msg"))
            if map_data:
                result["map"] = map_data

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
def index():
    return render_template_string(HTML_TEMPLATE)


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


def mapping_status(scan_age, map_age) -> str:
    """age 单位秒;None = 从没收到过。纯函数,可 headless 单测。"""
    if map_age is not None and map_age < MAP_FRESH_S:
        return "mapping"
    if scan_age is not None and scan_age < SCAN_FRESH_S:
        return "lidar_only"
    return "off"


MAPPING_CTL = os.environ.get("MAPPING_CTL", os.path.expanduser("~/mapping_ctl.sh"))
MAPPING_TIMEOUT_S = 45

_map_lock = threading.Lock()
_map_busy = False       # 一次只允许一个 start/stop 在路上
_map_action = None      # 正在进行的是什么,给 UI 显示「启动中…」用
_map_last = {"scan_age": None, "map_age": None}


def _map_state_payload():
    status = mapping_status(_map_last["scan_age"], _map_last["map_age"])
    return {
        "status": status,
        "scan_age": _map_last["scan_age"],
        "map_age": _map_last["map_age"],
        "busy": _map_busy,
        "action": _map_action,
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
        _map_busy = True
        _map_action = action
    threading.Thread(target=_mapping_run, args=(action,), daemon=True).start()
    return True


# ══════════════════════════════════════════════════════════════════════
# Broadcast Loop (SocketIO background task)
# ══════════════════════════════════════════════════════════════════════

_ros_node: SlamVizNode = None  # set by main()


def broadcast_loop():
    """SocketIO background task: poll ROS state → WebSocket emit."""
    last_map_emit = 0.0
    last_scan_hash = None
    last_pose_key = None
    last_map_status = None
    log.info("Broadcast loop started")
    while True:
        try:
            state = _ros_node.get_state_snapshot()

            # 建图状态:每轮都重算(状态会**在没人发包的时候**变陈旧,所以不能只在
            # 收到数据时算),变了才发 —— 边沿触发,和下面 pose 一个路子。
            feed = state.get("feed")
            if feed:
                _map_last["scan_age"] = feed["scan_age"]
                _map_last["map_age"] = feed["map_age"]
            p = _map_state_payload()
            key = (p["status"], p["busy"], p["action"])
            if key != last_map_status:
                socketio.emit("mapping_state", p)
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
                    socketio.emit("map", state["map"])
                    last_map_emit = now
                    log.debug(
                        "map emitted: %dx%d @ %.2fm",
                        state["map"]["width"],
                        state["map"]["height"],
                        state["map"]["resolution"],
                    )

            # Emit scan when changed
            if "scan" in state:
                pts = state["scan"]["points"]
                h = hash(len(pts)) if pts else None
                if h != last_scan_hash:
                    socketio.emit("scan", state["scan"])
                    last_scan_hash = h

            # 轨迹:state 里只在**真的有新点**时才带这个 key,所以这里是真·增量
            if "trajectory" in state:
                socketio.emit("trajectory", state["trajectory"])

            # Emit pose when changed
            if "pose" in state:
                p = state["pose"]
                key = (p["x"], p["y"], p["yaw"])
                if key != last_pose_key:
                    socketio.emit("pose", p)
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
_cam_status = {"running": False, "w": 0, "h": 0, "fps": 0.0, "error": None}


def _cam_publish_state():
    with _cam_lock:
        state = dict(_cam_status)
    socketio.emit("camera_state", state)


def camera_loop():
    """Daemon thread: pull JPEG from cam_stream.py → base64 → emit 'frame'."""
    log.info("Camera stream thread started (url=%s)", CAMERA_STREAM_URL)
    frame_budget = 1.0 / CAMERA_FPS
    fps_n, fps_t0 = 0, time.time()

    while True:
        try:
            if not _cam_enabled.is_set():
                # Idle: just poll the event (cam_stream handles the device)
                if _cam_status.get("running"):
                    _cam_status.update(running=False)
                    _cam_publish_state()
                time.sleep(0.2)
                continue

            # Pull the latest frame from the standalone camera process
            with urllib.request.urlopen(CAMERA_STREAM_URL, timeout=2.0) as resp:
                jpg = resp.read()
                w = int(resp.headers.get("X-Frame-W", 0))
                h = int(resp.headers.get("X-Frame-H", 0))

            if not jpg:
                time.sleep(frame_budget)
                continue

            b64 = base64.b64encode(jpg).decode("ascii")
            socketio.emit("frame", {"img": b64, "w": w, "h": h})

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
            time.sleep(max(0.0, frame_budget - (time.time() - now)))

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
_drive_owner_note_ts = 0.0

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


def _send_drive(l, a, armed):
    """把归一化摇杆值直发 pwm_proc(绕开 ROS)。

    尽力而为:没起 pwm_proc 时只是丢包,绝不能让这条路把网页的按键回调搞崩,
    所以这里吞掉 OSError 只记 debug。

    armed 一并带上,是因为 pwm_proc 不跑 ROS、拿不到 /motor_arm —— 它靠这个
    字段决定动不动电机,上电默认 false(必须网页点「启用电机」)。
    """
    try:
        _pwm_sock.sendto(
            json.dumps({"l": float(l), "a": float(a),
                        "armed": bool(armed)}).encode(),
            (PWM_UDP_HOST, PWM_UDP_PORT))
    except OSError as e:
        log.debug("pwm udp send failed: %s", e)


# ── SocketIO camera events ────────────────────────────────────────

@socketio.on("disconnect")
def _on_disconnect():
    global _motor_armed, _drive_owner
    _cam_clients.discard(request.sid)
    if not _cam_clients:
        _cam_enabled.clear()
        log.info("All clients left — camera stream OFF")
    _fire_clients.discard(request.sid)
    if not _fire_clients:
        _fire_enabled.clear()
        log.info("All clients left — fire stream OFF")
    # 驾驶者断开 → 急停(防止浏览器关页后机器人继续跑)。
    # 非驾驶者的客户端断开不动电机 —— 否则另一台设备的页面一刷新就把车停了。
    if _motor_armed and request.sid == _drive_owner:
        _motor_armed = False
        _drive_owner = None
        _ros_node.publish_cmd_vel(0.0, 0.0)
        _ros_node.set_motor_arm(False)
        _send_drive(0.0, 0.0, False)      # 直连通道也要停,pwm_proc 不看 ROS
        _emit_motor_state()
        log.warning("Driver disconnected — motors disarmed")


@socketio.on("camera_control")
def _on_camera_control(data):
    action = (data or {}).get("action")
    if action == "start":
        _cam_clients.add(request.sid)
        _cam_enabled.set()
        _cam_status.update(error=None)
        log.info("Camera stream ON (sid=%s)", request.sid)
    elif action == "stop":
        _cam_clients.discard(request.sid)
        if not _cam_clients:
            _cam_enabled.clear()
            log.info("Camera stream OFF (sid=%s)", request.sid)
    _cam_publish_state()


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


@socketio.on("motor_control")
def _on_motor_control(data):
    global _motor_armed, _drive_owner, _drive_owner_note_ts
    action = (data or {}).get("action")
    if action == "vel":
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
        # 摇杆/键盘发来已归一化 -1..1 的速度
        lin = float(data.get("linear", 0.0))
        ang = float(data.get("angular", 0.0))
        _ros_node.publish_cmd_vel(lin, ang)
        _send_drive(lin, ang, _motor_armed)   # 别硬编码 True;见上
    elif action == "arm":
        # 谁按的「启用电机」谁就是驾驶者;解除时让出控制权
        _motor_armed = bool(data.get("armed", False))
        _drive_owner = request.sid if _motor_armed else None
        _ros_node.set_motor_arm(_motor_armed)
        # 立刻同步给直连通道:解除时这一包就是急停,pwm_proc 不必等 deadman
        _send_drive(0.0, 0.0, _motor_armed)
        _emit_motor_state()
        log.info("Motor %s (driver=%s)",
                 "ARMED" if _motor_armed else "disarmed", _drive_owner or "-")
    elif action == "estop":
        _motor_armed = False
        _drive_owner = None
        _ros_node.publish_cmd_vel(0.0, 0.0)
        _ros_node.set_motor_arm(False)
        _send_drive(0.0, 0.0, False)
        _emit_motor_state()
        log.warning("ESTOP pressed — motors disarmed")


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
    socketio.emit("log", entry)


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
    # 之前这里没有任何日志,连上来用的是 websocket 还是 long-polling 完全看不见
    # —— 而这两者的延迟差一个数量级(polling 每个包都要一次 HTTP 往返)。
    _up = str(request.environ.get("HTTP_UPGRADE", "")).lower()
    log.info("Client %s connected (transport=%s)",
             request.sid, "websocket" if _up == "websocket" else "polling")
    # Send a snapshot of recent logs to the newly connected client
    with _log_lock:
        snapshot = list(_log_buffer)
    if snapshot:
        socketio.emit("log_snapshot", snapshot, to=request.sid)
    # Push current motor armed state (default disarmed)
    socketio.emit("motor_state",
                  {"armed": _motor_armed, "driver": _drive_owner}, to=request.sid)
    # 建图状态也一样要补发:边缘触发的广播循环只会在**变化时**发,不补这一发,
    # 刷新页面后按钮会一直停在 HTML 里的初始文案,直到下次状态变化才跟上。
    socketio.emit("mapping_state", _map_state_payload(), to=request.sid)
    # 让客户端知道自己的 sid,才能判断「我是不是驾驶者」
    socketio.emit("you", {"sid": request.sid}, to=request.sid)

    # 轨迹走的是**增量**,新客户端手里没有历史 —— 不补这一发,它就只会从
    # 「连上的那一刻」开始画,之前走过的路是空的(刷新页面、开第二个标签页
    # 尤其明显)。只发给这个 sid,不影响已经在线的客户端。
    if _ros_node is not None:
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
    log.info("Received signal %s, shutting down...", signum)
    rclpy.shutdown()
    sys.exit(0)


def main():
    global _ros_node

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    # Verify ROS2 env
    if "AMENT_PREFIX_PATH" not in os.environ:
        log.warning("ROS2 environment not detected!")
        log.warning("Run: source /opt/ros/humble/setup.bash")
        log.warning("     source ~/lidar_ws/install/setup.bash")

    rclpy.init(args=sys.argv[1:])
    _ros_node = SlamVizNode()

    # Spin rclpy in daemon thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(_ros_node,), daemon=True)
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
        socketio.run(app, host="0.0.0.0", port=PORT)
    finally:
        log.info("Cleaning up...")
        _ros_node.destroy_node()
        rclpy.shutdown()


# ══════════════════════════════════════════════════════════════════════
# HTML + JS 前端 (嵌入) — 平铺三窗口: 地图 / 相机 / 日志
# ══════════════════════════════════════════════════════════════════════

HTML_TEMPLATE = r"""<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>犀牛派 SLAM 控制台</title>
<script src="https://cdn.socket.io/4.7.5/socket.io.min.js"></script>
<style>
  :root {
    --bg:#0a0a14; --panel:#0d0d18; --panel2:#12121f; --line:#1e2030;
    --fg:#c8ccd4; --dim:#6b7080; --accent:#00d2ff;
  }
  * { margin:0; padding:0; box-sizing:border-box; }
  html,body { height:100%; }
  body {
    background:var(--bg); color:var(--fg); overflow:hidden;
    font-family:'Segoe UI',system-ui,sans-serif;
  }

  /* ── 顶栏 ─────────────────────────────── */
  #status-bar {
    position:fixed; top:0; left:0; right:0; height:34px; z-index:30;
    display:flex; align-items:center; gap:18px; padding:0 14px; font-size:12px;
    background:rgba(10,10,20,0.94); border-bottom:1px solid var(--line);
  }
  #status-conn { display:flex; align-items:center; gap:6px; }
  #conn-dot { width:8px; height:8px; border-radius:50%; background:#f44; }
  #conn-dot.live { background:#0f0; box-shadow:0 0 6px #0f0; }
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
    background:var(--panel); border:1px solid var(--line); border-radius:8px;
    display:flex; flex-direction:column; overflow:hidden; min-height:0; min-width:0;
    transition:border-color .15s;
  }
  .win:hover { border-color:#2a2e44; }
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
    font-size:13px; line-height:1; padding:3px 7px; border-radius:4px;
    font-family:inherit;
  }
  .btn:hover { background:#1e2030; color:#fff; }
  .btn.active { background:#0a6; color:#fff; }
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
  #map-body { background:#0a0a14; }
  #map-canvas { width:100%; height:100%; display:block; cursor:grab; }
  #map-canvas:active { cursor:grabbing; }
  #legend {
    position:absolute; left:10px; bottom:10px; z-index:2;
    display:flex; gap:14px; font-size:11px; color:#9aa0b0;
    background:rgba(10,10,20,0.72); padding:4px 10px; border-radius:6px;
    border:1px solid var(--line); pointer-events:none;
  }
  #legend .lg { display:flex; align-items:center; gap:5px; }
  #legend i { display:inline-block; width:10px; height:10px; border-radius:50%; }

  /* ── 驾驶面板 (地图窗口右下角) ───────────── */
  #drive {
    position:absolute; right:10px; bottom:10px; z-index:6;
    background:rgba(13,13,24,0.92); border:1px solid var(--line); border-radius:10px;
    padding:8px; display:flex; flex-direction:column; gap:6px;
    font-size:12px; user-select:none;
  }
  #drive-title { color:var(--dim); font-weight:600; text-align:center; }
  #drive-btns { display:flex; gap:6px; }
  #btn-arm {
    flex:1; padding:6px 8px; border:none; border-radius:6px; cursor:pointer;
    background:#2a2e44; color:var(--dim); font-size:12px; font-family:inherit;
    transition:background .15s, color .15s;
  }
  #btn-arm.armed { background:#0a6; color:#fff; }
  #btn-arm:disabled { opacity:.4; cursor:default; }
  #btn-estop {
    flex:1; padding:6px 8px; border:none; border-radius:6px; cursor:pointer;
    background:#5a1010; color:#ff9090; font-size:12px; font-family:inherit;
  }
  #btn-estop:active { background:#c00; color:#fff; }
  /* 建图按钮:放在地图窗口标题栏、和视图 glyph **同一组**的首位。
     不另起一个 span.grp —— .grp 带 margin-left:auto,两个 auto 会把空白平分,
     按钮就飘到标题中间去了。配色沿用 #btn-arm 那套。 */
  #btn-map {
    padding:3px 9px; margin-right:6px; border:none; border-radius:4px; cursor:pointer;
    background:#2a2e44; color:var(--dim); font-size:11px; font-family:inherit;
    transition:background .15s, color .15s; white-space:nowrap;
  }
  #btn-map:hover { color:#fff; }
  #btn-map.on      { background:#0a6;    color:#fff; }       /* 建图中 */
  #btn-map.partial { background:#7a5a10; color:#ffd479; }    /* 雷达在,SLAM 没起 */
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
        rgba(122,140,200,.20) 32% 32.7%, transparent 32.7% 64.5%,
        rgba(122,140,200,.20) 64.5% 65.2%, transparent 65.2%),
      radial-gradient(circle at 50% 50%, #1a1a2e, #0e0e1c);
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
  .joy-pad:active { cursor:grabbing; border-color:#3a4a7a; }
  .joy-knob {
    position:absolute; left:50%; top:50%; width:52px; height:52px; z-index:2;
    margin:-26px 0 0 -26px; border-radius:50%;
    background:var(--accent); opacity:.85; pointer-events:none;
    box-shadow:0 0 12px rgba(0,210,255,.4);
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
  #joy-mode { display:flex; border:1px solid var(--line); border-radius:6px; overflow:hidden; }
  #joy-mode button {
    padding:3px 9px; border:none; cursor:pointer; font-family:inherit; font-size:11px;
    background:#2a2e44; color:var(--dim); transition:background .15s, color .15s;
  }
  #joy-mode button + button { border-left:1px solid var(--line); }
  #joy-mode button.on { background:var(--accent); color:#0b0b16; font-weight:600; }
  #drive-val {
    color:var(--accent); text-align:center; font-size:12px;
    font-variant-numeric:tabular-nums; white-space:pre;
  }
  #drive-hint { color:var(--dim); text-align:center; font-size:10px; }

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
  #fire-hud.ok  { color:#4cff4c; text-shadow:0 0 8px rgba(76,255,76,.5); }
  #fire-hud.alarm { color:#ff2d2d; text-shadow:0 0 10px rgba(255,45,45,.7); }
  #fire-hud.off { color:var(--dim); }
  #s-fire.alarm { color:#ff2d2d; }
  #s-fire.ok { color:#4cff4c; }

  /* ── 日志窗口 ─────────────────────────── */
  #log-list {
    height:100%; overflow-y:auto; padding:4px 0;
    background:#0c0c16; font-family:Consolas,'Courier New',monospace;
    font-size:11px; line-height:1.55;
  }
  .log-line {
    padding:0 8px; white-space:pre-wrap; word-break:break-all;
    color:var(--fg);
  }
  .log-line:hover { background:#141424; }
  .lt { color:#4a5060; margin-right:8px; font-variant-numeric:tabular-nums; }
  .l-tag { margin-right:8px; font-weight:600; }
  .lv-debug .l-msg { color:#6b7080; }
  .lv-warn .l-msg, .lv-warning .l-msg { color:#ffa502; }
  .lv-error .l-msg { color:#ff5a5a; }
  .lv-critical .l-msg { color:#ff2d2d; }
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
  }
</style>
</head>
<body>

<div id="status-bar">
  <span id="status-conn"><span id="conn-dot"></span><span id="conn-text">Disconnected</span></span>
  <span><span class="stat-label">Map</span><span id="s-map">—</span></span>
  <span><span class="stat-label">Scan</span><span id="s-scan">—</span></span>
  <span><span class="stat-label">Path</span><span id="s-trajectory">0</span></span>
  <span><span class="stat-label">FPS</span><span id="s-fps">0</span></span>
  <span><span class="stat-label">Fire</span><span id="s-fire">—</span></span>
  <span class="spacer"></span>
  <span id="s-clock">--:--:--</span>
</div>

<div id="workspace">

  <section class="win" id="win-map">
    <header class="win-title">
      <span class="t">Map</span>
      <span class="hud" id="hud">x:— y:— θ:—</span>
      <span class="grp">
        <button id="btn-map" disabled>建图</button>
        <button class="btn" id="btn-follow" title="跟随机器人">◉</button>
        <button class="btn" id="btn-zoomout" title="缩小">−</button>
        <button class="btn" id="btn-zoomin" title="放大">+</button>
        <button class="btn" id="btn-reset" title="复位视图">⌂</button>
        <button class="btn" id="btn-full" title="最大化/还原地图">⛶</button>
      </span>
    </header>
    <div class="win-body" id="map-body">
      <canvas id="c"></canvas>
      <div id="legend">
        <span class="lg"><i style="background:#999"></i>地图</span>
        <span class="lg"><i style="background:#00d2ff"></i>&lt;1m</span>
        <span class="lg"><i style="background:#7bed9f"></i>1–3m</span>
        <span class="lg"><i style="background:#ffa502"></i>&gt;3m</span>
      </div>
      <div id="drive">
        <div id="drive-head">
          <div id="drive-title">Drive</div>
          <div id="joy-mode" role="group" aria-label="摇杆模式">
            <button type="button" data-mode="dual" title="左杆油门 / 右杆转向;两轴独立,可同时满舵">双杆</button>
            <button type="button" data-mode="single" title="一个盘同时给油门与转向;圆门,斜推时两轴各占一部分">单杆</button>
          </div>
        </div>
        <div id="drive-btns">
          <button id="btn-arm" disabled>启用电机</button>
          <button id="btn-estop" disabled>急停</button>
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
      </div>
    </div>
  </section>

  <section class="win" id="win-cam">
    <header class="win-title">
      <span class="t">Camera</span>
      <span class="sub" id="cam-info">—</span>
      <span class="grp">
        <button class="btn" id="btn-cam-collapse" title="折叠/展开相机">—</button>
      </span>
    </header>
    <div class="win-body" id="cam-body">
      <img id="cam-img" alt="camera" />
      <div id="cam-err" style="display:none;">相机离线</div>
    </div>
  </section>

  <section class="win" id="win-fire">
    <header class="win-title">
      <span class="t">Fire</span>
      <span class="hud off" id="fire-hud">—</span>
      <span class="sub" id="fire-info">—</span>
      <span class="grp">
        <button class="btn" id="btn-fire-collapse" title="折叠/展开火情">—</button>
      </span>
    </header>
    <div class="win-body" id="fire-body">
      <img id="fire-img" alt="fire" />
      <div id="fire-err" style="display:none;">火情检测离线</div>
    </div>
  </section>

  <section class="win" id="win-log">
    <header class="win-title">
      <span class="t">Log</span>
      <span class="sub" id="log-count">0</span>
      <span class="grp">
        <button class="btn active" id="btn-log-scroll" title="自动滚动">⇓</button>
        <button class="btn" id="btn-log-clear" title="清空">✕</button>
        <button class="btn" id="btn-log-collapse" title="折叠/展开日志">—</button>
      </span>
    </header>
    <div class="win-body" id="log-body">
      <div id="log-list"></div>
    </div>
  </section>

</div>

<script>
// ── 状态 ─────────────────────────────────────────────────────────
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
const socket = io();
const connDot = document.getElementById('conn-dot');
const connText = document.getElementById('conn-text');

socket.on('connect', () => {
  connDot.className = 'live';
  connText.textContent = 'Live';
  // 相机 + 火情窗口默认开启
  socket.emit('camera_control', { action: 'start' });
  socket.emit('fire_control', { action: 'start' });
});

socket.on('disconnect', () => {
  connDot.className = '';
  connText.textContent = 'Disconnected';
});

socket.on('map', data => {
  mapMeta = data;
  mapOriginX = data.origin_x;
  mapOriginY = data.origin_y;
  document.getElementById('s-map').textContent =
    `${data.width}×${data.height} @ ${(data.resolution*100).toFixed(0)}cm`;
  const img = new Image();
  img.onload = () => { mapImg = img; };
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

// ── 相机 ────────────────────────────────────────────────────────
const camImg = document.getElementById('cam-img');
const camInfo = document.getElementById('cam-info');
const camErr = document.getElementById('cam-err');

socket.on('frame', data => {
  camImg.src = 'data:image/jpeg;base64,' + data.img;
  camErr.style.display = 'none';
  if (data.w) camInfo.textContent = `${data.w}×${data.h}`;
});

socket.on('camera_state', state => {
  if (state.running) camInfo.textContent =
    `${state.w ? state.w+'×'+state.h+' ' : ''}${state.fps ? state.fps.toFixed(1)+' fps' : ''}`.trim();
  else if (state.error) camInfo.textContent = 'err';
});

document.getElementById('btn-cam-collapse').addEventListener('click', () => {
  const collapsed = document.getElementById('win-cam').classList.toggle('collapsed');
  updateGridClasses();
  socket.emit('camera_control', { action: collapsed ? 'stop' : 'start' });
});

// ── 火情 ────────────────────────────────────────────────────────
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
  // 右列三行(相机/火情/日志):折叠的行缩成 auto,其余均分
  const ws = document.getElementById('workspace');
  const row = el => el.classList.contains('collapsed') ? 'auto' : 'minmax(0,1fr)';
  ws.style.gridTemplateRows =
    row(document.getElementById('win-cam')) + ' ' +
    row(document.getElementById('win-fire')) + ' ' +
    row(document.getElementById('win-log'));
}

// ── 驾驶 (电机控制) ──────────────────────────────────────────────
let armed = false;
let mySid = null;        // 本页面的 SocketIO sid (服务端 connect 时下发)
let driverSid = null;    // 当前驾驶者的 sid; null = 还没人被授权
const btnArm = document.getElementById('btn-arm');
const btnEstop = document.getElementById('btn-estop');
const valEl = document.getElementById('drive-val');
const hintEl = document.getElementById('drive-hint');

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
};
const padList = [PADS.thr, PADS.steer, PADS.stick];

// 每种模式**读哪几个盘**。互斥地读是有意的:被隐藏的那个盘万一残留了 active
// (隐藏一个正持着 pointer capture 的元素时,pointerup 不保证还会送到它),
// 互斥能保证它绝不会偷偷继续出力。
const MODE_PADS = { dual: [PADS.thr, PADS.steer], single: [PADS.stick] };
let joyMode = 'dual';

// 同一份页面开在两台设备上时,双边都会 20Hz 发 /cmd_vel,没握摇杆的那台
// 持续灌 0 → 真值被切断 → 电机抖。服务端只认驾驶者,这里同步禁掉非驾驶者
// 的发送与操作,免得用户对着没反应的摇杆使劲。
function isDriver() {
  return !armed || driverSid === null || driverSid === mySid;
}

function driveGate() {
  const ok = isDriver();
  // 所有盘都刷(含当前隐藏的):这样切换模式时,新露出来的那个盘**已经**带着
  // 正确的 opacity/pointerEvents,不会先闪一下可点状态。
  for (const p of padList) {
    p.el.style.opacity = ok ? '1' : '.4';
    p.el.style.pointerEvents = ok ? 'auto' : 'none';
  }
  hintEl.textContent = ok
    ? (joyMode === 'single' ? '单杆推即走,斜推同时转向,或 WASD/方向键'
                            : '左杆油门 右杆转向,或 WASD/方向键')
    : '已被其它设备接管 —— 点「启用电机」抢回';
}

socket.on('you', m => { mySid = m.sid; driveGate(); });

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
}
socket.on('connect', () => driveUi(true));
socket.on('disconnect', () => driveUi(false));

btnArm.addEventListener('click', () => {
  socket.emit('motor_control', { action: 'arm', armed: !armed });
});
btnEstop.addEventListener('click', () => {
  socket.emit('motor_control', { action: 'estop' });
});

// ── 建图按钮 (地图窗口标题栏) ───────────────────────────────
// 状态是**服务端按 /scan 与 /map 的实际数据流**算出来的,不是「我按过了」。
// 所以别人从 ssh 起的管线,这里照样显示成建图中 —— 按钮不说谎是设计目标。
const btnMap = document.getElementById('btn-map');
let mapStatus = 'off';     // off | lidar_only | mapping  (服务端只发这三个)
let mapBusy  = false;      // 服务端有 start/stop 在路上
let mapAction = null;

// 三态 => 文案 / class / 鼠标悬停说明。文案说的是**当前状态**;
// 点下去做什么由 mapClick() 按状态决定。
const MAP_UI = {
  off:        { text: '启动建图', cls: null,
                tip: '点一下:拉起 雷达 + 激光里程计 + 静态TF + slam_toolbox' },
  lidar_only: { text: 'SLAM 未起 ⟳', cls: 'partial',
                tip: '雷达在出数据,但地图没有在更新 —— 雷达之后的某一环(里程计/TF/slam_toolbox)没起来或已崩。再点一次会重启整条链路' },
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
  btnMap.disabled = !socket.connected;
  btnMap.classList.toggle('on',      ui.cls === 'on');
  btnMap.classList.toggle('partial', ui.cls === 'partial');
  btnMap.textContent = ui.text;
  btnMap.title = ui.tip;
}

socket.on('mapping_state', st => {
  mapStatus = st.status || 'off';
  mapBusy   = !!st.busy;
  mapAction = st.action || null;
  mapPaint();
});

btnMap.addEventListener('click', () => {
  if (mapBusy) return;
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
document.addEventListener('keydown', e => {
  if (['ArrowUp','ArrowDown','ArrowLeft','ArrowRight',' '].includes(e.key)) e.preventDefault();
  if (e.repeat) return;
  keys[e.key] = true;
});
document.addEventListener('keyup', e => { keys[e.key] = false; });

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
  p.el.addEventListener('pointerup', () => { p.active = false; p.pid = null; padReset(p); });
  p.el.addEventListener('pointercancel', () => { p.active = false; p.pid = null; padReset(p); });
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
    socket.emit('motor_control', { action: 'vel', linear: 0, angular: 0 });
  }
}
window.addEventListener('blur', () => releaseAll(true));
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
  if (keys['w'] || keys['ArrowUp']) l += 1;
  if (keys['s'] || keys['ArrowDown']) l -= 1;
  if (keys['a'] || keys['ArrowLeft']) a += 1;
  if (keys['d'] || keys['ArrowRight']) a -= 1;
  return {
    l: Math.max(-1, Math.min(1, l)),
    a: Math.max(-1, Math.min(1, a)),
  };
}
function fmtSigned(v) { return (v < 0 ? '-' : '+') + Math.abs(v).toFixed(2); }

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
  const t = driveTarget();
  valEl.textContent = '前进 ' + fmtSigned(t.l) + '　转向 ' + fmtSigned(t.a);
  if (!armed || !isDriver()) return;
  socket.emit('motor_control', { action: 'vel', linear: t.l, angular: t.a });
}

function setDriveRunning(on) {
  if (on) {
    if (driveTimer === null) driveTimer = setInterval(driveTick, DRIVE_PERIOD_MS);
  } else if (driveTimer !== null) {
    clearInterval(driveTimer);
    driveTimer = null;
    // 定时器停了读数就不再刷新,留着最后一帧的「前进 +0.80」是**假**的 ——
    // 那个值此刻既没在发、车也不认。宁可写清楚它已经暂停。
    valEl.textContent = '已暂停(页面不可见)';
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
  const o = w2s(0, 0);
  const yaw = robotPose ? robotPose.yaw : 0;
  ctx.save();
  ctx.translate(o.x, o.y);
  ctx.rotate(-yaw);  // 屏幕 y 翻转,取负号
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

function render(ts) {
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
</body>
</html>"""


if __name__ == "__main__":
    main()
