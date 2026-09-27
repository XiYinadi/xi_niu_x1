#!/usr/bin/env python3
"""
cam_stream.py — 犀牛派 USB 相机 HTTP 服务(独立进程)
====================================================
为什么独立进程:ROS2 环境(rclpy)导入后会污染同进程内 cv2 的 V4L2
相机打开,所以相机捕获必须跑在【系统 python3、不导入 rclpy】的
单独进程里。本服务只做两件事:捕获相机 + HTTP 提供 JPEG。

用法(用系统 python,不要 source ROS):
  /usr/bin/python3 ~/cam_stream.py

环境变量:
  SLAM_CAM_DEVICE  设备节点      (默认 /dev/video2)
  SLAM_CAM_WIDTH   流目标宽度    (默认 640,保持宽高比)
  SLAM_CAM_QUALITY JPEG 质量     (默认 70)
  SLAM_CAM_FPS     目标帧率      (默认 10)
  SLAM_CAM_PORT    监听端口      (默认 5811)

端点:
  GET /frame.jpg    最新一帧 JPEG,响应头带 X-Frame-W / X-Frame-H
  GET /stream.mjpg  MJPEG 流(浏览器 <img> 直连)
  GET /             状态 JSON
"""

import json
import os
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

DEVICE = os.environ.get("SLAM_CAM_DEVICE", "/dev/video2")
STREAM_WIDTH = int(os.environ.get("SLAM_CAM_WIDTH", 640))
QUALITY = int(os.environ.get("SLAM_CAM_QUALITY", 70))
FPS = int(os.environ.get("SLAM_CAM_FPS", 10))
HOST = os.environ.get("SLAM_CAM_HOST", "0.0.0.0")
PORT = int(os.environ.get("SLAM_CAM_PORT", "5811"))

_lock = threading.Lock()
_latest: bytes = None  # 最新 JPEG bytes
_meta = {"w": 0, "h": 0, "fps": 0.0}
_ready = threading.Event()


def capture_loop():
    """后台线程:持续捕获 → 缩小 → JPEG,写入共享缓冲。"""
    global _latest, _meta
    cap = None
    budget = 1.0 / FPS
    n, t0 = 0, time.time()

    while True:
        try:
            if cap is None:
                cap = cv2.VideoCapture(DEVICE)
                if not cap.isOpened():
                    raise RuntimeError("open failed")
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                print(f"[cam_stream] camera opened: {DEVICE}", flush=True)

            ret, frame = cap.read()
            if not ret or frame is None:
                raise RuntimeError("read failed (disconnected?)")

            h, w = frame.shape[:2]
            if w > STREAM_WIDTH:
                nh = int(h * STREAM_WIDTH / w)
                frame = cv2.resize(
                    frame, (STREAM_WIDTH, nh), interpolation=cv2.INTER_AREA
                )

            ok, buf = cv2.imencode(
                ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, QUALITY]
            )
            if ok:
                with _lock:
                    _latest = buf.tobytes()
                    _meta["w"], _meta["h"] = frame.shape[1], frame.shape[0]
                _ready.set()

            n += 1
            now = time.time()
            if now - t0 >= 1.0:
                _meta["fps"] = round(n / (now - t0), 1)
                n, t0 = 0, now

            time.sleep(max(0.0, budget - (time.time() - now)))

        except Exception as e:
            print(f"[cam_stream] error: {e} — retry in 2s", flush=True)
            _ready.clear()
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            cap = None
            time.sleep(2.0)


class Handler(BaseHTTPRequestHandler):
    server_version = "cam_stream/1.0"

    def log_message(self, fmt, *args):
        pass  # 安静模式,不打访问日志

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            self._json({"device": DEVICE, **_meta, "alive": _ready.is_set()})
        elif path == "/frame.jpg":
            self._frame_jpeg()
        elif path == "/stream.mjpg":
            self._mjpeg()
        else:
            self.send_error(404)

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _frame_jpeg(self):
        if not _ready.wait(timeout=5):
            self.send_error(503, "no frame yet")
            return
        with _lock:
            data, w, h = _latest, _meta["w"], _meta["h"]
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Frame-W", str(w))
        self.send_header("X-Frame-H", str(h))
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        try:
            while True:
                if not _ready.wait(timeout=5):
                    break
                with _lock:
                    data = _latest
                    if data is last:
                        time.sleep(0.03)
                        continue
                    last = _latest
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: " + str(len(data)).encode() + b"\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    threading.Thread(target=capture_loop, daemon=True).start()
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print(
        f"[cam_stream] serving http://{HOST}:{PORT}  device={DEVICE} "
        f"width={STREAM_WIDTH} quality={QUALITY}",
        flush=True,
    )
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("[cam_stream] bye", flush=True)


if __name__ == "__main__":
    main()
