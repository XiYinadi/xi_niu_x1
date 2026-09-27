#!/usr/bin/env python3
"""
fire_detect.py — 犀牛派 火焰/烟雾检测(本地 VLM 直判,独立进程)
================================================================
把相机帧周期性送本地 Qwen2.5-VL-3B(AidGenSE :8888)判断画面
是否有火焰(明火)/燃烧/明显烟雾,结果实时叠加到标注视频流。

⚠️ 关键约束:必须用系统 python3 跑,不导入 rclpy
   (rclpy 会污染同进程 cv2 的 V4L2 相机打开,见 cam_stream.py 注释)。

用法:
  /usr/bin/python3 ~/fire_detect.py

环境变量:
  FIRE_DETECT_DEVICE       相机设备       (默认 /dev/video2)
  FIRE_DETECT_WIDTH        流目标宽度     (默认 640,保持宽高比)
  FIRE_DETECT_QUALITY      JPEG 质量      (默认 70)
  FIRE_DETECT_FPS          捕获帧率       (默认 10)
  FIRE_DETECT_PORT         标注流端口     (默认 5812)
  FIRE_DETECT_INTERVAL     判定间隔秒     (默认 3.0)
  FIRE_DETECT_CONF         告警置信度阈值 (默认 0.5)
  FIRE_DETECT_API          VLM API 基址   (默认 http://127.0.0.1:8888)
  FIRE_DETECT_MODEL        VLM 模型名     (默认 qwen2.5-vl-3b-instruct-392x392-qnn2.36-w4a16-qcs8550)
  FIRE_DETECT_SAVE_EVENTS  保存告警帧     (默认 1)

端点:
  GET /            状态 JSON(最新判定/置信度/延迟/样本数)
  GET /frame.jpg   最新标注帧 JPEG
  GET /stream.mjpg 标注 MJPEG 流(浏览器 <img> 直连)
"""

import base64
import json
import os
import re
import threading
import time
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

DEVICE = os.environ.get("FIRE_DETECT_DEVICE", "/dev/video2")
STREAM_WIDTH = int(os.environ.get("FIRE_DETECT_WIDTH", 640))
QUALITY = int(os.environ.get("FIRE_DETECT_QUALITY", 70))
FPS = int(os.environ.get("FIRE_DETECT_FPS", 10))
HOST = os.environ.get("FIRE_DETECT_HOST", "0.0.0.0")
PORT = int(os.environ.get("FIRE_DETECT_PORT", "5812"))
INTERVAL = float(os.environ.get("FIRE_DETECT_INTERVAL", "3.0"))
CONF_THRESH = float(os.environ.get("FIRE_DETECT_CONF", "0.5"))
API_BASE = os.environ.get("FIRE_DETECT_API", "http://127.0.0.1:8888")
MODEL = os.environ.get(
    "FIRE_DETECT_MODEL",
    "qwen2.5-vl-3b-instruct-392x392-qnn2.36-w4a16-qcs8550",
)
SAVE_EVENTS = os.environ.get("FIRE_DETECT_SAVE_EVENTS", "1") == "1"

FIRE_PROMPT = (
    "你是消防监控系统的图像判读模块,只负责判断监控画面里是否真实存在火情。"
    "严格区分:**只有真实的火焰(明火)、物体正在燃烧、明显的烟柱才算火情。**"
    "以下情况一律不算火情,不要误报:指示灯/led/霓虹灯、彩色光源、"
    "反射光斑、晚霞夕阳、暖色物体(橙红物体但未燃烧)、显示器屏幕上的画面。"
    "只输出一个 JSON 对象,不要输出任何其他文字,格式为:"
    '{"fire": <true或false>, "confidence": <0到1之间,表示你对该判断的把握>, '
    '"reason": "<不超过20字的中文说明>"}'
)

_lock = threading.Lock()
_latest_bgr = None            # 最新原始帧(BGR,numpy)
_cap_count = 0                # 帧计数(用于 MJPEG 去重编码)
_capture_ok = False
_verdict = {                  # 最近一次 VLM 判定
    "fire": False,
    "confidence": 0.0,
    "reason": "等待首次判定...",
    "t": 0.0,
    "latency": 0.0,
    "samples": 0,
}


# ── 相机捕获线程 ──────────────────────────────────────────────────
def capture_loop():
    global _latest_bgr, _cap_count, _capture_ok
    cap = None
    budget = 1.0 / FPS
    while True:
        try:
            if cap is None:
                cap = cv2.VideoCapture(DEVICE)
                if not cap.isOpened():
                    raise RuntimeError("open failed")
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                print(f"[fire_detect] camera opened: {DEVICE}", flush=True)

            ret, frame = cap.read()
            if not ret or frame is None:
                raise RuntimeError("read failed (disconnected?)")

            h, w = frame.shape[:2]
            if w > STREAM_WIDTH:
                nh = int(h * STREAM_WIDTH / w)
                frame = cv2.resize(
                    frame, (STREAM_WIDTH, nh), interpolation=cv2.INTER_AREA
                )
            with _lock:
                _latest_bgr = frame
                _cap_count += 1
            _capture_ok = True
            time.sleep(budget)

        except Exception as e:
            print(f"[fire_detect] capture error: {e} — retry in 2s", flush=True)
            _capture_ok = False
            if cap is not None:
                try:
                    cap.release()
                except Exception:
                    pass
            cap = None
            time.sleep(2.0)


# ── VLM 判定 ─────────────────────────────────────────────────────
def _parse_content(content):
    """从 VLM 回复中抽出 JSON 对象;失败返回 None。"""
    content = content.strip()
    # 去掉 ```json ... ``` 围栏
    if content.count("```") >= 2:
        content = content.split("```")[1]
        if content.startswith("json"):
            content = content[4:]
    start = content.find("{")
    end = content.rfind("}")
    if start == -1 or end == -1 or end < start:
        return None
    try:
        return json.loads(content[start:end + 1])
    except json.JSONDecodeError:
        # 容错:正则抓 fire / confidence
        m_fire = re.search(r'"fire"\s*:\s*(true|false)', content)
        if not m_fire:
            return None
        m_conf = re.search(r'"confidence"\s*:\s*([0-9.]+)', content)
        m_reason = re.search(r'"reason"\s*:\s*"([^"]*)"', content)
        return {
            "fire": m_fire.group(1) == "true",
            "confidence": float(m_conf.group(1)) if m_conf else 0.0,
            "reason": m_reason.group(1) if m_reason else "",
        }


def _vlm_ask(bgr):
    """送一帧给本地 VLM,返回 (解析后的 dict 或 None, latency秒)。"""
    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
    if not ok:
        return None, 0.0
    b64 = base64.b64encode(buf.tobytes()).decode()
    payload = {
        "model": MODEL,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "image_url",
                 "image_url": {"url": "data:image/jpeg;base64," + b64}},
                {"type": "text", "text": FIRE_PROMPT},
            ],
        }],
        "stream": False,
        "temperature": 0.7,  # 必须 > 0,0.0 会触发采样器断言 abort
    }
    req = urllib.request.Request(
        API_BASE + "/v1/chat/completions",
        data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"},
    )
    t = time.time()
    with urllib.request.urlopen(req, timeout=60) as resp:
        data = json.loads(resp.read().decode())
    latency = time.time() - t
    content = data["choices"][0]["message"]["content"]
    return _parse_content(content), latency


def _save_event(bgr, conf, reason):
    try:
        d = os.path.expanduser("~/fire_events")
        os.makedirs(d, exist_ok=True)
        ts = time.strftime("%Y%m%d_%H%M%S")
        path = os.path.join(d, f"fire_{ts}_{int(conf * 100)}.jpg")
        cv2.imwrite(path, bgr)
        print(f"[fire_detect] 事件帧已保存: {path}", flush=True)
    except Exception as e:
        print(f"[fire_detect] 保存事件帧失败: {e}", flush=True)


def judge_loop():
    """判定线程:每隔 INTERVAL 秒抽最新帧送 VLM,更新 _verdict。"""
    global _verdict
    while True:
        with _lock:
            bgr = _latest_bgr
        if bgr is None:
            time.sleep(1.0)
            continue
        try:
            obj, latency = _vlm_ask(bgr)
        except Exception as e:
            print(f"[fire_detect] VLM error: {e} — retry in {max(2, INTERVAL)}s",
                  flush=True)
            time.sleep(max(2.0, INTERVAL))
            continue
        if obj is None:
            print("[fire_detect] 无法解析 VLM 回复", flush=True)
            time.sleep(max(2.0, INTERVAL))
            continue

        fire = bool(obj.get("fire"))
        conf = float(obj.get("confidence", 0.0))
        reason = str(obj.get("reason", ""))[:40]
        with _lock:
            _verdict = {
                "fire": fire,
                "confidence": conf,
                "reason": reason,
                "t": time.time(),
                "latency": latency,
                "samples": _verdict["samples"] + 1,
            }

        flag = "🔥 火焰/烟雾" if fire else "✅ 无火焰"
        print(f"[fire_detect] {flag} conf={conf:.2f} lat={latency:.2f}s "
              f"reason={reason}", flush=True)
        if fire and conf >= CONF_THRESH and SAVE_EVENTS:
            _save_event(bgr, conf, reason)
        time.sleep(max(0.5, INTERVAL))


# ── 标注渲染 ─────────────────────────────────────────────────────
def _annotated(bgr):
    """叠加最近判定到帧上,返回标注后的 BGR 帧。"""
    img = bgr.copy()
    with _lock:
        v = dict(_verdict)
    h, w = img.shape[:2]

    fire, conf = v["fire"], v["confidence"]
    alarm = fire and conf >= CONF_THRESH
    low = fire and not alarm

    color = (0, 190, 0)              # 绿:无火焰
    status = "OK NO FIRE"
    if alarm:
        color = (0, 0, 255)          # 红:告警
        status = "FIRE! FLAME/SMOKE"
    elif low:
        color = (0, 160, 255)        # 橙:疑似(低置信)
        status = "SUSPECT FIRE (low conf)"

    cv2.rectangle(img, (0, 0), (w, 30), color, -1)
    label = f"{status}  conf={conf:.2f}  {v['reason']}"
    cv2.putText(img, label, (8, 22), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2, cv2.LINE_AA)
    if alarm:
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 4)
    elif low:
        cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 160, 255), 2)

    tstr = time.strftime("%H:%M:%S", time.localtime(v["t"]))
    cv2.putText(img, f"VLM {v['latency']:.1f}s  #{v['samples']}  {tstr}",
                (8, h - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (255, 255, 255), 1, cv2.LINE_AA)
    return img


# ── HTTP 服务 ────────────────────────────────────────────────────
_last_served = -1
_last_jpeg = None
_last_served_raw = -1
_last_raw_jpeg = None


def _annotated_jpeg():
    """返回最新标注帧 JPEG(bytes);无帧返回 None。编码按帧去重。"""
    global _last_served, _last_jpeg
    with _lock:
        bgr = _latest_bgr
        count = _cap_count
        if bgr is None:
            return None
    if count != _last_served or _last_jpeg is None:
        ann = _annotated(bgr)
        ok, buf = cv2.imencode(".jpg", ann, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
        if not ok:
            return None
        _last_jpeg = buf.tobytes()
        _last_served = count
    return _last_jpeg


def _raw_jpeg():
    """返回最新原始帧 JPEG(未标注,供相机面板);无帧返回 None。"""
    global _last_served_raw, _last_raw_jpeg
    with _lock:
        bgr = _latest_bgr
        count = _cap_count
        if bgr is None:
            return None
    if count != _last_served_raw or _last_raw_jpeg is None:
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, QUALITY])
        if not ok:
            return None
        _last_raw_jpeg = buf.tobytes()
        _last_served_raw = count
    return _last_raw_jpeg


class Handler(BaseHTTPRequestHandler):
    server_version = "fire_detect/1.0"

    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        path = self.path.split("?")[0]
        if path in ("/", "/index.html"):
            self._json(self._status())
        elif path == "/frame.jpg":
            self._jpeg(_annotated_jpeg())
        elif path == "/raw.jpg":
            self._jpeg(_raw_jpeg())
        elif path == "/stream.mjpg":
            self._mjpeg()
        else:
            self.send_error(404)

    def _status(self):
        with _lock:
            v = dict(_verdict)
        return {
            "device": DEVICE,
            "capture_ok": _capture_ok,
            "fire": v["fire"],
            "confidence": v["confidence"],
            "reason": v["reason"],
            "last_check": v["t"],
            "latency_s": v["latency"],
            "samples": v["samples"],
            "conf_thresh": CONF_THRESH,
            "interval_s": INTERVAL,
        }

    def _json(self, obj):
        body = json.dumps(obj).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _jpeg(self, data):
        if data is None:
            self.send_error(503, "no frame yet")
            return
        self.send_response(200)
        self.send_header("Content-Type", "image/jpeg")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        with _lock:
            bgr = _latest_bgr
        if bgr is not None:
            h, w = bgr.shape[:2]
            self.send_header("X-Frame-W", str(w))
            self.send_header("X-Frame-H", str(h))
        self.end_headers()
        self.wfile.write(data)

    def _mjpeg(self):
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        last = None
        try:
            while True:
                data = _annotated_jpeg()
                if data is None:
                    time.sleep(0.5)
                    continue
                if data is last:
                    time.sleep(0.05)
                    continue
                last = data
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(b"Content-Length: " +
                                 str(len(data)).encode() + b"\r\n\r\n")
                self.wfile.write(data)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(1.0 / FPS)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main():
    threading.Thread(target=capture_loop, daemon=True).start()
    threading.Thread(target=judge_loop, daemon=True).start()
    srv = ThreadingHTTPServer((HOST, PORT), Handler)
    print(f"[fire_detect] serving http://{HOST}:{PORT}  device={DEVICE}  "
          f"interval={INTERVAL}s  conf>={CONF_THRESH}  model={MODEL}",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("[fire_detect] bye", flush=True)


if __name__ == "__main__":
    main()
