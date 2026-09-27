#!/usr/bin/env python3
"""RPLIDAR A1 Web Mapper — real-time LiDAR visualization + REST API."""

import json
import math
import threading
import time
import logging

from flask import Flask, jsonify, request, render_template_string
from rplidar import RPLidar, RPLidarException

# ── Config ──────────────────────────────────────────────────
PORT = 5800
DEVICE = "/dev/ttyUSB0"
BAUDRATE = 115200
MAX_POINTS = 360
SCAN_HISTORY = 5  # keep last N scans

# ── Globals ─────────────────────────────────────────────────
app = Flask(__name__)
log = logging.getLogger("rplidar_web")
log.setLevel(logging.INFO)

lidar = None
lidar_lock = threading.Lock()
scans = []  # list of lists of (quality, angle, distance)
latest_scan = []
motor_on = False
device_info = {}
health_status = ("unknown", 0)

# ── HTML Template ───────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>RPLIDAR A1 - 犀牛派</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0f;color:#c8d6e5;font-family:'Segoe UI',system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;min-height:100vh}
h1{font-size:1.4rem;margin:12px 0 4px;letter-spacing:0.5px}
.status-bar{display:flex;gap:24px;font-size:0.82rem;color:#8395a7;margin-bottom:8px}
.status-bar span.good{color:#2ed573}
.status-bar span.warn{color:#ffa502}
.status-bar span.bad{color:#ff4757}
canvas{display:block;background:#111118;border-radius:12px;
  box-shadow:0 0 30px rgba(0,200,255,0.06)}
.legend{display:flex;gap:20px;font-size:0.75rem;color:#576574;margin-top:8px}
.legend i{display:inline-block;width:12px;height:12px;border-radius:50%;margin-right:4px;vertical-align:middle}
.legend .near{background:#00d2ff}
.legend .mid{background:#7bed9f}
.legend .far{background:#ffa502}
</style>
</head>
<body>
<h1>RPLIDAR A1 - 犀牛派</h1>
<div class="status-bar">
  <span id="status">connecting...</span>
  <span id="fps">0 FPS</span>
  <span id="points">0 points</span>
  <span id="motor">motor: --</span>
</div>
<canvas id="c" width="520" height="520"></canvas>
<div class="legend">
  <div><i class="near"></i>near (&lt;1m)</div>
  <div><i class="mid"></i>mid (1-3m)</div>
  <div><i class="far"></i>far (&gt;3m)</div>
</div>

<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const CX = canvas.width / 2, CY = canvas.height / 2;
const R = 240;
const SCALE = R / 6000; // 6m max range

let lastFrame = performance.now();
let frameCount = 0;

function colorByDist(mm) {
  if (mm <= 1000) return [0, 210, 255, 1.0];
  if (mm <= 3000) {
    let t = (mm - 1000) / 2000;
    return [123 * t, 237 - 50 * t, 159 - 50 * t, 0.9];
  }
  return [255, 165, 2, 0.7];
}

function draw(points) {
  ctx.fillStyle = '#111118';
  ctx.fillRect(0, 0, canvas.width, canvas.height);

  // Range rings
  for (let d = 1000; d <= 6000; d += 1000) {
    const r = d * SCALE;
    ctx.beginPath(); ctx.arc(CX, CY, r, 0, Math.PI * 2);
    ctx.strokeStyle = 'rgba(255,255,255,0.05)'; ctx.stroke();
    ctx.fillStyle = 'rgba(255,255,255,0.2)';
    ctx.font = '10px monospace';
    ctx.fillText((d / 1000).toFixed(0) + 'm', CX + r - 16, CY - 2);
  }

  // Crosshair
  ctx.beginPath();
  ctx.moveTo(CX - R, CY); ctx.lineTo(CX + R, CY);
  ctx.moveTo(CX, CY - R); ctx.lineTo(CX, CY + R);
  ctx.strokeStyle = 'rgba(255,255,255,0.08)'; ctx.stroke();

  // LiDAR points
  for (const p of points) {
    const [q, deg, mm] = p;
    if (mm <= 0 || mm > 6000) continue;
    const rad = (deg - 90) * Math.PI / 180;
    const x = CX + Math.cos(rad) * mm * SCALE;
    const y = CY + Math.sin(rad) * mm * SCALE;
    const [cr, cg, cb, ca] = colorByDist(mm);
    ctx.fillStyle = 'rgba(' + cr + ',' + cg + ',' + cb + ',' + ca + ')';
    ctx.beginPath(); ctx.arc(x, y, 2, 0, Math.PI * 2); ctx.fill();
  }
}

async function poll() {
  try {
    const r = await fetch('/api/scan');
    const data = await r.json();
    draw(data.points || []);
    document.getElementById('points').textContent = (data.points || []).length + ' points';
    document.getElementById('status').innerHTML = '<span class="good">scanning</span>';
    document.getElementById('motor').textContent = 'motor: ON';
  } catch (e) {
    document.getElementById('status').innerHTML = '<span class="warn">waiting</span>';
    document.getElementById('motor').textContent = 'motor: --';
  }

  frameCount++;
  const now = performance.now();
  if (now - lastFrame >= 1000) {
    document.getElementById('fps').textContent =
      Math.round(frameCount * 1000 / (now - lastFrame)) + ' FPS';
    lastFrame = now; frameCount = 0;
  }
}

setInterval(poll, 80);
</script>
</body>
</html>"""


def connect_lidar():
    """Connect to LiDAR with retries."""
    global lidar
    for attempt in range(5):
        try:
            l = RPLidar(DEVICE, baudrate=BAUDRATE, timeout=1)
            info = l.get_info()
            health = l.get_health()
            log.info("LiDAR connected: model=%s hw=%s fw=%s.%s serial=%s health=%s",
                     info.get("model"), info.get("hardware"),
                     info.get("firmware", (0, 0))[0],
                     info.get("firmware", (0, 0))[1],
                     info.get("serialnumber", "?")[:8] + "...",
                     health[0])
            lidar = l
            return info, health
        except Exception as e:
            log.warning("LiDAR connect attempt %d/5: %s", attempt + 1, e)
            time.sleep(2)
    raise RuntimeError("Failed to connect to LiDAR after 5 attempts")


def scanner_loop():
    """Background thread: continuously read scans."""
    global latest_scan, scans, motor_on, health_status, lidar

    while True:
        try:
            with lidar_lock:
                if lidar is None:
                    time.sleep(1)
                    continue
                l = lidar

            if not motor_on:
                l.start_motor()
                motor_on = True
                time.sleep(1)

            for scan in l.iter_scans(max_buf_meas=2000, min_len=10):
                latest_scan = [(q, a, d) for q, a, d in scan if d > 0]
                scans.append(latest_scan)
                if len(scans) > SCAN_HISTORY:
                    scans = scans[-SCAN_HISTORY:]
                break  # one scan per outer iteration
        except RPLidarException as e:
            log.error("LiDAR error: %s - reconnecting...", e)
            with lidar_lock:
                try:
                    lidar.disconnect()
                except Exception:
                    pass
                lidar = None
                motor_on = False
            try:
                info, health = connect_lidar()
                device_info.update(info)
                health_status = health
            except Exception:
                time.sleep(3)
        except Exception as e:
            log.error("Scanner error: %s", e)
            time.sleep(1)


# ── Routes ──────────────────────────────────────────────────
@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/st")
def status():
    """Simple status check (compatible with old endpoint)."""
    with lidar_lock:
        has_lidar = lidar is not None
    n = len(latest_scan)
    return jsonify({
        "status": "running" if has_lidar else "no_lidar",
        "scans": len(scans),
        "latest_points": n,
        "motor": motor_on,
    })


@app.route("/api/info")
def api_info():
    return jsonify({
        "device": device_info,
        "health": list(health_status),
        "motor": motor_on,
        "serial": DEVICE,
    })


@app.route("/api/scan")
def api_scan():
    return jsonify({
        "points": latest_scan,
        "count": len(latest_scan),
        "motor": motor_on,
    })


@app.route("/api/scan/latest")
def api_scan_latest():
    """Return latest scan with optional ?jsonp=callback support."""
    data = {
        "points": latest_scan,
        "count": len(latest_scan),
        "ts": time.time(),
    }
    cb = request.args.get("jsonp") or request.args.get("callback")
    if cb:
        return f"{cb}({json.dumps(data)})", 200, {"Content-Type": "application/javascript"}
    return jsonify(data)


# ── Main ────────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s"
    )

    log.info("Connecting to LiDAR at %s...", DEVICE)
    info, health = connect_lidar()
    device_info.update(info)
    health_status = health

    log.info("Starting scanner thread...")
    t = threading.Thread(target=scanner_loop, daemon=True)
    t.start()

    log.info("Starting web server on 0.0.0.0:%d", PORT)
    app.run(host="0.0.0.0", port=PORT, debug=False, threaded=True)
