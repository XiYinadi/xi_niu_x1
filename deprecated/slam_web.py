#!/usr/bin/env python3
"""SLAM Web Visualizer — Flask server reading shared state from ROS bridge."""

import json
import os
import time
import threading

from flask import Flask, jsonify, render_template_string

PORT = int(os.environ.get("PORT", 5800))
STATE_FILE = "/tmp/slam_web_state.json"
app = Flask(__name__)

# ── HTML ────────────────────────────────────────────────────
HTML = r"""<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>SLAM Map - 犀牛派</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#0a0a0f;color:#c8d6e5;font-family:'Segoe UI',system-ui,sans-serif;
  display:flex;flex-direction:column;align-items:center;min-height:100vh}
h1{font-size:1.2rem;margin:10px 0 2px}
.status{font-size:0.78rem;color:#8395a7;margin-bottom:6px;display:flex;gap:20px}
.good{color:#2ed573}.warn{color:#ffa502}
canvas{background:#1a1a24;border-radius:8px;box-shadow:0 0 20px rgba(0,200,255,0.05)}
.legend{display:flex;gap:16px;font-size:0.7rem;color:#576574;margin-top:6px}
.legend span{display:inline-block;width:10px;height:10px;margin-right:4px}
</style>
</head>
<body>
<h1>SLAM Map - 犀牛派</h1>
<div class="status">
  <span id="map-status">map: --</span>
  <span id="scan-status">scan: --</span>
  <span id="fps">0 FPS</span>
</div>
<canvas id="c" width="600" height="600"></canvas>

<script>
const canvas = document.getElementById('c');
const ctx = canvas.getContext('2d');
const W = canvas.width, H = canvas.height;
let lastFrame = performance.now(), frames = 0;
let mapCache = null;

function drawMap(meta, data) {
  if (!meta || !data) return;
  const off = document.createElement('canvas');
  off.width = meta.width; off.height = meta.height;
  const octx = off.getContext('2d');
  const img = octx.createImageData(meta.width, meta.height);
  for (let i = 0; i < data.length; i++) {
    const v = data[i], j = i * 4;
    if (v < 0) { img.data[j]=25; img.data[j+1]=25; img.data[j+2]=30; img.data[j+3]=255; }
    else {
      const c = Math.floor(255 - v * 2.55);
      img.data[j]=c; img.data[j+1]=c; img.data[j+2]=c; img.data[j+3]=255;
    }
  }
  octx.putImageData(img, 0, 0);
  mapCache = {canvas: off, meta: meta};
}

function render() {
  ctx.fillStyle = '#1a1a24';
  ctx.fillRect(0, 0, W, H);
  const cx = W / 2, cy = H / 2;

  if (mapCache) {
    const m = mapCache.meta;
    // Scale map to fit canvas
    const scale = Math.min(W / (m.width * m.resolution * 20), H / (m.height * m.resolution * 20));
    const mw = m.width * m.resolution * 20 * scale;
    const mh = m.height * m.resolution * 20 * scale;
    const ox = cx - mw / 2, oy = cy - mh / 2;
    ctx.drawImage(mapCache.canvas, ox, oy, mw, mh);
  }

  // Crosshair
  ctx.strokeStyle = 'rgba(0,210,255,0.4)'; ctx.lineWidth = 1;
  ctx.beginPath(); ctx.moveTo(cx-25, cy); ctx.lineTo(cx+25, cy); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(cx, cy-25); ctx.lineTo(cx, cy+25); ctx.stroke();
  ctx.fillStyle = '#00d2ff'; ctx.beginPath(); ctx.arc(cx, cy, 3, 0, 2*Math.PI); ctx.fill();
}

async function poll() {
  try {
    const r = await fetch('/api/state');
    const s = await r.json();

    if (s.map_meta && s.map_data) {
      document.getElementById('map-status').innerHTML =
        '<span class="good">map: '+s.map_meta.width+'x'+s.map_meta.height+'</span>';
      drawMap(s.map_meta, s.map_data);
    } else {
      document.getElementById('map-status').innerHTML =
        '<span class="warn">map: waiting for movement...</span>';
    }

    if (s.scan_count > 0) {
      document.getElementById('scan-status').innerHTML =
        '<span class="good">scan: '+s.scan_count+' points</span>';
    } else {
      document.getElementById('scan-status').innerHTML =
        '<span class="warn">scan: waiting...</span>';
    }

    render();
  } catch(e) {
    console.error(e);
  }

  frames++;
  const now = performance.now();
  if (now - lastFrame >= 1000) {
    document.getElementById('fps').textContent =
      Math.round(frames*1000/(now-lastFrame))+' FPS';
    lastFrame = now; frames = 0;
  }
}

setInterval(poll, 500);
</script>
</body>
</html>"""


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/api/state")
def api_state():
    try:
        with open(STATE_FILE, "r") as f:
            return jsonify(json.load(f))
    except (FileNotFoundError, json.JSONDecodeError):
        return jsonify({"map_meta": None, "map_data": None, "scan_count": 0})


if __name__ == "__main__":
    print(f"SlamWeb serving on 0.0.0.0:{PORT}")
    app.run(host="0.0.0.0", port=PORT, debug=False, threaded=True)
