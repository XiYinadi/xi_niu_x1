#!/usr/bin/env python3
"""诊断:连接 slam_viz :5800,开 fire_control,验证能收到 fire_frame / fire_status。"""
import json
import time

import socketio

sio = socketio.Client(logger=False, reconnection=False)
got_frame = False
got_status = False


@sio.on("fire_frame")
def on_frame(d):
    global got_frame
    got_frame = True
    print("[test] fire_frame img len =", len(d.get("img", "")), flush=True)


@sio.on("fire_status")
def on_status(st):
    global got_status
    got_status = True
    key = ("fire", "confidence", "reason", "samples", "latency_s", "running")
    print("[test] fire_status:", json.dumps({k: st.get(k) for k in key}, ensure_ascii=False),
          flush=True)


print("[test] connecting...", flush=True)
sio.connect("http://127.0.0.1:5800")
sio.emit("fire_control", {"action": "start"})
print("[test] connected, fire_control=start sent", flush=True)

t0 = time.time()
while time.time() - t0 < 8:
    if got_frame and got_status:
        break
    time.sleep(0.5)

print("[test] RESULT got_frame =", got_frame, "| got_status =", got_status, flush=True)
sio.emit("fire_control", {"action": "stop"})
sio.disconnect()
