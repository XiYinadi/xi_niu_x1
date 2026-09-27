#!/usr/bin/env python3
"""诊断:连接 slam_viz :5800,开 camera_control,验证能收到 frame(原始帧)。"""
import time

import socketio

sio = socketio.Client(logger=False, reconnection=False)
got = {"cam": False}


@sio.on("frame")
def on_frame(d):
    got["cam"] = True
    print("[test] camera frame img len =", len(d.get("img", "")), flush=True)


print("[test] connecting...", flush=True)
sio.connect("http://127.0.0.1:5800")
sio.emit("camera_control", {"action": "start"})
print("[test] connected, camera_control=start sent", flush=True)
time.sleep(5)
print("[test] RESULT camera frame received =", got["cam"], flush=True)
sio.emit("camera_control", {"action": "stop"})
sio.disconnect()
