#!/usr/bin/env python3
"""诊断:单次 VLM 长提示词请求是否会打崩服务。直接 cv2 抓相机帧。"""
import base64
import json
import time
import urllib.request

import cv2

FIRE_PROMPT = (
    "你是火灾监测系统,负责判断监控画面是否存在火情。"
    "请判断图中是否有火焰(明火)、燃烧迹象或明显烟雾。"
    "只输出一个 JSON 对象,不要输出任何其他文字,格式为:"
    '{"fire": <true或false>, "confidence": <0到1之间的数字>, '
    '"reason": "<不超过20字的中文说明>"}'
)

cap = cv2.VideoCapture("/dev/video2")
if not cap.isOpened():
    raise SystemExit("camera open failed")
ret, frame = cap.read()
cap.release()
if not ret:
    raise SystemExit("camera read failed")
ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
print(f"相机帧: {len(buf.tobytes())} bytes")

b64 = base64.b64encode(buf.tobytes()).decode()
payload = {
    "model": "qwen2.5-vl-3b-instruct-392x392-qnn2.36-w4a16-qcs8550",
    "messages": [{"role": "user", "content": [
        {"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + b64}},
        {"type": "text", "text": FIRE_PROMPT},
    ]}],
    "stream": False,
    "temperature": 0.7,  # 必须 > 0
}
req = urllib.request.Request(
    "http://127.0.0.1:8888/v1/chat/completions",
    data=json.dumps(payload).encode(),
    headers={"Content-Type": "application/json"},
)
t = time.time()
try:
    r = json.loads(urllib.request.urlopen(req, timeout=90).read())
    print(f"OK 延迟 {time.time()-t:.2f}s")
    print("回复:", r["choices"][0]["message"]["content"][:150])
except Exception as e:
    print("FAIL:", e)
