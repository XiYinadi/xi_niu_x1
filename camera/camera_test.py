import cv2
import os

# 必须在 import cv2 之前设好 DISPLAY（但 cv2 已经 import 了...）
# 关键：需要在启动脚本时设置环境变量

cap = cv2.VideoCapture('/dev/video2', cv2.CAP_V4L2)
if not cap.isOpened():
    print('无法打开摄像头 /dev/video2')
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

print('摄像头已打开，按 Q 退出...')
print(f'分辨率: {cap.get(cv2.CAP_PROP_FRAME_WIDTH):.0f}x{cap.get(cv2.CAP_PROP_FRAME_HEIGHT):.0f}')

while True:
    ret, frame = cap.read()
    if not ret:
        print('无法读取画面')
        break
    cv2.imshow('USB Camera (犀牛派)', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print('已退出')
