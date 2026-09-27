import cv2
import time

WIN = 'USB Camera (犀牛派)'
cv2.namedWindow(WIN, cv2.WINDOW_NORMAL)
cv2.resizeWindow(WIN, 640, 480)
print('摄像头预览启动...')

while True:
    cap = cv2.VideoCapture('/dev/video2')
    if not cap.isOpened():
        print('等待摄像头...')
        time.sleep(1)
        continue

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    print('摄像头已连接')

    while True:
        ret, frame = cap.read()
        if not ret:
            print('断连, 重试...')
            break
        cv2.imshow(WIN, frame)
        key = cv2.waitKey(1) & 0xFF
        # 按 Q 退出 或 点击关闭按钮
        if key == ord('q') or cv2.getWindowProperty(WIN, cv2.WND_PROP_VISIBLE) < 1:
            cap.release()
            cv2.destroyAllWindows()
            print('退出')
            exit(0)

    cap.release()
    time.sleep(1)
