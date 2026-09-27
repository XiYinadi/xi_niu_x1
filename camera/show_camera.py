import cv2

# ´ò¿ªÉãÏñÍ·£¬0 ´ú±íµÚÒ»¸öÉãÏñÍ·£¬Èç¹ûÓÐÁ½¸ö£¬¿ÉÄÜÓÃ 1
cap = cv2.VideoCapture(2)

# ÈÃÉãÏñÍ·»­Ãæ³ß´çÐ¡Ò»µã£¬²»»á¿¨
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

while True:
    # ´ÓÉãÏñÍ·¶ÁÈ¡Ò»Ö¡»­Ãæ
    ret, frame = cap.read()
    if not ret:
        print("¶Á²»µ½»­Ãæ£¬¼ì²éÉãÏñÍ·ÊÇ·ñ²åºÃ")
        break

    # ÔÚÒ»¸ö´°¿ÚÀïÏÔÊ¾»­Ãæ
    cv2.imshow('ÎÒµÄÉãÏñÍ·', frame)

    # µÈ´ý 1 ºÁÃë£¬Èç¹ûÔÚÕâÆÚ¼ä°´ÏÂÁË¼üÅÌÉÏµÄ 'q' ¼ü£¬¾ÍÍË³ö
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# ¹Ø±ÕÉãÏñÍ·ºÍ´°¿Ú
cap.release()
cv2.destroyAllWindows()
