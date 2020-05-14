import cv2
import time

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_FPS, 60)
time.sleep(0.2)

_, frame = cam.read()
cv2.imwrite('test.jpg', frame)

cam.release()