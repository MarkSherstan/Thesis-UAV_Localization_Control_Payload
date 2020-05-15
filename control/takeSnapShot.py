import cv2
import time

cam = cv2.VideoCapture(-1, cv2.CAP_V4L)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
cam.set(cv2.CAP_PROP_FPS, 30)
time.sleep(0.2)

_, frame = cam.read()
cv2.imwrite('test.jpg', frame)

cam.release()