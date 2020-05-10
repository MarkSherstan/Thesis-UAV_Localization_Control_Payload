import cv2
import time

# Camera properties 
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
cam.set(cv2.CAP_PROP_FPS, 10)

# Counting variables
frameCount = 0
startTime = time.time()

while (True):
    try:
        _, frame = cam.read()
        frameCount += 1
    except KeyboardInterrupt:
        break

endTime = time.time()


height, width, _ = frame.shape 
print('Width: ', width, ' Height: ', height)

print('Frames: ', frameCount, 'Time: ', (endTime - startTime))
print('FPS: ', frameCount / (endTime - startTime))
