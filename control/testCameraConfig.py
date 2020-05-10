import cv2
import time

# Set desired parameters
desiredWidth  = 1920
desiredHeight = 1080
desiredFPS  = 30

# Camera properties 
cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, desiredWidth)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desiredHeight)
cam.set(cv2.CAP_PROP_FPS, desiredFPS)

# Counting variables
frameCount = 0
startTime = time.time()

# Run test
print('Test running for 10 secounds')
while (time.time() < startTime+10):
    _, frame = cam.read()
    frameCount += 1

# Record final time and frame size
endTime = time.time()
actualHeight, actualWidth, _ = frame.shape 

# Print results
print('Frame Width\tD: ', desiredWidth, '\tA: ', actualWidth)
print('Frame Height\tD: ', desiredHeight, '\tA: ', actualHeight)
print('FPS\t\tD: ', desiredFPS, '\t\tA: ', round(frameCount / (endTime - startTime),2))
