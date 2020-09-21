import cv2 as cv2
import time

# Import custom class
import sys
sys.path.append('../')
from T265 import T265
from filter import TimeSync

cam = T265()
sync = TimeSync(0.5)
sync.startTimer()
counter = 0
startTime = time.time()

# cam = cv2.VideoCapture(0)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cam.set(cv2.CAP_PROP_FPS, 30)
        
try:
    while(True):
        # Get frames
        tempA = cam.Img1
        tempB = cam.Img2

        # _, tempA = cam.read()
        # _, tempB = cam.read()
  
        # Write frames
        cv2.imwrite(str(counter) + 'A.png', tempA)
        cv2.imwrite(str(counter) + 'B.png', tempB)
        
        # Stabilize and increment
        counter += 1
        _ = sync.stabilize()
        
except KeyboardInterrupt:
    print('Closing')

finally:
    print('Done')
    print(time.time() - startTime, counter)
    cam.close() 
    