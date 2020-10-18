import cv2 as cv2
import time

# Import custom classes
import sys
sys.path.append('../')
from T265 import T265
from filter import TimeSync

# Initialize
cam = T265()

sync = TimeSync(1/30)
sync.startTimer()

# Variables
counter = 0
startTime = time.time()
        
try:
    while(True):
        # Get frames
        tempA = cam.Img1
        tempB = cam.Img2

        # Write frames
        cv2.imwrite(str(counter) + 'A.png', tempA)
        cv2.imwrite(str(counter) + 'B.png', tempB)
        
        # Stabilize and increment
        counter += 1
        _ = sync.stabilize()
        
except KeyboardInterrupt:
    print('Closing')
    endTime = time.time()
    dt = endTime - startTime

finally:
    print('Done')
    print('Capture Rate: ', counter/dt)
    cam.close() 
    