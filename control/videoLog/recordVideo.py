import time
import cv2

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
aList = []
bList = []
counter = 0
startTime = time.time()
        
try:
    while(True):
        # Get frames
        aList.append(cam.Img1)
        bList.append(cam.Img2)

        # Stabilize and increment
        counter += 1
        _ = sync.stabilize()
        
except KeyboardInterrupt:
    print('Closing')
    endTime = time.time()
    dt = endTime - startTime

finally:
    print('Capture Rate: ', counter/dt)
    cam.close() 
    
    print('Saving A Images')
    for ii in range(len(aList)):
        cv2.imwrite(str(ii) + 'A.png', aList[ii])
        
    print('Saving B Images')
    for ii in range(len(bList)):
        cv2.imwrite(str(ii) + 'B.png', bList[ii])
        
    print('Done')
    