import cv2 as cv
import time

# Import custom class
import sys
sys.path.append('../')
from T265 import T265
from filter import TimeSync

cam = T265()
sync = TimeSync(1/30)
counter = 0

try:
    while(True):
        # Get frames
        tempA = cam.Img1
        tempB = cam.Img2

        # Write frames
        cv2.imwrite(str(ii) + 'A.png', tempA)
        cv2.imwrite(str(ii) + 'B.png', tempB)
        
        # Stabilize and increment
        counter += 1
        _ = sync.stabilize()
        
    except KeyboardInterrupt:
        print('Closing')

    finally:
        print('Done')
        cam.close() 
        