from multiprocessing import Queue
import queue
from VisionMultiCore import *
import time

def main():    
    # Set desired parameters
    desiredWidth  = [1280]
    desiredHeight = [720]
    desiredFPS    = 30

    # Create a queue
    q = Queue()

    for ii in range(len(desiredWidth)):
        # Camera properties 
        v = VisionMultiCore(q, desiredWidth[ii], desiredHeight[ii], desiredFPS, 0)
        v.startPoseProcess()
 
        # Block until there is data
        data = None
        while (data is None):
            try:
                data = q.get(False)  
            except queue.Empty:
                data = None
        
        # Counting variables
        startTime = time.time()

        # Run test
        print('Test running for 10 secounds\n')
        while (time.time() < startTime+10):
            
            # # Unpack the data
            data = q.get()

            print(time.time()-startTime, data)
            # print('N: %0.2f E: %0.2f D: %0.2f Y: %0.2f' % (dataOut[0], dataOut[1, dataOut[2], dataOut[3]]))
            time.sleep(0.5)
            
            # Increment timer 
            # loopCount += 1

        print("end")

        # Record final time and frame size            
        # endTime = time.time()
        # actualHeight, actualWidth, _ = v.frame.shape 

        # Print results
        # print('Frame Width\tD: ', desiredWidth[ii], '\tA: ', actualWidth)
        # print('Frame Height\tD: ', desiredHeight[ii], '\tA: ', actualHeight)
        # print('Frame rate\tD: ', desiredFPS, '\t\tA: ', round(v.frameCount / (endTime - startTime),2))
        # print('Pose rate\tD: ', desiredFPS, '\t\tA: ', round(v.poseCount / (endTime - startTime),2))

        # Close process and camera connection
        v.close()
        
        print("yea the thread did not close")
        # Small pause before next resolution test
        time.sleep(1)
    
# Main loop
if __name__ == '__main__':
    main()
    
