from vision import *
import time

def main():    
    # Set desired parameters
    desiredWidth  = [1280]
    desiredHeight = [720]
    desiredFPS    = 30

    for ii in range(len(desiredWidth)):
        # Camera properties 
        v = Vision(desiredWidth[ii], desiredHeight[ii], desiredFPS)
        v.startFrameThread()
        v.startPoseThread()
    
        # Counting variables
        startTime = time.time()
        v.frameCount = 0
        v.poseCount = 0
        loopCount = 0

        # Run test
        print('Test running for 10 secounds\n')
        while (time.time() < startTime+10):
            # Print pose info
            print('N: %0.2f E: %0.2f D: %0.2f Y: %0.2f' % (v.North, v.East, v.Down, v.Yaw))
            time.sleep(0.5)
            
            # Increment timer 
            loopCount += 1

        # Record final time and frame size            
        endTime = time.time()
        actualHeight, actualWidth, _ = v.frame.shape 

        # Print results
        print('Frame Width\tD: ', desiredWidth[ii], '\tA: ', actualWidth)
        print('Frame Height\tD: ', desiredHeight[ii], '\tA: ', actualHeight)
        print('Frame rate\tD: ', desiredFPS, '\t\tA: ', round(v.frameCount / (endTime - startTime),2))
        print('Pose rate\tD: ', desiredFPS, '\t\tA: ', round(v.poseCount / (endTime - startTime),2))

        # Close threads and camera connection
        v.close()
        
        # Small pause before next resolution test
        time.sleep(5)
    
# Main loop
if __name__ == '__main__':
    main()
    
