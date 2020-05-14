from vision import *
import time

def main():    
    # Set desired parameters
    desiredWidth  = 1280     # 1920, 1280, 1280, 640
    desiredHeight = 960      # 1080, 720,  960, 480
    desiredFPS    = 30
    autoFocus     = False

    # Camera properties 
    v = Vision(desiredWidth, desiredHeight, desiredFPS, autoFocus)
    v.startFrameThread()
    v.startPoseThread()
 
    # Counting variables
    startTime = time.time()
    loopCount = 0

    # Run test
    print('Test running for 10 secounds\n')
    while (time.time() < startTime+10):
        # Print pose info
        print('N: %0.2f E: %0.2f D: %0.2f Y: %0.2f' % (v.North, v.East, v.Down, v.Yaw))
        time.sleep(0.2)
        
        # Increment timer 
        loopCount += 1

    # Record final time and frame size            
    endTime = time.time()
    actualHeight, actualWidth, _ = v.frame.shape 

    # Print results
    print('Frame Width\tD: ', desiredWidth, '\tA: ', actualWidth)
    print('Frame Height\tD: ', desiredHeight, '\tA: ', actualHeight)
    print('Frame rate\tD: ', desiredFPS, '\t\tA: ', round(v.frameCount / (endTime - startTime),2))
    print('Pose rate\tD: ', desiredFPS, '\t\tA: ', round(v.poseCount / (endTime - startTime),2))
    print('Main process\tD: ', desiredFPS, '\t\tA: ', round(loopCount / (endTime - startTime),2))

    # Close threads and camera connection
    v.close()
    
# Main loop
if __name__ == '__main__':
    main()
    
