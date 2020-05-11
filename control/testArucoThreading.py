from vision import *
import time

def main():    
    # Set desired parameters
    desiredWidth  = 640     # 1920, 1280, 800, 640
    desiredHeight = 480     # 1080, 720,  600, 480
    desiredFPS    = 30
    autoFocus     = False
    showFrame     = False

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
        loopCount += 1
        if showFrame is True:
            cv2.imshow('Frame', v.frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Record final time and frame size            
    endTime = time.time()
    actualHeight, actualWidth, _ = v.frame.shape 

    # Print results
    print('Frame Width\tD: ', desiredWidth, '\tA: ', actualWidth)
    print('Frame Height\tD: ', desiredHeight, '\tA: ', actualHeight)
    print('Frame rate\tD: ', desiredFPS, '\t\tA: ', round(v.frameCount / (endTime - startTime),2))
    print('Pose rate\tD: ', desiredFPS, '\t\tA: ', round(v.poseCount / (endTime - startTime),2))
    print('Main process\tD: ', desiredFPS, '\t\tA: ', round(loopCount / (endTime - startTime),2))

    # Close any windows if there are any
    v.close()
    cv2.destroyAllWindows() 
    
# Main loop
if __name__ == '__main__':
    main()
    