from threading import Thread
import time
import cv2

class threadCapture:
    def __init__(self, desiredWidth, desiredHeight, desiredFPS, autoFocus, src=0):
        # Threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None

        # Frame
        self.frame = None
        self.frameCount = 0

        # Camera config 
        self.desiredWidth  = desiredWidth
        self.desiredHeight = desiredHeight
        self.desiredFPS    = desiredFPS   
        self.autoFocus     = autoFocus    

        # Start the connection to the camera
        try:
            self.cam = cv2.VideoCapture(src)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.desiredWidth)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.desiredHeight)
            self.cam.set(cv2.CAP_PROP_FPS, self.desiredFPS)
            self.cam.set(cv2.CAP_PROP_AUTOFOCUS, self.autoFocus)
            print('Camera start')
        except:
            print('Camera setup failed')

    def startFrameThread(self):
        # Create a thread
        if self.frameThread == None:
            self.frameThread = Thread(target=self.acquireFrame)
            self.frameThread.start()
            print('Camera thread start')

            # Block till we start receiving values
            while self.isReceivingFrame != True:
                time.sleep(0.1)

    def acquireFrame(self):
        # Acquire until closed
        while(self.isRunFrame):
            _, self.frame = self.cam.read()
            self.frameCount += 1
            self.isReceivingFrame = True

    def close(self):
        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('Camera thread closed')

        # Release the camera connection
        self.cam.release()
        print('Camera closed')


def main():    
    # Set desired parameters
    desiredWidth  = 640     # 1920, 1280, 800, 640
    desiredHeight = 480     # 1080, 720,  600, 480
    desiredFPS    = 30
    autoFocus     = False
    showFrame     = False

    # Camera properties 
    v = threadCapture(desiredWidth, desiredHeight, desiredFPS, autoFocus)
    v.startFrameThread()

    # Counting variables
    startTime = time.time()
    loopCount = 0

    # Run test
    print('Test running for 10 secounds')
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
    print('FPS (true)\tD: ', desiredFPS, '\t\tA: ', round(v.frameCount / (endTime - startTime),2))
    print('FPS (threaded)\tD: ', desiredFPS, '\t\tA: ', round(loopCount / (endTime - startTime),2))

    # Close any windows if there are any
    v.close()
    cv2.destroyAllWindows() 
    
# Main loop
if __name__ == '__main__':
    main()
    