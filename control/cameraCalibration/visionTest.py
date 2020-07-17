from threading import Thread
import cv2.aruco as aruco
import numpy as np
import pickle
import time
import cv2

class VisionTest:
    def __init__(self):
        # Capture threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None
        self.frame = None

        # Performance parameters
        self.frameCount = 0
        self.poseCount = 0
        self.loopCount = 0
        self.frameStartTime = None
        self.startTime = None
        self.endTime = None

        # Aruco dictionary to be used and pose processing parameters
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.adaptiveThreshConstant = 10

        # Board properties
        self.lengthMarker = 19.3
        self.spacing = 9.7

        # Initial conditions for pose calculation 
        self.rvec = None
        self.tvec = None

        # Create the grid board
        self.board = aruco.GridBoard_create(
            markersX=4,                      # Columns
            markersY=3,                      # Rows
            markerLength=19.3,               # cm
            markerSeparation=9.7,            # cm
            dictionary=self.arucoDict)
        
        # Get calibration data
        try:
            file = open('resources/calibration.pckl', 'rb')
            self.mtx, self.dist = pickle.load(file)
            file.close()
        except:
            print('Calibration not found!')

    def startCamera(self, desiredWidth=1280, desiredHeight=720, desiredFPS=30, src=0):
        # Camera config     
        try:
            self.cam = cv2.VideoCapture(src, cv2.CAP_V4L)
            self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, desiredWidth)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desiredHeight)
            self.cam.set(cv2.CAP_PROP_FPS, desiredFPS)
            self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            self.cam.set(cv2.CAP_PROP_FOCUS, 20)
            self.cam.set(cv2.CAP_PROP_ZOOM, 0)
            self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            print('Camera start')
        except:
            print('Camera setup failed')

    def startFrameThread(self):
        # Create a thread
        if self.frameThread == None:
            self.frameThread = Thread(target=self.acquireFrame)
            self.frameThread.start()
            print('Capture thread start')

            # Block till we start receiving values
            while self.isReceivingFrame != True:
                time.sleep(0.1)

            # Start the timer 
            self.frameStartTime = time.time()

    def acquireFrame(self):
        # Acquire until closed
        while(self.isRunFrame):
            _, self.frame = self.cam.read()
            self.frameCount += 1
            self.isReceivingFrame = True

    def run(self):
        # Start the connection to the camera and start threading
        self.startCamera()
        self.startFrameThread()

        # Start the performance metrics
        self.startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # display the resulting frame
                cv2.imshow('Frame', self.frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Increment the counter 
                self.loopCount += 1
        except KeyboardInterrupt:
            pass
            
        # End performance metrics 
        self.endTime = time.time()

        # Close everything down
        self.close()

    def close(self):
        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('\nCamera thread closed')

        # Camera closed
        self.cam.release()
        print('Camera closed\n')

        # Print the results
        print('Frame rate: ', round(self.frameCount / (self.endTime - self.frameStartTime),2))
        print('Loop rate: ', round(self.loopCount / (self.endTime - self.startTime),2))
        print('Pose rate: ', round(self.poseCount / (self.endTime - self.startTime),2))

def main():    
    # Initialize class
    vt = VisionTest()
    vt.run()

# Main loop
if __name__ == '__main__':
    main()
