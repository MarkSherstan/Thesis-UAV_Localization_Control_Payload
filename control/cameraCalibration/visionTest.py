from threading import Thread
import cv2.aruco as aruco
import numpy as np
import pickle
import time
import cv2

class VisionTest:
    def __init__(self):
        # Create custom dictionary (# markers, # bits)
        self.arucoDict = aruco.custom_dictionary(17, 3)

        # Capture threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None
        self.frame = None

        # Performance parameters
        self.frameCount = 0
        self.poseCount = 0
        self.frameStartTime = None

        # Aruco dictionary to be used and pose processing parameters
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.adaptiveThreshConstant = 10

        # Camera calibration matrix 
        self.mtx = None
        self.dist = None

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

        # Calibration directories
        self.calibrationDir = 'calibrationImgs/'
        self.imgExtension = '.jpg'

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
        # Get calibration data
        try:
            self.getCalibration()
        except:
            print('Calibration not found!')

        # Start the connection to the camera
        self.startCamera()

        # Start the capture thread
        self.startFrameThread()

        # Start the performance metrics
        counter = 0
        startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # # Process a frame
                # self.getPose()

                # display the resulting frame
                cv2.imshow('Frame', self.frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Increment the counter 
                counter += 1
        except KeyboardInterrupt:
            pass
            
        # End performance metrics 
        endTime = time.time()

        # Close the capture thread and camera 
        self.close()
        self.cam.release()

        # Release the camera connection
        print('Camera closed')
        print('Vision loop rate: ', round(counter / (endTime - startTime),2))
        print('Pose rate: ', round(self.poseCount / (endTime - startTime),2))

    def getCalibration(self):
        # Open file, retrieve variables, and close
        file = open('resources/calibration.pckl', 'rb')
        self.mtx, self.dist = pickle.load(file)
        file.close()

    def close(self):
        # Print the results
        print('Frame rate: ', round(self.frameCount / (time.time() - self.frameStartTime),2))

        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('Camera thread closed')

def main():    
    # Initialize class
    vt = VisionTest()
    vt.run()


# Main loop
if __name__ == '__main__':
    main()
