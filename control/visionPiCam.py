from threading import Thread
from picamera.array import PiRGBArray
from picamera import PiCamera
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

class VisionPiCam:
    def __init__(self, desiredWidth, desiredHeight, desiredFPS):
        # Threading parameters
        self.isReceivingFrame = False
        self.isReceivingPose = False

        self.isRunFrame = True
        self.isRunPose = True

        self.frameThread = None
        self.poseThread = None

        # Frame
        self.frame = None
        self.frameCount = 0
        self.poseCount = 0

        # Camera config 
        self.desiredWidth  = desiredWidth
        self.desiredHeight = desiredHeight
        self.desiredFPS    = desiredFPS     
        
        # Aruco dictionary to be used and pose processing parameters
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.parm = aruco.DetectorParameters_create()
        self.parm.adaptiveThreshConstant = 10

        # Calibration values  
        # # C270: 640x480
        # self.mtx = np.array([[821.08729420, 0.0000000000, 329.75642931],
        #                      [0.0000000000, 820.81370137, 235.77898949],
        #                      [0.0000000000, 0.000000e000, 1.0000000000]])
        # self.dist = np.array([[1.23116014e-01, -1.32202826e+00, -1.37712626e-03, 5.69880008e-03, 7.08258838e+00]])
        
        # # C270: 1280x720
        # self.mtx = np.array([[1.39422292e+03, 0.00000000e+00, 6.42110040e+02],
        #                      [0.00000000e+00, 1.39396841e+03, 3.03067657e+02],
        #                      [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        # self.dist = np.array([[0.03632819, -0.28223148, -0.00964304, 0.00237123, 1.99747268]])
                
        # # C920: 640x480
        # self.mtx = np.array([[632.47602078, 0.0000000000, 322.06287257],
        #                      [0.0000000000, 634.79038355, 261.59013669],
        #                      [0.0000000000, 0.000000e000, 1.0000000000]])
        # self.dist = np.array([[1.36627983e-01, -1.46404481e+00,  6.49112090e-03, -8.68510828e-04, 4.91419701e+00]])
        
        # # C920: 1280x720
        # self.mtx = np.array([[949.45904321, 0.0000000000, 643.06473799],
        #                      [0.0000000000, 951.68543795, 389.81354980],
        #                      [0.0000000000, 0.000000e000, 1.0000000000]])
        # self.dist = np.array([[0.04976768, -0.28103135, 0.0062585, -0.00345049, 0.43938616]])

        # # C920: 1920x1080
        self.mtx = np.array([[1.43643180e+03, 0.00000000e+00, 9.55889925e+02],
                             [0.00000000e+00, 1.43772789e+03, 5.51182985e+02],
                             [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
        self.dist = np.array([[4.94485823e-02, -2.49097156e-01, -6.81702545e-05, -6.06313917e-03, 3.34638915e-01]])

        # Marker properties
        self.lengthMarker = 0.176
        self.markerID = 17

        # Offset values in cm
        self.offsetNorth = 0
        self.offsetEast  = 0
        self.offsetDown  = 0

        # Output values
        self.North = 0
        self.East  = 0
        self.Down  = 0
        self.Yaw   = 0
        self.isReady = False

        # Start the connection to the camera
        try:          
            # initialize the camera and grab a reference to the raw camera capture
            self.camera = PiCamera()
            self.camera.resolution = (self.desiredWidth, self.desiredHeight)
            self.camera.framerate = self.desiredFPS
            self.rawCapture = PiRGBArray(self.camera, size=(self.desiredWidth, self.desiredHeight))
            time.sleep(0.2)
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
        for image in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            # grab the raw NumPy array representing the image
            self.frame = image.array
            
            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)
            
            # Count the frames
            self.frameCount += 1
        
            # Threading flags and exit condition
            self.isReceivingFrame = True
            if self.isRunFrame is False:
                break

    def startPoseThread(self):
        # Block until a frame has been acquired
        while self.frame is None:
            time.sleep(0.1)

        # Create a thread
        if self.poseThread == None:
            self.poseThread = Thread(target=self.processFrame)
            self.poseThread.start()
            print('Frame processing thread start')

            # Block till we start receiving values
            while (self.isReceivingPose != True):
                time.sleep(0.1)

    def processFrame(self):
        # Process data until closed
        while(self.isRunPose):
            self.getPose()
            self.poseCount += 1
            self.isReceivingPose = True

    def getPose(self):
        # Get frame and convert to gray
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=self.parm)

        # Only continue if a marker was found and is the correct ID
        if np.all(ids != None):
            if (len(ids) != 1):
                pass
            elif (ids[0][0] == self.markerID):
                # Estimate the pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.lengthMarker, self.mtx, self.dist)

                # Save the distances
                self.North = (tvec[0][0][2]*100 - self.offsetNorth)
                self.East  = (tvec[0][0][0]*100 - self.offsetEast)
                self.Down  = (tvec[0][0][1]*100 - self.offsetDown)

                # Convert to rotation matrix and extract yaw
                R, _ = cv2.Rodrigues(rvec)
                eulerAngles = self.rotationMatrixToEulerAngles(R)
                self.Yaw = eulerAngles[1]
            else:
                pass

        # Change state for controller class
        self.isReady = True

    def isRotationMatrix(self, R):
        # Checks if matrix is valid
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrixToEulerAngles(self, R):
        # Check if rotation matrix is valid
        assert(self.isRotationMatrix(R))

        # Check if singular
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

        if (sy < 1e-6):
            # Singular
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0
        else:
            # Not singular
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])

        # Return roll, pitch, and yaw in some order
        return np.array([x, y, z])

    def close(self):
        # Close the pose processing thread
        self.isRunPose = False
        self.poseThread.join()
        print('\nFrame processing thread closed')

        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('Camera thread closed')
