from multiprocessing import Process, Queue
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

class VisionMultiCore:
    def __init__(self, q, desiredWidth, desiredHeight, desiredFPS, src):
        # The queue
        self.q = q

        # Processing parameters
        self.isReceivingPose = False
        self.isRunPose = True
        self.poseProcess = None

        # Frame
        self.frame = None
        self.frameCount = 0
        self.poseCount = 0

        # Camera config
        self.cameraIdx     = src
        self.desiredWidth  = desiredWidth
        self.desiredHeight = desiredHeight
        self.desiredFPS    = desiredFPS      
        
        # Camera calibration matrix 
        self.mtx = np.array([[904.1343822,   0.0,            650.03003509],
                             [0.0,           903.58516942,   352.54361217],
                             [0.0,           0.0,            1.0         ]])
        self.dist = np.array([[0.08141624, -0.19751567, -0.00318761, 0.01176431, 0.16102671]])

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

    def startPoseProcess(self):
        # Create a process 
        if self.poseProcess == None:
            self.poseProcess = Process(target=self.processFrame)
            self.poseProcess.start()
            print('Computer vision process start')

            # Block till we start receiving values
            # while (self.isReceivingPose != True):
            #     time.sleep(0.1)

    def processFrame(self):
        # Aruco dictionary to be used and pose processing parameters
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.parm = aruco.DetectorParameters_create()
        self.parm.adaptiveThreshConstant = 10

        # Start the connection to the camera
        try:
            # self.cam = cv2.VideoCapture(self.cameraIdx, cv2.CAP_V4L)
            # self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            self.cam = cv2.VideoCapture(self.cameraIdx)
            self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.desiredWidth)
            self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.desiredHeight)
            self.cam.set(cv2.CAP_PROP_FPS, self.desiredFPS)
            self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            print('Camera start')
        except:
            print('Camera setup failed')

        # Process data until closed
        while(self.isRunPose):
            # Capture a frame
            _, self.frame = self.cam.read()
            self.frameCount += 1

            # Process the frame
            self.getPose()

            # Add data to the queue
            dataOut = [self.North, self.East, self.Down, self.Yaw]
            self.q.put(dataOut)

            # # Update receiving state
            # self.isReceivingPose = True

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

                # Only count processed frames
                self.poseCount += 1
            else:
                pass

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
        self.poseProcess.join()
        print('\nComputer vision process closed')

        # Rlease the camera connection
        self.cam.release()
        print('Camera closed')
