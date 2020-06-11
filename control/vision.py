from threading import Thread
import cv2.aruco as aruco
import numpy as np
import math
import time
import cv2

class Vision:
    def __init__(self, desiredWidth=1280, desiredHeight=720, desiredFPS=30, cameraIdx=0):
        # Camera config
        self.desiredWidth  = desiredWidth
        self.desiredHeight = desiredHeight
        self.desiredFPS    = desiredFPS
        self.cameraIdx     = cameraIdx      
        
        # Capture threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None
        self.frame = None

        # Performance parameters
        self.frameCount = 0
        self.frameStartTime = None

        # Camera calibration matrix 
        self.mtx = np.array([[904.1343822,   0.0,            650.03003509],
                             [0.0,           903.58516942,   352.54361217],
                             [0.0,           0.0,            1.0         ]])
        self.dist = np.array([[0.08141624, -0.19751567, -0.00318761, 0.01176431, 0.16102671]])

        # Marker properties
        self.lengthMarker = 24.7
        self.markerID = 17

        # Offset values in cm
        self.offsetNorth = 0
        self.offsetEast  = 0
        self.offsetDown  = 0

        # Output variables 
        self.North = 0
        self.East  = 0
        self.Down  = 0 
        self.Yaw   = 0 

    def startFrameThread(self, cam):
        # Create a thread
        if self.frameThread == None:
            self.frameThread = Thread(target=self.acquireFrame, args=(cam,))
            self.frameThread.start()
            print('Capture thread start')

            # Block till we start receiving values
            while self.isReceivingFrame != True:
                time.sleep(0.1)

            # Start the timer 
            self.frameStartTime = time.time()

    def acquireFrame(self, cam):
        # Acquire until closed
        while(self.isRunFrame):
            _, self.frame = cam.read()
            self.frameCount += 1
            self.isReceivingFrame = True

    def processFrame(self, q):
        # Aruco dictionary to be used and pose processing parameters
        self.arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
        self.parm = aruco.DetectorParameters_create()
        self.parm.adaptiveThreshConstant = 10

        # Start the connection to the camera
        try:
            cam = cv2.VideoCapture(0, cv2.CAP_V4L)
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.desiredWidth)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.desiredHeight)
            cam.set(cv2.CAP_PROP_FPS, self.desiredFPS)
            cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            print('Camera start')
        except:
            print('Camera setup failed')

        # Start the capture thread
        self.startFrameThread(cam)

        # Start the performance metrics
        counter = 0
        startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # Process a frame
                self.getPose()

                # Add data to the queue
                q.put([self.North, self.East, self.Down, self.Yaw])

                # Increment the counter 
                counter += 1
        except KeyboardInterrupt:
            pass
            
        # End performance metrics 
        endTime = time.time()

        # Close the capture thread and camera 
        self.close()
        cam.release()

        # Release the camera connection
        print('Camera closed')
        print('Performance rate: ', round(counter / (endTime - startTime),2))

    def getPose(self):
        # Convert frame to gray and rotate to normal
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.rotate(gray, cv2.ROTATE_180)

        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=self.parm)

        # Only continue if a marker was found and is the correct ID
        if np.all(ids != None):
            if (len(ids) != 1):
                pass
            elif (ids[0][0] == self.markerID):
                # Estimate the pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.lengthMarker, self.mtx, self.dist)

                # Convert from vector to rotation matrix and then transform to body frame
                R, _ = cv2.Rodrigues(rvec)
                R, t = self.transform2Body(R, tvec)

                # Get yaw
                _, _, yaw = self.rotationMatrix2EulerAngles(R)

                # Save values
                self.North = t[0] 
                self.East  = t[1]
                self.Down  = t[2]
                self.Yaw   = -(yaw - 90)
            else:
                pass

    def isRotationMatrix(self, R):
        # Checks if matrix is valid
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def rotationMatrix2EulerAngles(self, R):
        try:
            # Check if rotation matrix is valid
            assert(self.isRotationMatrix(R))

            # Dont rotate more than 45 degrees in any direction and we will not get gimbal lock / singularities
            roll  = math.degrees(-math.asin(R[2,0]))
            pitch = math.degrees(math.atan2(R[2,1], R[2,2]))
            yaw   = math.degrees(math.atan2(R[1,0], R[0,0]))
            
            # Return results
            return roll, pitch, yaw
        except:
            # Return 0's upon failure
            print('Not a rotation matrix')
            return 0, 0, 0

    def transform2Body(self, R, t):
        # Original (ArUco wrt camera)
        Tca = np.append(R, np.transpose(t), axis=1)
        Tca = np.append(Tca, np.array([[0, 0, 0, 1]]), axis=0)

        # Transformation (camera wrt drone)
        Tbc = np.array([[0,  0,  1,  self.offsetNorth],
                        [1,  0,  0,  self.offsetEast],
                        [0,  1,  0,  self.offsetDown],
                        [0,  0,  0,   1]])

        # Resultant pose (ArUco wrt drone)
        Tba = np.dot(Tbc, Tca)

        # Return results
        R = Tba[0:3,0:3]
        t = Tba[0:3,3]

        # Return reults 
        return R, t

    def close(self):
        # Print the results
        print('Frame rate: ', round(self.frameCount / (time.time() - self.frameStartTime),2))

        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('Camera thread closed')
