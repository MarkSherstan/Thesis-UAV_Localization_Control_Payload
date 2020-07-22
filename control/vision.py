from threading import Thread
import cv2.aruco as aruco
import numpy as np
import math
import time
import cv2

class Vision:
    def __init__(self, lengthMarker=4.50, spacing=2.25):
        # Board properties
        self.lengthMarker = lengthMarker
        self.spacing = spacing

        # Camera config
        self.desiredWidth  = 1280
        self.desiredHeight = 720
        self.desiredFPS    = 30
        self.cameraIdx     = 0      
        
        # Capture threading parameters
        self.isReceivingFrame = False
        self.isRunFrame = True
        self.frameThread = None
        self.frame = None

        # Performance parameters
        self.frameCount = 0
        self.loopCount  = 0
        self.poseCount  = 0
        self.startTime  = None
        self.endTime    = None
        self.frameStartTime = None

        # Camera calibration matrix 
        self.mtx = np.array([[915.03603689,  0.0,            665.08947886],
                             [0.0,           913.98237919,   353.51979348],
                             [0.0,           0.0,            1.0         ]])
        self.dist = np.array([[0.08750648, -0.17636763, -0.00021177, 0.00591364, 0.04690385]])

        # Initial conditions for pose calculation 
        self.rvec = None
        self.tvec = None
        
        # Camera to body frame values in cm
        self.offsetNorth = 0
        self.offsetEast  = 0
        self.offsetDown  = 0

        # Output variables: Position of body frame wrt ArUco frame converted to UAV NED (observing from above)
        # Where the marker is from the UAV
        #   North (negative when UAV is behind the target)
        #   East  (negative when UAV is to the right of the target)
        #   Down  (negative when UAV is below the target -> always positive)
        #   Yaw   (Positive clockwise viewing UAV from top)
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
        # Aruco dictionary and parameter to be used for pose processing
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.minMarkerPerimeterRate = 0.1
        self.parm.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parm.cornerRefinementWinSize = 5
        self.parm.cornerRefinementMaxIterations = 100
        self.parm.cornerRefinementMinAccuracy = 0.00001

        # Create the board
        self.board = aruco.GridBoard_create(
            markersX=4,                      # Columns
            markersY=3,                      # Rows
            markerLength=self.lengthMarker,  # cm
            markerSeparation=self.spacing,   # cm
            dictionary=self.arucoDict)
        
        # Start the connection to the camera
        try:
            cam = cv2.VideoCapture(0, cv2.CAP_V4L)
            cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
            cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.desiredWidth)
            cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.desiredHeight)
            cam.set(cv2.CAP_PROP_FPS, self.desiredFPS)
            cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            cam.set(cv2.CAP_PROP_FOCUS, 0)
            cam.set(cv2.CAP_PROP_ZOOM, 0)
            print('Camera start')
        except:
            print('Camera setup failed')

        # Start the capture thread
        self.startFrameThread(cam)

        # Start the performance metrics
        self.startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # Process a frame
                self.getPose()

                # Add data to the queue
                q.put([self.North, self.East, self.Down, self.Yaw])

                # Increment the counter 
                self.loopCount += 1
        except KeyboardInterrupt:
            pass
            
        # End performance metrics 
        self.endTime = time.time()

        # Close the capture thread and camera, post perfromance metrics
        self.close(cam)
 
    def getPose(self):
        # Convert frame to gray and rotate to normal
        gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict, parameters=self.parm, cameraMatrix=self.mtx, distCoeff=self.dist)

        # Only continue if a marker was found
        if np.all(ids != None):
            # Estimate the pose
            _, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.mtx, self.dist, self.rvec, self.tvec)

            # Convert from vector to rotation matrix and then transform to body frame
            R, _ = cv2.Rodrigues(rvec)
            R, t = self.transform2Body(R, tvec)

            # Get yaw
            _, _, yaw = self.rotationMatrix2EulerAngles(R)

            # Save values
            self.North =  t[1] 
            self.East  = -t[0]
            self.Down  =  t[2]
            self.Yaw   = -yaw

            # Increment counter
            self.poseCount += 1
            
            # Save translation and rotation for next iteration 
            self.rvec = rvec
            self.tvec = tvec
        else:
            self.North = np.nan 
            self.East  = np.nan
            self.Down  = np.nan
            self.Yaw   = np.nan

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
            x = math.degrees(math.atan2(R[2,1], R[2,2]))
            y = math.degrees(-math.asin(R[2,0]))
            z = math.degrees(math.atan2(R[1,0], R[0,0]))
            
            # Return results
            return x, y, z
        except:
            # Return 0's upon failure
            print('Not a rotation matrix')
            return 0, 0, 0

    def transform2Body(self, R, t):
        # Original (ArUco wrt camera)
        Tca = np.append(R, t, axis=1)
        Tca = np.append(Tca, np.array([[0, 0, 0, 1]]), axis=0)

        # Transformation (camera wrt drone body frame)
        Tbc = np.array([[1,  0,  0,  self.offsetEast],
                        [0,  1,  0,  self.offsetDown],
                        [0,  0,  1,  self.offsetNorth],
                        [0,  0,  0,  1]])

        # Resultant pose (ArUco wrt drone body frame)
        Tba = np.dot(Tbc, Tca)
        R = Tba[0:3,0:3]
        t = Tba[0:3,3]

        # Drone body frame wrt ArUco
        R = np.transpose(R)
        t = np.dot(-R, t)

        # Return reults 
        return R, t

    def close(self, cam):
        # Close the capture thread
        self.isRunFrame = False
        self.frameThread.join()
        print('Camera thread closed')

        # Camera closed
        cam.release()
        print('Camera closed\n')

        # Print the results
        print('Frame rate: ', round(self.frameCount / (self.endTime - self.frameStartTime),1))
        print('Pose rate: ', round(self.poseCount / (self.endTime - self.startTime),1))
        print('Loop rate: ', round(self.loopCount / (self.endTime - self.startTime),1))
