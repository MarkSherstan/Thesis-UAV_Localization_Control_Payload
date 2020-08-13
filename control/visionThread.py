from multiprocessing import Queue
from threading import Thread
import cv2.aruco as aruco
from T265 import T265
import numpy as np
import math
import time
import cv2

class Vision:
    def __init__(self, lengthMarker=6.43, spacing=3.22):
        # Board properties
        self.lengthMarker = lengthMarker
        self.spacing = spacing
        
        # Capture threading parameters
        self.isReceivingPose = False
        self.isRunPose = True
        self.poseThread = None
        self.poseThreadCount = 0
        self.poseStartTime = None
        self.poseEndTime = None

        # Performance parameters
        self.loopCount  = 0
        self.poseCount  = 0
        self.startTime  = None
        self.endTime    = None

        # Camera calibration matrices 
        self.mtx1 = np.array([[284.65527842,  0.0,            420.10118496],
                        [0.0,           285.43891088,   403.82029423],
                        [0.0,           0.0,            1.0         ]])
        self.dist1 = np.array([[-0.01159942, 0.00393409, 0.00059457, -0.0002535, -0.0006091]])

        self.mtx2 = np.array([[287.8954394,  0.0,            418.40412543],
                        [0.0,           287.99235758,   410.12408383],
                        [0.0,           0.0,            1.0         ]])
        self.dist2 = np.array([[-0.00818909, 0.00187817, 0.00132013, -0.00018278, -0.00044735]])

        # Initial conditions for pose calculation 
        self.rvec1 = None;  self.rvec2 = None
        self.tvec1 = None;  self.tvec2 = None
        
        # Camera to body frame values in cm
        self.offset1 = [-2.3, -15.0, 0]
        self.offset2 = [4.1, -15.0, 0]
        
        # Output variables: Position of body frame wrt ArUco frame converted to UAV NED (observing from above)
        # Where the UAV is relative to marker following ArUco coords
        #   North (negative when UAV is behind the target)
        #   East  (negative when UAV is to the left of the target)
        #   Down  (negative when UAV is below the target -> always positive)
        #   Yaw   (Positive counter clockwise viewing UAV from top)
        self.N1 = 0
        self.E1 = 0
        self.D1 = 0
        self.Y1 = 0

    def startPoseThread(self, cam):
        # Create a thread
        if self.poseThread == None:
            self.poseThread = Thread(target=self.acquirePose, args=(cam, ))
            self.poseThread.start()
            print('Pose thread start')

            # Block till we start receiving values
            while self.isReceivingPose != True:
                time.sleep(0.1)

            # Start the timer 
            self.poseCount = 0
            self.poseStartTime = time.time()

    def acquirePose(self, cam):
        # Acquire until closed
        while(self.isRunPose):
            # Calculate pose
            self.N1, self.E1, self.D1, self.Y1, self.rvec1, self.tvec1 = self.getPose(cam.Img1, self.mtx1, self.dist1, self.rvec1, self.tvec1, self.offset1)

            # Performance and threading
            self.poseThreadCount += 1
            self.isReceivingPose = True

        self.poseEndTime = time.time()
        
    def processFrame(self, q):
        # Aruco dictionary and parameter to be used for pose processing
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parm.cornerRefinementWinSize = 5
        self.parm.cornerRefinementMaxIterations = 100
        self.parm.cornerRefinementMinAccuracy = 0.00001

        # Create the ArUco board
        self.board = aruco.GridBoard_create(
            markersX=4,                      # Columns
            markersY=3,                      # Rows
            markerLength=self.lengthMarker,  # cm
            markerSeparation=self.spacing,   # cm
            dictionary=self.arucoDict)
        
        # Start the connection to the T265
        cam = T265()

        # Start pose thread
        self.startPoseThread(cam)

        # Start the performance metrics
        self.startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # Get data from T265: Save local approximating a locked thread before heavy pose calcs
                gray1 = cam.Img1
                gray2 = cam.Img2
                psiRate = cam.psiRate   # Deg/s
                vN  = cam.vz * -100.0    # Cm/s
                vE  = cam.vx *  100.0    # Cm/s
                vD  = cam.vy *  100.0    # Cm/s

                # Process frames
                # N1, E1, D1, Y1, self.rvec1, self.tvec1 = self.getPose(gray1, self.mtx1, self.dist1, self.rvec1, self.tvec1, self.offset1)
                # N2, E2, D2, Y2, self.rvec2, self.tvec2 = self.getPose(gray2, self.mtx2, self.dist2, self.rvec2, self.tvec2, self.offset2)
                time.sleep(1/60)

                # Average the results between cameras
                North = 0 #(N1 + N2) / 2.0
                East  = 0 #(E1 + E2) / 2.0
                Down  = 0 #(D1 + D2) / 2.0
                Yaw   = 0 #(Y1 + Y2) / 2.0
                
                # Add data to the queue
                q.put([North, East, Down, vN, vE, vD, Yaw, psiRate])

                # Increment the counter 
                self.loopCount += 1

        except KeyboardInterrupt:
            pass
            
        # End performance metrics 
        self.endTime = time.time()

        # Close the capture thread and camera, post perfromance metrics
        self.close(cam)
 
    def getPose(self, gray, mtx, dist, rvec, tvec, offset):
        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict, parameters=self.parm, cameraMatrix=mtx, distCoeff=dist)

        # Only estimate the pose if a marker was found
        if np.all(ids != None):
            _, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, mtx, dist, rvec, tvec)
            
            # Increment counter
            self.poseCount += 1

        # Prevent error when no target is found (start up)
        if np.all(rvec == None) or np.all(tvec == None):
            return 0, 0, 0, 0, None, None

        # Convert from vector to rotation matrix and then transform to body frame
        R, _ = cv2.Rodrigues(rvec)
        R, t = self.transform2Body(R, tvec, offset)

        # Get yaw
        _, _, yaw = self.rotationMatrix2EulerAngles(R)

        # Save values
        North = t[1] 
        East  = t[0]
        Down  = t[2]
        Yaw   = yaw
        
        return North, East, Down, Yaw, rvec, tvec

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

    def transform2Body(self, R, t, offset):
        # Original (ArUco wrt camera)
        Tca = np.append(R, t, axis=1)
        Tca = np.append(Tca, np.array([[0, 0, 0, 1]]), axis=0)

        # Transformation (camera wrt drone body frame)
        Tbc = np.array([[1,  0,  0,  offset[0]],
                        [0,  1,  0,  offset[1]],
                        [0,  0,  1,  offset[2]],
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
        self.isRunPose = False
        self.poseThread.join()
        print('Pose thread closed')

        # Camera closed
        cam.close()

        # Print the results
        print('Pose rate: ', round((self.poseCount / 2) / (self.endTime - self.startTime),1))
        print('Loop rate: ', round(self.loopCount / (self.endTime - self.startTime),1))
        print('Pose rate thread: ', round((self.poseThreadCount) / (self.poseEndTime - self.poseStartTime),1))
