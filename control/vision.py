from filter import KalmanFilterNxN, TimeSync
from threading import Thread
import cv2.aruco as aruco
from T265 import T265
import numpy as np
import queue
import math
import time
import cv2

class Vision:
    def __init__(self):
        # Performance parameters
        self.startTime = None
        self.endTime   = None
        self.counter   = 0

        # Camera calibration matrices 
        self.mtx1 = np.array([[284.65527842,  0.0,            420.10118496],
                        [0.0,           285.43891088,   403.82029423],
                        [0.0,           0.0,            1.0         ]])
        self.dist1 = np.array([[-0.01159942, 0.00393409, 0.00059457, -0.0002535, -0.0006091]])

        self.mtx2 = np.array([[287.8954394,  0.0,            418.40412543],
                        [0.0,           287.99235758,   410.12408383],
                        [0.0,           0.0,            1.0         ]])
        self.dist2 = np.array([[-0.00818909, 0.00187817, 0.00132013, -0.00018278, -0.00044735]])
        
        # Camera to body frame values in cm
        self.offset1 = [-2.3, -15.0, 0]
        self.offset2 = [4.1, -15.0, 0]
    
    def run(self, Q):
        # Start the connection to the T265
        cam = T265()

        # Start the threads
        VP1 = VisionPose(ID='1', mtx=self.mtx1, dist=self.dist1, offset=self.offset1)
        VP1.startThread(cam.Img1)
        
        VP2 = VisionPose(ID='2', mtx=self.mtx2, dist=self.dist2, offset=self.offset2)
        VP2.startThread(cam.Img2)
        
        # Kalman filter
        KF = KalmanFilterNxN(3.0, 5.0, 10.0)

        # Loop rate stabilization
        sync = TimeSync(1/35)

        # Start timers
        self.startTime = time.time()
        sync.startTimer()

        # Process data until closed
        try: 
            while(True):
                # Get data from T265
                VP1.updateImg(cam.Img1)
                VP2.updateImg(cam.Img2)
                yRate = cam.psiRate     # Deg/s
                vN  = cam.vz * -100.0   # Cm/s
                vE  = cam.vx *  100.0   # Cm/s
                vD  = cam.vy *  100.0   # Cm/s
                aN  = cam.az * -100.0   # Cm/s^2
                aE  = cam.ax *  100.0   # Cm/s^2
                aD  = cam.ay *  100.0   # Cm/s^2
                dt  = cam.dt            # s

                # Stabilize rate and give time for image to be processed
                _ = sync.stabilize()

                # Average the results between cameras
                nRaw = (VP1.N + VP2.N) / 2.0
                eRaw = (VP1.E + VP2.E) / 2.0
                dRaw = (VP1.D + VP2.D) / 2.0
                yRaw = (VP1.Y + VP2.Y) / 2.0
                
                # Find difference bettween cameras
                nDif = (VP1.N - VP2.N)
                eDif = (VP1.E - VP2.E)
                dDif = (VP1.D - VP2.D)
                yDif = (VP1.Y - VP2.Y)
                
                # Estimate the kalman filter
                N, E, D, Y = KF.update(dt, np.array([nRaw, vN, 
                                                     eRaw, vE,
                                                     dRaw, vD,
                                                     yRaw, psiRate]).T)

                # Add data to the queue                
                Q.put([N, vN, aN, nRaw, nDif,
                       E, vE, aE, eRaw, eDif,
                       D, vD, aD, dRaw, dDif,
                       Y, yRate,  yRaw, yDif])

                # Increment the counter 
                self.counter += 1

        except KeyboardInterrupt:
            # Prep to close
            self.endTime = time.time()
            print('Closing Vision!')
        
        finally: 
            # Close all the threads (small delay for printing results)
            time.sleep(0.5)
            cam.close()
            VP1.close()
            VP2.close()
            
            # Performance of main thread
            print('\nMaster vision thread rate: ', round(self.counter / (self.endTime - self.startTime),1))

class VisionPose:
    def __init__(self, ID, mtx, dist, offset, lengthMarker=14.15, spacing=21.6):        
        # ID
        self.ID = ID
        
        # Camera parameters (intrinsic and extrensic)
        self.mtx = mtx
        self.dist = dist
        self.offset = offset
        
        # Initial conditions for pose calculation 
        self.rvec = None
        self.tvec = None
        
        # Threading parameters
        self.isReceiving = False
        self.isRun = True
        self.threadX = None

        # Performance
        self.startTime = None
        self.endTime = None
        self.loopCounter = 0
        self.poseCounter = 0
        
        # Aruco dictionary and parameter to be used for pose processing
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parm.cornerRefinementWinSize = 5
        self.parm.cornerRefinementMaxIterations = 50
        self.parm.cornerRefinementMinAccuracy = 0.01
        self.parm.adaptiveThreshWinSizeStep = 15
        self.parm.minMarkerPerimeterRate = 0.05

        # Create the ArUco board
        self.board = aruco.GridBoard_create(
            markersX=3,                 # Columns
            markersY=1,                 # Rows
            markerLength=lengthMarker,  # cm
            markerSeparation=spacing,   # cm
            dictionary=self.arucoDict)

        # Output variables: Position of body frame wrt ArUco frame (observing from above)
        #   North (negative when UAV is behind the target)
        #   East  (negative when UAV is to the left of the target)
        #   Down  (negative when UAV is below the target -> always positive)
        #   Yaw   (Positive counter clockwise viewing UAV from top)
        self.N = 0
        self.E = 0
        self.D = 0
        self.Y = 0

    def startThread(self, img):
        # Initialize an image before start
        self.gray = img
        
        # Create a thread
        if self.threadX == None:
            self.threadX = Thread(target=self.run)
            self.threadX.start()
            print('Thread ' + self.ID + ' start.')

            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

            # Start the timer and reset counters
            self.loopCounter = 0
            self.poseCounter = 0
            self.startTime = time.time()
    
    def run(self):
        # Run until thread is closed
        while(self.isRun):
            self.getPose()
            self.loopCounter += 1
            self.isReceiving = True
        
        # Record end time 
        self.endTime = time.time()
        
    def updateImg(self, img):
        self.gray = img
        
    def getPose(self):
        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(image=self.gray, dictionary=self.arucoDict, 
                                              parameters=self.parm, cameraMatrix=self.mtx, 
                                              distCoeff=self.dist)

        # Only estimate the pose if a marker was found
        if np.all(ids != None):
            _, self.rvec, self.tvec = aruco.estimatePoseBoard(corners=corners, ids=ids, board=self.board, 
                                                              cameraMatrix=self.mtx, distCoeffs=self.dist, 
                                                              rvec=self.rvec, tvec=self.tvec)
            
            # Convert from vector to rotation matrix and then transform to body frame
            R, _ = cv2.Rodrigues(self.rvec)
            R, t = self.transform2Body(R, self.tvec)

            # Get yaw
            _, _, yaw = self.rotationMatrix2EulerAngles(R)

            # Save values
            self.N = t[1] 
            self.E = t[0]
            self.D = t[2]
            self.Y = yaw

            # Increment pose counter 
            self.poseCounter += 1
        
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
        Tbc = np.array([[1,  0,  0,  self.offset[0]],
                        [0,  1,  0,  self.offset[1]],
                        [0,  0,  1,  self.offset[2]],
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

    def close(self):
        # Close the thread
        self.isRun = False
        self.threadX.join()
        print('Thread ' + self.ID + ' closed.')
        
        # Print performance
        print('  Loop rate (' + self.ID + '): ', round(self.loopCounter / (self.endTime - self.startTime),1))
        print('  Pose rate (' + self.ID + '): ', round(self.poseCounter / (self.endTime - self.startTime),1))

class VisionQueueThread:
    def __init__(self, Q):
        # Vision queue
        self.Q = Q
        
        # Data list
        self.qList = []
        
        # Threading parameters
        self.isReceiving = False
        self.thread = None

        # Start thread automatically
        self.startThread()
        
    def startThread(self):        
        # Create a thread
        if self.thread == None:
            self.thread = Thread(target=self.run, daemon=True)
            self.thread.start()
            print('Vision queue thread start')

            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)
    
    def run(self):
        # Run until main thread is terminated
        while(True):
            # Vision Data
            try:
                self.qList = self.Q.get(timeout=2)
            except queue.Empty:
                time.sleep(1/30)

            # Update thread state
            self.isReceiving = True
        
    def getList(self):
        # Return results
        return self.qList.copy()

class VisionData:
    def __init__(self, Q):
        # Start the queue thread
        self.VQT = VisionQueueThread(Q)
        
        # Create C like structures
        self.N = self.North()
        self.E = self.East()
        self.D = self.Down()
        self.Y = self.Yaw()
    
    def update(self):
        # Extract data from list into a useful variable
        temp = self.VQT.getList()
        
        self.N.Pos = temp[0]
        self.N.Vel = temp[1]
        self.N.Acc = temp[2]
        self.N.Raw = temp[3]
        self.N.Dif = temp[4]

        self.E.Pos = temp[5]
        self.E.Vel = temp[6]
        self.E.Acc = temp[7]
        self.E.Raw = temp[8]
        self.E.Dif = temp[9]
        
        self.D.Pos = temp[10]
        self.D.Vel = temp[11]
        self.D.Acc = temp[12]
        self.D.Raw = temp[13]
        self.D.Dif = temp[14]
                
        self.Y.Ang = temp[15]
        self.Y.Vel = temp[16]
        self.Y.Raw = temp[17]
        self.Y.Dif = temp[18]
                            
    class North:
        def __init__(self):
            self.Pos = None 
            self.Vel = None 
            self.Acc = None
            self.Raw = None
            self.Dif = None 
            
    class East:
        def __init__(self):
            self.Pos = None 
            self.Vel = None 
            self.Acc = None
            self.Raw = None
            self.Dif = None          
    
    class Down:
        def __init__(self):
            self.Pos = None 
            self.Vel = None 
            self.Acc = None
            self.Raw = None
            self.Dif = None 
    
    class Yaw:
        def __init__(self):
            self.Ang = None
            self.Vel = None
            self.Raw = None
            self.Dif = None
            