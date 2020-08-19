from multiprocessing import Process, Queue
from threading import Thread
import cv2.aruco as aruco
from T265 import T265
import numpy as np
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
    
        # Connect to vision, create the queue, and start the core
        VP1 = VisionPose(ID='1', mtx=self.mtx1, dist=self.dist1, offset=self.offset1)
        self.Q1img  = Queue()
        self.Q1pose = Queue()
        P1 = Process(target=VP1.run, args=(self.Q1img, self.Q1pose, ))
        P1.start()
        
        VP2 = VisionPose(ID='2', mtx=self.mtx2, dist=self.dist2, offset=self.offset2)
        self.Q2img  = Queue()
        self.Q2pose = Queue()
        P2 = Process(target=VP2.run, args=(self.Q2img, self.Q2pose, ))
        P2.start()
        
    def run(self, Q):
        # Start the connection to the T265
        cam = T265()

        # Start the main thread timer
        self.startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # Get data from T265
                self.Q1img.put(cam.Img1)
                self.Q2img.put(cam.Img2)
                psiRate = cam.psiRate   # Deg/s
                vN  = cam.vz * -100.0   # Cm/s
                vE  = cam.vx *  100.0   # Cm/s
                vD  = cam.vy *  100.0   # Cm/s

                # Get vision data
                temp1 = self.Q1pose.get()
                temp2 = self.Q2pose.get()
                
                # Extract the vision data
                N1 = temp1[0];  N2 = temp2[0]
                E1 = temp1[1];  E2 = temp2[1]
                D1 = temp1[2];  D2 = temp2[2]
                Y1 = temp1[3];  Y2 = temp2[3]
                
                # Average the results between cameras
                North = (N1 + N2) / 2.0
                East  = (E1 + E2) / 2.0
                Down  = (D1 + D2) / 2.0
                Yaw   = (Y1 + Y2) / 2.0
                
                # Add data to the queue
                Q.put([North, East, Down, vN, vE, vD, Yaw, psiRate])
                time.sleep(1/30)

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
            
            # Performance of main thread
            print('\nMaster vision thread rate: ', round(self.counter / (self.endTime - self.startTime),1))

class VisionPose:
    def __init__(self, ID, mtx, dist, offset, lengthMarker=6.43, spacing=3.22):        
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
        self.endTimeImg = None
        self.loopCounter = 0
        self.poseCounter = 0
        self.imgCounter  = 0
        
        # Image 
        self.gray = None

        # Marker parameters
        self.lengthMarker = lengthMarker
        self.spacing = spacing

        # Output variables: Position of body frame wrt ArUco frame (observing from above)
        #   North (negative when UAV is behind the target)
        #   East  (negative when UAV is to the left of the target)
        #   Down  (negative when UAV is below the target -> always positive)
        #   Yaw   (Positive counter clockwise viewing UAV from top)

    def createArucoStructures(self):
        # Aruco dictionary and parameter to be used for pose processing
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parm.cornerRefinementWinSize = 5
        self.parm.cornerRefinementMaxIterations = 50
        self.parm.cornerRefinementMinAccuracy = 0.01
        # self.parm.adaptiveThreshWinSizeStep = 15
        self.parm.minMarkerPerimeterRate = 0.05

        # Create the ArUco board
        self.board = aruco.GridBoard_create(
            markersX=4,                 # Columns
            markersY=3,                 # Rows
            markerLength=self.lengthMarker,  # cm
            markerSeparation=self.spacing,   # cm
            dictionary=self.arucoDict)
        
    def startThread(self, Qimg):        
        # Create a thread
        if self.threadX == None:
            self.threadX = Thread(target=self.getImg, args=(Qimg, ))
            self.threadX.start()
            print('Thread ' + self.ID + ' start.')

            # Block till we start receiving values
            while self.isReceiving != True:
                time.sleep(0.1)

            # Start the timer and reset counters
            self.loopCounter = 0
            self.poseCounter = 0
            self.imgCounter  = 0
            self.startTime = time.time()
    
    def getImg(self, Qimg):
        # Run until thread is closed
        while(self.isRun):
            self.gray = Qimg.get(timeout=3)
            self.imgCounter += 1
            self.isReceiving = True
        
        # Record end time 
        self.endTimeImg = time.time()
        
    def run(self, Qimg, Qpose):
        # Create aruco parameters and board (not pickalable for multiprocessing)
        self.createArucoStructures()
        
        # Start the image queue thread
        self.startThread(Qimg)
        
        # Run forever 
        try: 
            while(True):
                # lists of ids and corners belonging to each id
                corners, ids, _ = aruco.detectMarkers(image=self.gray, dictionary=self.arucoDict, 
                                                    parameters=self.parm, cameraMatrix=self.mtx, 
                                                    distCoeff=self.dist)

                # Increment loop counter
                self.loopCounter += 1
                
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
                    N = t[1] 
                    E = t[0]
                    D = t[2]
                    Y = yaw

                    # Put data in queue
                    Qpose.put([N, E, D, Y])
                    
                    # Increment pose counter 
                    self.poseCounter += 1
        except:
            # Prep to close
            self.endTime = time.time()
            print('Closing ' + self.ID)
            
        finally:
            self.close()
            
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
        time.sleep(1)
        
        # Print performance
        print('  Loop rate (' + self.ID + '): ', round(self.loopCounter / (self.endTime - self.startTime),1))
        print('  Pose rate (' + self.ID + '): ', round(self.poseCounter / (self.endTime - self.startTime),1))
        # print(self.imgCounter, self.endTimeImg, self.startTime)
        print('  Img rate  (' + self.ID + '): ', round(self.imgCounter / (self.endTime - self.startTime),1))
