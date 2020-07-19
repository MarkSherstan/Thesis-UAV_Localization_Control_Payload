from threading import Thread
import cv2.aruco as aruco
import numpy as np
import pickle
import math
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
        self.loopCount = 0
        self.frameStartTime = None
        self.startTime = None
        self.endTime = None

        # Aruco dictionary to be used and pose processing parameters
        self.arucoDict = aruco.custom_dictionary(17, 3)
        self.parm = aruco.DetectorParameters_create()
        self.parm.adaptiveThreshConstant = 10

        # Initial conditions for pose calculation 
        self.rvec = None
        self.tvec = None

        # Create the grid board
        self.board = aruco.GridBoard_create(
            markersX=4,                      # Columns
            markersY=3,                      # Rows
            markerLength=4.5,                # cm
            markerSeparation=2.25,           # cm
            dictionary=self.arucoDict)
        
        # Get calibration data
        try:
            file = open('resources/calibration.pckl', 'rb')
            self.mtx, self.dist = pickle.load(file)
            file.close()
        except:
            print('Calibration not found!')

        # Formatting
        self.fontFace=cv2.FONT_HERSHEY_SIMPLEX
        self.fontScale=0.6
        self.colorA=(0,255,0)
        self.colorB=(255,0,0)
        self.colorC=(0,0,255)
        self.color0=(0,0,0)
        self.lineType=2

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

    def isRotationMatrix(self, R):
        # Checks if matrix is valid
        Rt = np.transpose(R)
        shouldBeIdentity = np.dot(Rt, R)
        I = np.identity(3, dtype = R.dtype)
        n = np.linalg.norm(I - shouldBeIdentity)
        return n < 1e-6

    def ArotationMatrix2EulerAngles(self, R):
        try:
            # Check if rotation matrix is valid
            assert(self.isRotationMatrix(R))

            # Dont rotate more than 45 degrees in any direction and we will not get gimbal lock / singularities
            roll = math.degrees(math.atan2(R[2,1], R[2,2]))
            pitch  = math.degrees(-math.asin(R[2,0]))
            yaw   = math.degrees(math.atan2(R[1,0], R[0,0]))
            
            # Return results
            return roll, pitch, yaw
        except:
            # Return 0's upon failure
            print('Not a rotation matrix')
            return 0, 0, 0

    def BrotationMatrix2EulerAngles(self, R):

        assert(self.isRotationMatrix(R))
        
        sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
        
        singular = sy < 1e-6

        if  not singular :
            x = math.atan2(R[2,1] , R[2,2])
            y = math.atan2(-R[2,0], sy)
            z = math.atan2(R[1,0], R[0,0])
        else :
            x = math.atan2(-R[1,2], R[1,1])
            y = math.atan2(-R[2,0], sy)
            z = 0

        return math.degrees(x), math.degrees(y), math.degrees(z)

    def CrotationMatrix2EulerAngles(self, R):
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

    def eulerAnglesToRotationMatrix(self, theta):
        theta[0] = math.radians(theta[0])
        theta[1] = math.radians(theta[1])
        theta[2] = math.radians(theta[2])
        
        R_x = np.array([[1,         0,                  0                   ],
                        [0,         math.cos(theta[0]), -math.sin(theta[0]) ],
                        [0,         math.sin(theta[0]), math.cos(theta[0])  ]
                        ])
                
        R_y = np.array([[math.cos(theta[1]),    0,      math.sin(theta[1])  ],
                        [0,                     1,      0                   ],
                        [-math.sin(theta[1]),   0,      math.cos(theta[1])  ]
                        ])
                    
        R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                        [math.sin(theta[2]),    math.cos(theta[2]),     0],
                        [0,                     0,                      1]
                        ])
                        
        R = np.dot(R_z, np.dot( R_y, R_x ))
        
        return R

    def changeDatum(self, R, t):
        # Original (ArUco wrt camera)
        Tca = np.append(R, t, axis=1)
        Tca = np.append(Tca, np.array([[0, 0, 0, 1]]), axis=0)

        # Transformation (camera wrt drone)
        Tbc = np.array([[1,  0,  0,  0],
                        [0,  1,  0,  0],
                        [0,  0,  1,  0],
                        [0,  0,  0,  1]])

        # Resultant pose (ArUco wrt drone)
        Tba = np.dot(Tbc, Tca)

        # Return results
        R = Tba[0:3,0:3]
        t = Tba[0:3,3]

        # Body frame wrt ArUco
        R = np.transpose(R)
        # t = np.dot(-R, t)

        return R, t[0], t[1], t[2]

    def getPose(self):
        # Store a local frame 
        localFrame = self.frame
        
        # Get frame and convert to gray
        gray = cv2.cvtColor(localFrame, cv2.COLOR_BGR2GRAY)
        
        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict, parameters=self.parm, cameraMatrix=self.mtx, distCoeff=self.dist)

        # Only continue if a marker was found
        if np.all(ids != None):
            # Estimate the pose
            _, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.mtx, self.dist, self.rvec, self.tvec)

            # Draw on the frame 
            aruco.drawDetectedMarkers(localFrame, corners)
            aruco.drawAxis(localFrame, self.mtx, self.dist, rvec, tvec, 7)
            
            # Convert from vector to rotation matrix
            R, _ = cv2.Rodrigues(rvec)

            # Transform
            R, xx, yy, zz = self.changeDatum(R, tvec)
  
            # Get angles (two different methods)
            roll_A, pitch_A, yaw_A = self.ArotationMatrix2EulerAngles(R)
            A = R - self.eulerAnglesToRotationMatrix([roll_A, pitch_A, yaw_A])
            if (np.sum(np.absolute(A)) < 1e-9):
                aState = 'ZYX'
            else:
                aState = 'ERROR'

            roll_B, pitch_B, yaw_B = self.BrotationMatrix2EulerAngles(R)
            B = R - self.eulerAnglesToRotationMatrix([roll_B, pitch_B, yaw_B])
            if (np.sum(np.absolute(B)) < 1e-9):
                bState = 'ZYX'
            else:
                bState = 'ERROR'

            x, y, z = self.CrotationMatrix2EulerAngles(R)
            C = R - self.eulerAnglesToRotationMatrix([x, y, z])
            if (np.sum(np.absolute(C)) < 1e-9):
                cState = 'ZYX'
            else:
                cState = 'ERROR'

            # # Bounding
            if (x > 0):
                x -= 180
            elif (x < 0):
                x += 180
                
            # Save translation and rotation for next iteration 
            self.rvec = rvec
            self.tvec = tvec 

            # Print to screen 
            cv2.putText(localFrame, aState, (0, 25), self.fontFace, self.fontScale, self.colorA, self.lineType)
            cv2.putText(localFrame, "R: " + str(round(roll_A,1)),  (0, 50),  self.fontFace, self.fontScale, self.colorA, self.lineType)
            cv2.putText(localFrame, "P: " + str(round(pitch_A,1)), (0, 75),  self.fontFace, self.fontScale, self.colorA, self.lineType)
            cv2.putText(localFrame, "Y: " + str(round(yaw_A,1)),   (0, 100),  self.fontFace, self.fontScale, self.colorA, self.lineType)

            cv2.putText(localFrame, bState, (1150, 25), self.fontFace, self.fontScale, self.colorB, self.lineType)     
            cv2.putText(localFrame, "R: " + str(round(roll_B,1)),  (1150, 50), self.fontFace, self.fontScale, self.colorB, self.lineType)
            cv2.putText(localFrame, "P: " + str(round(pitch_B,1)), (1150, 75), self.fontFace, self.fontScale, self.colorB, self.lineType)
            cv2.putText(localFrame, "Y: " + str(round(yaw_B,1)),   (1150, 100), self.fontFace, self.fontScale, self.colorB, self.lineType)

            cv2.putText(localFrame, cState, (0, 625), self.fontFace, self.fontScale, self.colorC, self.lineType)     
            cv2.putText(localFrame, "z: " + str(round(z,1)), (0, 645), self.fontFace, self.fontScale, self.colorC, self.lineType)
            cv2.putText(localFrame, "y: " + str(round(y,1)), (0, 670), self.fontFace, self.fontScale, self.colorC, self.lineType)
            cv2.putText(localFrame, "x: " + str(round(x,1)), (0, 695), self.fontFace, self.fontScale, self.colorC, self.lineType)
 
            cv2.putText(localFrame, "X: " + str(round(xx,1)), (1150, 645), self.fontFace, self.fontScale, self.color0, self.lineType)
            cv2.putText(localFrame, "Y: " + str(round(yy,1)), (1150, 670), self.fontFace, self.fontScale, self.color0, self.lineType)
            cv2.putText(localFrame, "Z: " + str(round(zz,1)), (1150, 695), self.fontFace, self.fontScale, self.color0, self.lineType)

        return localFrame
                
    def run(self):
        # Start the connection to the camera and start threading
        self.startCamera()
        self.startFrameThread()

        # Start the performance metrics
        self.startTime = time.time()

        # Process data until closed
        try: 
            while(True):
                # Calculate pose and display
                localFrame = self.getPose()

                # display the resulting frame
                cv2.imshow('Frame', localFrame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

                # Stabilize rate
                time.sleep(1/60)

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

def main():    
    # Initialize class
    vt = VisionTest()
    vt.run()

# Main loop
if __name__ == '__main__':
    main()
