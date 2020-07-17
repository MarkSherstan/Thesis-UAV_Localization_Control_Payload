import cv2.aruco as aruco
import numpy as np
import pickle
import cv2

class VisionTest:
    def __init__(self):
        # Create custom dictionary (# markers, # bits)
        self.arucoDict = aruco.custom_dictionary(17, 3)

        # Calibration 
        self.mtx = None
        self.dist = None

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

    def calibrateCamera(self, rows=7, columns=5, lengthSquare=0.0354, lengthMarker=0.0177):
        # Create charuco board with actual measured dimensions from print out
        board = aruco.CharucoBoard_create(
                    squaresX=columns,
                    squaresY=rows,
                    squareLength=lengthSquare,
                    markerLength=lengthMarker,
                    dictionary=self.arucoDict)

        # Sub pixel corner detection criteria 
        subPixCriteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 0.00001)

        # Storage variables
        cornerList = []
        idList = []

        # Get image paths from calibration folder
        paths = glob.glob(self.calibrationDir + '*' + self.imgExtension)

        # Empty imageSize variable to be determined at runtime
        imageSize = None

        # Loop through all images
        for filePath in paths:
            # Read image and convert to gray
            img = cv2.imread(filePath)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find aruco markers in the query image
            corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict)

            # Sub pixel detection
            for corner in corners:
                cv2.cornerSubPix(
                    image=gray, 
                    corners=corner,
                    winSize = (3,3),
                    zeroZone = (-1,-1),
                    criteria = subPixCriteria)

            # Get charuco corners and ids from detected aruco markers
            response, charucoCorners, charucoIDs = aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=gray,
                    board=board)

            # If at least 20 corners were found
            if response > 20:
                # Add the corners and ids to calibration list
                cornerList.append(charucoCorners)
                idList.append(charucoIDs)

                # If image size is still None, set it to the image size
                if not imageSize:
                    imageSize = gray.shape[::-1]
                    
            else:
                # Error message
                print('Error in: ' + str(filePath))

        # Make sure at least one image was found
        if len(paths) < 1:
            print('No images of charucoboards were found.')
            return

        # Make sure at least one charucoboard was found
        if not imageSize:
            print('Images supplied were not regonized by calibration')
            return

        # Run calibration
        _, self.mtx, self.dist, _, _ = aruco.calibrateCameraCharuco(
                charucoCorners=cornerList,
                charucoIds=idList,
                board=board,
                imageSize=imageSize,
                cameraMatrix=None,
                distCoeffs=None)

        # Display matrix and distortion coefficients
        print('Image size: ', imageSize)
        print(self.mtx)
        print(self.dist)

        # Pickle the results
        f = open('resources/calibration.pckl', 'wb')
        pickle.dump((self.mtx, self.dist), f)
        f.close()

    def getCalibration(self, printFlag=True):
        # Open file, retrieve variables, and close
        file = open('resources/calibration.pckl', 'rb')
        self.mtx, self.dist = pickle.load(file)
        file.close()

        # Print results for later use
        if printFlag is True:
            print(self.mtx)
            print(self.dist)
        