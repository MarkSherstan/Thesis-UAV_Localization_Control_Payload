import numpy as np
import pickle
import time
import glob
import cv2
import cv2.aruco as aruco

class CalibrateCamera:
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
            self.cam.set(cv2.CAP_PROP_FOCUS, 0)
            self.cam.set(cv2.CAP_PROP_ZOOM, 0)
            self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
            print('Camera start')
        except:
            print('Camera setup failed')        
    
    def startT265(self):
        # Import custom class
        import sys
        sys.path.append('../')
        from T265 import T265
        
        # Save the object to this class
        self.cam = T265()
        
    def generateCharucoBoard(self, rows=7, columns=5):
        # Create the board
        board = aruco.CharucoBoard_create(
                    squaresX=columns,
                    squaresY=rows,
                    squareLength=0.025,
                    markerLength=0.0125,
                    dictionary=self.arucoDict)
        img = board.draw((100*columns, 100*rows))

        # Save it to a file
        cv2.imwrite('CharucoBoard.png', img)

    def generateArucoBoard(self, rows=3, columns=4):
        # Create the board 
        board = aruco.GridBoard_create(
            markersX=columns,
            markersY=rows,
            markerLength=0.1,
            markerSeparation=0.1/2,
            dictionary=self.arucoDict)
        img = board.draw((175*columns, 175*rows))
        
        # Show the image 
        cv2.imshow('ArUco Board', img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()        

        # Save it to a file
        cv2.imwrite('ArUcoBoard.png', img)
      
    def generateArucoMarker(self, ID=2, size=450):
        # Create an image from the marker
        img = aruco.drawMarker(self.arucoDict, ID, size)

        # Save and display (press any key to exit)
        cv2.imwrite('ARUCO_'+str(ID)+'.png', img)
        cv2.imshow('Marker ID: ' + str(ID), img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def captureCalibrationImages(self, T265=True, camIdx=1, C920=False):
        # Start the proper capture device
        if T265 is True:
            self.startT265()
        elif C920 is True:
            self.startCamera()
        else:
            print('ERROR')
            return

        # Initialize snapshot counter
        index = 0

        while(True):
            # Get a frame
            if T265 is True:
                if camIdx == 1:
                    frame = self.cam.Img1
                elif camIdx == 2:
                    frame = self.cam.Img2
                else:
                    print("ERROR: Camera index, use 1 or 2 for the T265")
                    return
            elif C920 is True:
                _, frame = self.cam.read()
            else:
                print('ERROR')
                return

            # Show the image
            cv2.imshow('Frame', frame)

            # Check keyboard commands
            #   'space' -> Snapshot
            #   'q' -> Quit
            key = cv2.waitKey(1)

            if key != -1:
                if key & 0xFF == ord(' '):
                    indexedFile = self.calibrationDir + 'IMG_' + str(index+1) + self.imgExtension
                    print(indexedFile)
                    cv2.imwrite(indexedFile, frame)
                    index += 1
                elif key & 0xFF == ord('q'):
                    break

        # Clear connections
        if T265 is True:
            self.cam.close()
        elif C920 is True:
            self.cam.release()

        # Close all windows
        cv2.destroyAllWindows()

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

        # Start a timer
        startTime = time.time()

        # Run calibration
        _, self.mtx, self.dist, _, _ = aruco.calibrateCameraCharuco(
                charucoCorners=cornerList,
                charucoIds=idList,
                board=board,
                imageSize=imageSize,
                cameraMatrix=None,
                distCoeffs=None)

        # Print how long it took
        print("Calibration took: ", round(time.time()-startTime),' s')

        # Display matrix and distortion coefficients
        print('Image size: ', imageSize)
        print(self.mtx)
        print(self.dist)

        # Pickle the results
        f = open('calibration.pckl', 'wb')
        pickle.dump((self.mtx, self.dist), f)
        f.close()

    def generateCalibrationImg(self, rows=7, columns=5, lengthSquare=0.0354, lengthMarker=0.0177):
        # Create charuco board with actual measured dimensions from print out
        board = aruco.CharucoBoard_create(
                    squaresX=columns,
                    squaresY=rows,
                    squareLength=lengthSquare,
                    markerLength=lengthMarker,
                    dictionary=self.arucoDict)

        # Get image paths from calibration folder
        paths = glob.glob(self.calibrationDir + '*' + self.imgExtension)

        # Image list
        data = []
        
        # Counter
        ii = 0

        # Loop through all images
        for filePath in paths:
            # Read image and convert to gray
            img = cv2.imread(filePath)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find aruco markers in the query image
            corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict)

            # Outline the aruco markers found in the query image
            aruco.drawDetectedMarkers(image=img, corners=corners)
       
            # Get charuco corners and ids from detected aruco markers
            response, charucoCorners, charucoIDs = aruco.interpolateCornersCharuco(
                    markerCorners=corners,
                    markerIds=ids,
                    image=gray,
                    board=board)

            # If at least 20 corners were found
            if response > 20:
                # Draw the Charuco board detected to show calibration results
                aruco.drawDetectedCornersCharuco(image=img, charucoCorners=charucoCorners, charucoIds=charucoIDs)
                
                # Resize image and save to list
                data.append(img)
                
                # Incrememnt counter
                ii += 1     
            else:
                # Error message
                print('Error in: ' + str(filePath))

            # Exit condition
            if (ii >= 9):
                break

        # Create matrix image
        A = np.concatenate((data[0], data[1], data[2], data[3]), axis=1)
        B = np.concatenate((data[4], data[5], data[6], data[7]), axis=1)
        D = np.concatenate((A, B), axis=0)
                
        # Display and save matrix image
        cv2.imshow('Photo Matrix', D)
        cv2.imwrite('PhotoMatrix.png', D)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    def getCalibration(self, printFlag=True):
        # Open file, retrieve variables, and close
        file = open('calibration.pckl', 'rb')
        self.mtx, self.dist = pickle.load(file)
        file.close()

        # Print results for later use
        if printFlag is True:
            print(self.mtx)
            print(self.dist)
        
def main():    
    # Initialize class
    CC = CalibrateCamera()

    # CC.generateCharucoBoard()
    # CC.generateArucoBoard()
    # CC.generateArucoMarker()

    # CC.captureCalibrationImages()
    # CC.calibrateCamera()
    # CC.generateCalibrationImg()

    # CC.getCalibration()


# Main loop
if __name__ == '__main__':
    main()
