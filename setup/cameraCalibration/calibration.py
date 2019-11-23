import numpy as np
import pickle
import glob
import cv2
import cv2.aruco as aruco

class ARUCO:
    def __init__(self, arucoDict):
        self.arucoDict = aruco.Dictionary_get(arucoDict)
        self.frameWidth = 1280
        self.frameHeight = 720

        self.mtx = None
        self.dist = None

        self.calibrationDir = 'calibration/'
        self.imgExtension = '.jpg'

    def generateMarker(self, ID=7, size=700):
        # Create an image from the marker
        img = aruco.drawMarker(self.arucoDict, ID, size)

        # Save and display (press any key to exit)
        cv2.imwrite('ARUCO_'+str(ID)+'.png', img)
        cv2.imshow('Marker ID: ' + str(ID), img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def captureCalibrationImages(self):
        # Initialize snapshot counter
        index = 0

        # Start webcam
        cam = cv2.VideoCapture(1)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.frameWidth)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frameHeight)

        while(True):
            # Get frame and show
            _, frame = cam.read()
            cv2.imshow('Frame', frame)

            # Check keyboard commands
            #   'space' -> Snapshot
            #   'q' -> Quit
            key = cv2.waitKey(1)

            if key != -1:
                if key & 0xFF == ord(' '):
                    indexedFile = self.calibrationDir + 'IMG_' + str(index) + self.imgExtension
                    print(indexedFile)
                    cv2.imwrite(indexedFile, frame)
                    index += 1
                elif key & 0xFF == ord('q'):
                    break

        # Clear connections and window
        cam.release()
        cv2.destroyAllWindows()

    def calibrateCamera(self):
        # Based on: https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
        # Store object points (3D) and image points (2D) from processed images
        objpoints = []
        imgpoints = []

        # Chessboard size
        rowCount = 9
        colCount = 6

        # prepare object points, e.g (0,0,0), (1,0,0), (2,0,0) ... (rowCount-1,colCount-1,0)
        objp = np.zeros((rowCount*colCount,3), np.float32)
        objp[:,:2] = np.mgrid[0:rowCount,0:colCount].T.reshape(-1,2)

        # Get image paths from calibration folder
        paths = glob.glob(self.calibrationDir + '*' + self.imgExtension)

        # Empty imageSize variable for run time
        imageSize = None

        # Loop through all the images
        for filePath in paths:
            # Read image and convert to gray
            img = cv2.imread(filePath)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

            # Find the chess board corners
            ret, corners = cv2.findChessboardCorners(gray, (rowCount,colCount), None)

            # If points are found
            if ret == True:
                # Add points discovered
                objpoints.append(objp)

                # Refines the corner locations and add the points
                corners2 = cv2.cornerSubPix(
                                image = gray,
                                corners = corners,
                                winSize = (11,11),
                                zeroZone = (-1,-1),
                                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                imgpoints.append(corners2)

                # Get image size for calibration later
                if not imageSize:
                    imageSize = gray.shape[::-1]

                # Draw and display the calibration. Wait for user to smash a key before moving on
                img = cv2.drawChessboardCorners(img, (rowCount,colCount), corners2, ret)
                cv2.imshow(filePath, img)
                cv2.waitKey(0)
            else:
                print('Not able to detect a chessboard in image: {}'.format(filePath))

        # Close all windows
        cv2.destroyAllWindows()

        # Error handling
        if not imageSize:
            print('Calibration failed. Try again with more / new images')
            exit()

        # Run calibration
        ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
                                            objectPoints = objpoints,
                                            imagePoints = imgpoints,
                                            imageSize = imageSize,
                                            cameraMatrix = None,
                                            distCoeffs = None)

        # Display matrix and distortion coefficients
        print(mtx)
        print(dist)

        # Pickle the results
        f = open('calibration.pckl', 'wb')
        pickle.dump((mtx, dist), f)
        f.close()

    def getCalibration(self):
        # Open file, retrieve variables, and close
        file = open('calibration.pckl', 'rb')
        self.mtx, self.dist = pickle.load(file)
        file.close()

    def trackAruco(self):
        # Get calibration data
        try:
            self.getCalibration()
        except:
            print('Calibration not found!')

        # Font and color for screen writing
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontColor = (0, 255, 0)

        # Start webcam
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.frameWidth)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frameHeight)

        # Set parameters
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        while(True):
            # Get frame and convert to gray
            _, frame = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # lists of ids and corners belonging to each id
            corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=parameters)

            # Only continue if a marker was found
            if np.all(ids != None):
                # Estimate the pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)

                # Draw axis for each aruco marker found
                for ii in range(0, ids.size):
                    aruco.drawAxis(frame, self.mtx, self.dist, rvec[ii], tvec[ii], 0.1)

                # Draw square around markers
                aruco.drawDetectedMarkers(frame, corners)

                # Print ids found in top left
                idz = ''
                for ii in range(0, ids.size):
                    idz += str(ids[ii][0])+' '
                    x = round(tvec[ii][0][0]*100,2)
                    y = round(tvec[ii][0][1]*100,2)
                    z = round(tvec[ii][0][2]*100,2)
                    print(x,y,z)

                cv2.putText(frame, "ID: " + idz, (0, 25), font, 1, fontColor, 2)

            # display the resulting frame
            cv2.imshow('frame', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When complete close everything down
        cam.release()
        cv2.destroyAllWindows()

    def blockManipulator(self, serialPort='/dev/cu.usbmodem14201', rec=False):
        # Get calibration data
        try:
            self.getCalibration()
        except:
            print('Calibration not found!')

        # Prepare the video recorder
        if rec is True:
            now = datetime.datetime.now()
            timeStamp = now.strftime("%Y-%m-%d_%H.%M.%S") + ".avi"

            out = cv2.VideoWriter(timeStamp, cv2.VideoWriter_fourcc('M','J','P','G'), 30, (self.frameWidth, self.frameHeight))

        # Font for screen writing
        font = cv2.FONT_HERSHEY_SIMPLEX

        # Start webcam
        cam = cv2.VideoCapture(0)
        cam.set(cv2.CAP_PROP_FRAME_WIDTH, self.frameWidth)
        cam.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frameHeight)

        # Start serial port
        ser = serial.Serial(serialPort, 9600)

        # Set parameters
        parameters = aruco.DetectorParameters_create()
        parameters.adaptiveThreshConstant = 10

        while(True):
            # Get frame and convert to gray
            _, frame = cam.read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            # lists of ids and corners belonging to each id
            corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=parameters)

            # Only continue if a marker was found
            if np.all(ids != None):
                # Estimate the pose
                rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)

                # Draw axis for each aruco marker found
                for ii in range(0, ids.size):
                    aruco.drawAxis(frame, self.mtx, self.dist, rvec[ii], tvec[ii], 0.1)

                # Draw square around markers
                aruco.drawDetectedMarkers(frame, corners)

                # Look at IDs found
                if (ids.size is 1):
                    idz = str(ids[0][0])
                    x = round(tvec[0][0][0]*100,1)
                    y = round(tvec[0][0][1]*100,1)
                    z = round(tvec[0][0][2]*100,1)

                    # Grab
                    if ((idz == '21') or (idz == '97')):
                        fontColor = (0, 255, 0)
                        cv2.putText(frame, "ID: " + idz, (0, 30), font, 1, fontColor, 2)
                        cv2.putText(frame, "X " + str(x), (0, 60), font, 1, fontColor, 2)
                        cv2.putText(frame, "Y " + str(y), (0, 90), font, 1, fontColor, 2)
                        cv2.putText(frame, "Z " + str(z), (0, 120), font, 1, fontColor, 2)

                        if ((abs(x) < 3) and (abs(y) < 3) and (18 <= z <= 22)):
                            ser.write(chr(0x68))
                            # cv2.putText(frame, "Grabbing item!", (1000, 30), font, 1, (0, 0, 0), 2)
                    # Dont grab
                    elif ((idz == '15') or (idz == '13')):
                        fontColor = (0, 0, 255)
                        cv2.putText(frame, "ID: " + idz, (0, 30), font, 1, fontColor, 2)
                        cv2.putText(frame, "X " + str(x), (0, 60), font, 1, fontColor, 2)
                        cv2.putText(frame, "Y " + str(y), (0, 90), font, 1, fontColor, 2)
                        cv2.putText(frame, "Z " + str(z), (0, 120), font, 1, fontColor, 2)
                else:
                    cv2.putText(frame, "To many objects", (0, 25), font, 1, (0, 0, 0), 2)

            # display the resulting frame
            cv2.imshow('frame', frame)

            if rec is True:
                out.write(frame)

            # Either quit (q) or reset the claw (' ' -> Spacebar)
            key = cv2.waitKey(1)

            if key != -1:
                if key & 0xFF == ord(' '):
                    ser.write(chr(0x14))
                    print('Reset gripper')
                elif key & 0xFF == ord('q'):
                    if rec is True:
                        out.release()
                    cam.release()
                    cv2.destroyAllWindows()
                    break

def main():
    ac = ARUCOCLAW(aruco.DICT_5X5_1000)

    # ac.generateMarker(ID=97, size=90)
    # ac.generateMarker(ID=35, size=500)

    # ac.captureCalibrationImages()

    # ac.calibrateCamera()

    # ac.getCalibration()
    # print(type(ac.mtx))
    #
    # print(type(ac.dist))
    # print(ac.mtx)
    # print(ac.dist)

    ac.trackAruco()
    # ac.blockManipulator(rec=True)

# Main loop
if __name__ == '__main__':
	main()
