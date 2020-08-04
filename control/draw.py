import cv2.aruco as aruco
import numpy as np
import cv2

class Draw:
    def __init__(self, mtx, dist, lengthMarker=4.50, spacing=2.25):
        # Camera calibration matrix and array
        self.mtx = mtx
        self.dist = dist

        # Initial conditions for pose calculation 
        self.rvec = None
        self.tvec = None

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
            markersX=4,                 # Columns
            markersY=3,                 # Rows
            markerLength=lengthMarker,  # cm
            markerSeparation=spacing,   # cm
            dictionary=self.arucoDict)
        
        # Save length of marker
        self.lengthMarker = lengthMarker
        
    def arucoBoard(self, frame):
        # Convert frame to gray and rotate to normal
        gray = frame
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict, parameters=self.parm, cameraMatrix=self.mtx, distCoeff=self.dist)

        # Only continue if a marker was found
        if np.all(ids != None):
            # Estimate the pose
            _, rvec, tvec = aruco.estimatePoseBoard(corners, ids, self.board, self.mtx, self.dist, self.rvec, self.tvec)
            
            # Draw
            aruco.drawDetectedMarkers(frame, corners, ids)
            aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, 7)

            # Save translation and rotation for next iteration 
            self.rvec = rvec
            self.tvec = tvec
        
        # Return the result
        return frame

    def arucoMarker(self, frame):
        # Convert frame to gray and rotate to normal
        gray = frame
        frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

        # lists of ids and corners belonging to each id
        corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict, parameters=self.parm, cameraMatrix=self.mtx, distCoeff=self.dist)

        # Only continue if a marker was found
        if np.all(ids != None):
            # Estimate the pose
            rvec, tvec, _ =	aruco.estimatePoseSingleMarkers(corners, self.lengthMarker, self.mtx, self.dist, self.rvec, self.tvec)
            
            # Draw
            aruco.drawDetectedMarkers(frame, corners, ids)
            aruco.drawAxis(frame, self.mtx, self.dist, rvec, tvec, 3)

            # Save translation and rotation for next iteration 
            self.rvec = rvec
            self.tvec = tvec
        
        # Return the result
        return frame
    