from discountVision import Vision
import numpy as np
import glob
import cv2

# ArUco stuff
V = Vision()

# Lower and upper thresholding block size
threshLow = 3
threshHigh = 15

# Get image paths from calibration folder
camA = sorted(glob.glob('*A.png'))
camB = sorted(glob.glob('*B.png'))

# Loop through values
for ii in range(len(camA)):
    # Read raw images
    tempA = cv2.imread(camA[ii])
    tempB = cv2.imread(camB[ii])
    
    tempA = cv2.cvtColor(tempA, cv2.COLOR_BGR2GRAY)
    tempB = cv2.cvtColor(tempB, cv2.COLOR_BGR2GRAY)
        
    # Apply adaptive thresh A
    lowerA = cv2.adaptiveThreshold(tempA.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, threshLow, 7)
    upperA = cv2.adaptiveThreshold(tempA.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, threshHigh, 7)

    lowerB = cv2.adaptiveThreshold(tempB.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, threshLow, 7)
    upperB = cv2.adaptiveThreshold(tempB.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, threshHigh, 7)

    # Update ArUco
    tempA = V.VP1.getPose(tempA)
    tempB = V.VP2.getPose(tempB)
    
    # Group images
    raw    = np.concatenate((tempA, tempB),   axis=1)
    lower  = np.concatenate((lowerA, lowerB), axis=1)
    upper  = np.concatenate((upperA, upperB), axis=1)
    master = np.concatenate((raw, lower, upper), axis=0)
    
    # Write 
    fileName = 'Master-' + str(ii) + '.png'
    cv2.imwrite(fileName, master)
