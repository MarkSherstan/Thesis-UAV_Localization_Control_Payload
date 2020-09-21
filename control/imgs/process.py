import numpy as np
import glob
import cv2

# Lower and upper thresholding block size
lower = 3
upper = 15

# Get image paths from calibration folder
camA = sorted(glob.glob('*A.png'))
camB = sorted(glob.glob('*B.png'))

# Loop through values
for ii in len(camA):
    # Read raw images
    tempA = cv2.imread(camA[ii])
    tempB = cv2.imread(camB[ii])
    
    # Apply adaptive thresh A
    lowerA = cv2.adaptiveThreshold(tempA.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, lower, 7)
    upperA = cv2.adaptiveThreshold(tempA.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, upper, 7)

    lowerB = cv2.adaptiveThreshold(tempB.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, lower, 7)
    upperB = cv2.adaptiveThreshold(tempB.copy(), 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY_INV, upper, 7)

    # Group images
    raw    = np.concatenate((tempA, tempB),   axis=1)
    lower  = np.concatenate((lowerA, lowerB), axis=1)
    upper  = np.concatenate((upperA, upperB), axis=1)
    master = np.concatenate((raw, lower, upper), axis=0)
    
    # Write 
    fileName = 'Master-' + str(ii) + '.png'
    cv2.imwrite(fileName, master)
    