from discountVision import Vision
import numpy as np
import shutil
import glob
import cv2
import os

# Change file names to 000.png file format (natural sorting)
sourcedir = os.getcwd(); extensions = (".png")
files = [(f, f[f.rfind("."):], f[:f.rfind(".")]) for f in os.listdir(sourcedir)if f.endswith(extensions)]
maxlen = len(max([f[2] for f in files], key = len))

for item in files:
    zeros = maxlen-len(item[2])
    shutil.move(sourcedir+"/"+item[0], sourcedir+"/"+str(zeros*"0")+item[0])

# ArUco stuff
V = Vision()

V.VP1.parm.adaptiveThreshWinSizeMin = 9
V.VP2.parm.adaptiveThreshWinSizeMin = 9

V.VP1.parm.adaptiveThreshWinSizeMax = 23
V.VP2.parm.adaptiveThreshWinSizeMax = 23

V.VP1.parm.adaptiveThreshWinSizeStep = 14
V.VP2.parm.adaptiveThreshWinSizeStep = 14

# Lower and upper thresholding block size
threshLow = 9
threshHigh = 23

# Counter 
countA = 0
countB = 0

# Get image paths from calibration folder
camA = sorted(glob.glob('*A.png'))
camB = sorted(glob.glob('*B.png'))

# Loop through values
for ii in range(len(camA)):
    # Read raw images and convert to gray scale
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
    tempA, numA = V.VP1.getPose(tempA, graphics=True)
    tempB, numB = V.VP2.getPose(tempB, graphics=True)
    
    # Increment counter
    countA += numA
    countB += numB
    
    # Group images
    lowerA = cv2.cvtColor(lowerA, cv2.COLOR_GRAY2RGB)
    lowerB = cv2.cvtColor(lowerB, cv2.COLOR_GRAY2RGB)
    upperA = cv2.cvtColor(upperA, cv2.COLOR_GRAY2RGB)
    upperB = cv2.cvtColor(upperB, cv2.COLOR_GRAY2RGB)
    
    A = np.concatenate((tempA, lowerA, upperA), axis=1)
    B = np.concatenate((tempB, lowerB, upperB), axis=1)
    master = np.concatenate((A, B), axis=0)
        
    # Write 
    fileName = 'Master-' + str(ii) + '.png'
    cv2.imwrite(fileName, master)

print(countA, countB)
