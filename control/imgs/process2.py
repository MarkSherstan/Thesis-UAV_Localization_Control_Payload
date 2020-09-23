from discountVision import Vision
import pandas as pd
import numpy as np
import time
import glob
import cv2

# Get image paths from calibration folder
camA = sorted(glob.glob('*A.png'))
camB = sorted(glob.glob('*B.png'))

# Vars
data = []
counter = 1
startTime = time.time()

# Loop through min, max, and steps
for adaptiveThreshWinSizeMin in range(3, 30):
    for adaptiveThreshWinSizeMax in range(3, 30):
        for adaptiveThreshWinSizeStep in range(3, 30):
            # Lazy assert
            min3 = (adaptiveThreshWinSizeMin >= 3) and (adaptiveThreshWinSizeMax >= 3)
            maxMin = adaptiveThreshWinSizeMax >= adaptiveThreshWinSizeMin
            not0 = adaptiveThreshWinSizeStep > 0

            # Check number of scales
            if (min3 and maxMin and not0):
                nScales = int((adaptiveThreshWinSizeMax - adaptiveThreshWinSizeMin) / adaptiveThreshWinSizeStep + 1)

                # If scales is two proceed with calc
                if (nScales == 2):
                    # ArUco stuff
                    V = Vision()
                    V.VP1.parm.adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin
                    V.VP1.parm.adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax
                    V.VP1.parm.adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep
                    V.VP2.parm.adaptiveThreshWinSizeMin = adaptiveThreshWinSizeMin
                    V.VP2.parm.adaptiveThreshWinSizeMax = adaptiveThreshWinSizeMax
                    V.VP2.parm.adaptiveThreshWinSizeStep = adaptiveThreshWinSizeStep

                    # Zero counting vars
                    countA = 0
                    countB = 0

                    # Loop through images
                    for ii in range(len(camA)):
                        # Read raw images
                        tempA = cv2.imread(camA[ii])
                        tempB = cv2.imread(camB[ii])

                        # Check if an ArUco marker was found
                        _, foundA = V.VP1.getPose(tempA)
                        _, foundB = V.VP2.getPose(tempB)

                        # Increment counter
                        countA += foundA
                        countB += foundB
                    
                    # Thresh windows
                    thresh1 = adaptiveThreshWinSizeMin
                    thresh2 = adaptiveThreshWinSizeMin + adaptiveThreshWinSizeStep
                                       
                    # Log the data
                    data.append([adaptiveThreshWinSizeMin, adaptiveThreshWinSizeMax,
                                 adaptiveThreshWinSizeStep, countA, countB, countA+countB, 
                                 thresh1, thresh2, len(camA), len(camB)])

                    # Update user
                    print('{:<d} of 1654 complete. Time elapsed: {:<0.2f}'.format(counter, time.time()-startTime))
                    counter += 1

df = pd.DataFrame(data, columns=['adaptiveThreshWinSizeMin', 'adaptiveThreshWinSizeMax',
                                 'adaptiveThreshWinSizeStep', 'foundA', 'foundB', 'Total Found', 
                                 'Thresh1', 'Thresh2', 'Cam A Imgs', 'Cam B Imgs'])
df.to_csv('Data.csv', index=None, header=True)
