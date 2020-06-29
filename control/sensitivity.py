from filter import MovingAverage, KalmanFilter
from multiprocessing import Process, Queue
from vision import Vision
import pandas as pd
import numpy as np
import statistics
import datetime
import time

def getVision(Q):
    # Vision Data
    temp = Q.get()
    return temp[0], temp[1], temp[2], temp[3]

def main():
    # Connect to vision, create the queue, and start the core
    V = Vision()
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()

    # Create low pass filters
    nAvg = MovingAverage(5)
    eAvg = MovingAverage(3)
    dAvg = MovingAverage(3)
    yAvg = MovingAverage(3)

    # Create a Kalman filter
    yKF = KalmanFilter()
    kalmanTimer = time.time()
    
    # Logging variables
    freqList = []
    data = []

    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()

    try:
        while(True):
            # Get vision data
            northVraw, eastVraw, downVraw, yawVraw = getVision(Q)

            # Smooth vision data with moving average low pass filter 
            northV = nAvg.update(northVraw)
            eastV  = eAvg.update(eastVraw)
            downV  = dAvg.update(downVraw)
            yawV   = yAvg.update(yawVraw)
            
            # Kalman Filter
            # yawV = yKF.update(time.time() - kalmanTimer, np.array([yawVraw, zGyro]).T)
            # kalmanTimer = time.time()

            # Print data
            freqLocal = (1 / (time.time() - loopTimer))
            freqList.append(freqLocal)
            print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))
            loopTimer = time.time()

            # Log data
            data.append([time.time()-startTime, freqLocal, 
                        northV, eastV, downV, yawV, 
                        northVraw, eastVraw, downVraw, yawVraw])
            
    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')

    finally:        
        # Post main loop rate
        print("Average loop rate: ", round(statistics.mean(freqList),2), "+/-", round(statistics.stdev(freqList), 2))

        # Write data to a data frame
        df = pd.DataFrame(data, columns=['Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'northVraw', 'eastVraw', 'downVraw', 'yawVraw', 'zGyro'])

        # Save data to CSV
        # now = datetime.datetime.now()
        # fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        # df.to_csv(fileName, index=None, header=True)
        # print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
