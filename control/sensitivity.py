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
    nAvg = MovingAverage(1)
    eAvg = MovingAverage(1)
    dAvg = MovingAverage(1)
    yAvg = MovingAverage(1)

    # Create a Kalman filter
    yKF = KalmanFilter()
    kalmanTimer = time.time()
    
    # Logging variables
    freqList = []
    data = []

    nList = []
    eList = []
    dList = []
    yList = []

    count = 0

    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()

    try:
        while(True):
            # Get vision data
            northVraw, eastVraw, downVraw, yawVraw = getVision(Q)

            # Log data
            nList.append(northVraw)
            eList.append(eastVraw)
            dList.append(downVraw)
            yList.append(yawVraw)

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
            print('idx: {:<8.0f} f: {:<8.0f} N: {:<8.1f} E: {:<8.1f} D: {:<8.1f} Y: {:<8.1f}'.format(count, freqLocal, northV, eastV, downV, yawV))
            loopTimer = time.time()

            if (count > 300):
                break
            else:
                count = count + 1

            # Log data
            # data.append([time.time()-startTime, freqLocal, 
            #             northV, eastV, downV, yawV, 
            #             northVraw, eastVraw, downVraw, yawVraw])
            
    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')

    finally:        
        # Post main loop rate
        print("Average loop rate: ", round(statistics.mean(freqList),2), "+/-", round(statistics.stdev(freqList), 2))


        print("North: ", round(statistics.mean(nList),3), "+/-", round(statistics.stdev(nList), 3))
        print("East: ", round(statistics.mean(eList),3), "+/-", round(statistics.stdev(eList), 3))
        print("Down: ", round(statistics.mean(dList),3), "+/-", round(statistics.stdev(dList), 3))
        print("Yaw: ", round(statistics.mean(yList),3), "+/-", round(statistics.stdev(yList), 3))

        # Write data to a data frame
        # df = pd.DataFrame(data, columns=['Time', 'Freq',
        #                     'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
        #                     'northVraw', 'eastVraw', 'downVraw', 'yawVraw'])

        # Save data to CSV
        # now = datetime.datetime.now()
        # fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        # df.to_csv(fileName, index=None, header=True)
        # print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
