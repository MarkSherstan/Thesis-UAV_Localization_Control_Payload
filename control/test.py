
from multiprocessing import Process, Queue
from vision import Vision
import statistics
import time

printFlag = True

def getVision(Q):
    # Vision Data
    temp = Q.get()
    return temp[0], temp[1], temp[2], temp[3]

def main():
    # Connect to vision, create the queue, and start the core
    V = Vision(lengthMarker=4.5, spacing=2.25)
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()

    # Logging variables
    freqList = []
    loopTimer = time.time()

    try:
        while(True):
            # Get vision data
            northV, eastV, downV, yawV = getVision(Q)

            # Print data
            freqLocal = (1 / (time.time() - loopTimer))
            freqList.append(freqLocal)
            loopTimer = time.time()
            time.sleep(1/30)

            if printFlag is True:
                print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))

    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')
    finally:        
        # Post main loop rate
        print("Average loop rate: ", round(statistics.mean(freqList),2), "+/-", round(statistics.stdev(freqList), 2))

if __name__ == "__main__":
    main()
