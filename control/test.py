from multiprocessing import Process, Queue
from vision import Vision
import statistics
import time

def main():
    # Connect to vision, create the queue, and start the core
    V = Vision()
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()

    # Start timer
    loopTimer = time.time()
    freqList = []

    # Run forever unitl caught
    try:
        while(True):
            # Get vision data
            temp = Q.get()
            print(temp, Q.qsize())

            # Print data
            freqLocal = (1 / (time.time() - loopTimer))
            freqList.append(freqLocal)
            loopTimer = time.time()

    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')
        
    finally:        
        # Post main loop rate
        print("Average loop rate: ", round(statistics.mean(freqList),2), "+/-", round(statistics.stdev(freqList), 2))

        
if __name__ == "__main__":
    main()
