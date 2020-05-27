from multiprocessing import Process, Queue
from VisionMultiCore import *
import multiprocessing as mp
import queue
import time
import cv2

def main():
    # Set up the vision class
    v = VisionMultiCore()

    # Create an exit event
    quitVision = mp.Event()

    # Initialize queue and start the computer vision core
    q = Queue()
    p = Process(target=v.processFrame, args=(q, quitVision))
    p.start()

    # Run for 10 secounds 
    print("Starting")
    startTime = time.time()
    try:
        while(True):
            print(q.get())
    except:
        # Close the core 
        quitVision.set()
        p.join()
        
# Main loop
if __name__ == '__main__':
    main()
