from multiprocessing import Process, Queue
import queue
import time
from VisionMultiCore import *
import cv2


def main():
    v = VisionMultiCore(1280, 720, 30, 0)

    q = Queue()
    p = Process(target=v.processFrame, args=(q,))
    p.start()

    print("Starting")
    startTime = time.time()
    while(time.time() < startTime+10):
        print(q.get())
        
    
    # for ii in range(10):
    q.put(-1)
    
    print('hello world')
    p.join()
    print('hello world 2')
    

# Main loop
if __name__ == '__main__':
    main()
