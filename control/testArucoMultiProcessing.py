# from multiprocessing import Process, Queue
# import queue
# from VisionMultiCore import *
# import time
# import cv2

# from multiprocessing import Process, Queue

# def f(q):
#     q.put([42, None, 'hello'])

# if __name__ == '__main__':
#     q = Queue()
#     p = Process(target=f, args=(q,))
#     p.start()
#     print(q.get())    # prints "[42, None, 'hello']"
#     p.join()


# import cv2

# # Required to communicate with USB capture device
# cam = cv2.VideoCapture(0)
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
# cam.set(cv2.CAP_PROP_FPS, 30)
# cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
# print('Camera start')

# # Get frame, resize, and display forever until someone enters q
# while(True):
#     ret,frame = cam.read()
#     frameNew = cv2.resize(frame, (1920,1080))
#     cv2.imshow('Window',frameNew)

#     if (cv2.waitKey(1) & 0xFF == ord('q')):
#         break

# # Clear connections and window
# cam.release()
# cv2.destroyAllWindows()

# def processFrame(q):
#     # Start the connection to the camera
#     try:
#         # self.cam = cv2.VideoCapture(self.cameraIdx, cv2.CAP_V4L)
#         # self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#         cam = cv2.VideoCapture(0)
#         cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#         cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#         cam.set(cv2.CAP_PROP_FPS, 30)
#         cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
#         print('Camera start')
#     except:
#         print('Camera setup failed')

#     # Process data until closed
#     while(True):
#         # Check if we can end the process
#         exitCode = q.get()
#         if (exitCode == -1):
#             break

#         # Capture a frame
#         _, frame = cam.read()

#         # # Process the frame
#         # self.getPose()

#         # Add data to the queue
#         q.put([1, 2, 3, 4])

#     # Release the camera connection
#     cam.release()
#     print('Camera closed')


# def main():    
#     # # Set desired parameters
#     # desiredWidth  = 1280
#     # desiredHeight = 720
#     # desiredFPS    = 30
#     # cameraIdx     = 0

#     # Initialize class 
#     # v = VisionMultiCore(desiredWidth, desiredHeight, desiredFPS, cameraIdx)

#     # Create a queue
#     q = Queue()

#     # Start a process
#     p = Process(target=processFrame, args=(q,))
#     p.start()

#     # Block until there is data
#     data = None
#     while (data is None):
#         try:
#             data = q.get(False)  
#         except queue.Empty:
#             data = None

#     # Run test
#     print('Test running for 10 secounds\n')
#     startTime = time.time()

#     while (time.time() < startTime+10):
#         print(q.get())

#     q.put(-1)
#     time.sleep(1)
#     p.join()

#     print('Complete')
#     exit()

        # Check if we can end the process
        # exitCode = q.get()
        # if (exitCode == -1):
        #     break        
        # try:
        #     data = q.get(False)

#         #     if (data == -1):
#         #         break
#         #     else:
#         #         q.put(data)

#         # except queue.Empty:
#         #     data = None

from multiprocessing import Process, Queue
import queue
import time

def f(q):
    # Initialize
    counter = 0

    # loop until broken 
    while(True):
        # Check if we can end the process
        try:
            data = q.get(False)  
            if data == -1:
                break
        except queue.Empty:
            pass
        
        # some infinite data    
        counter += 1
        q.put(counter)


if __name__ == '__main__':
    q = Queue()
    p = Process(target=f, args=(q,))
    p.start()

    startTime = time.time()
    while(time.time() < startTime+0.1):
        data = q.get()
        print(time.time()-startTime, data)

    q.put(-1)
    
    print('hello world')
    p.join()
    print('hello world 2')
    




#     # for ii in range(len(desiredWidth)):
#     #     # Camera properties 
#     #     v = VisionMultiCore(q, desiredWidth[ii], desiredHeight[ii], desiredFPS, 0)
#     #     v.startPoseProcess()
 
#     #     # Block until there is data
#     #     data = None
#     #     while (data is None):
#     #         try:
#     #             data = q.get(False)  
#     #         except queue.Empty:
#     #             data = None
        
#     #     # Counting variables
#     #     startTime = time.time()

#     #     # Run test
#     #     print('Test running for 10 secounds\n')
#     #     while (time.time() < startTime+10):
            
#     #         # # Unpack the data
#     #         data = q.get()

#     #         print(time.time()-startTime, data)
#     #         # print('N: %0.2f E: %0.2f D: %0.2f Y: %0.2f' % (dataOut[0], dataOut[1, dataOut[2], dataOut[3]]))
#     #         time.sleep(0.5)
            


#     #     print("end")

#     #     # Close process and camera connection
#     #     v.close()
        
#     #     print("yea the thread did not close")
#     #     # Small pause before next resolution test
#     #     time.sleep(1)
    
# Main loop
# if __name__ == '__main__':
#     main()
    




# THIS WORKS

# from multiprocessing import Process, Queue
# import time

# def f(q):
#     counter = 0

#     while(counter < 10000):
#         counter += 1
#         q.put(counter)

#     q.put(-1)



# if __name__ == '__main__':
#     q = Queue()
#     p = Process(target=f, args=(q,))
#     p.start()

#     startTime = time.time()
#     while(True):
#         data = q.get()
#         print(time.time()-startTime, data)
        
#         if data == -1:
#             break
        
        
#     print('hello world')
#     p.join()
#     print('hello world 2')

