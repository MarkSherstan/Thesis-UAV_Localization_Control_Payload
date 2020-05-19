from multiprocessing import Process, Queue
import queue
import time
# from VisionMultiCore import *
import cv2


# def processFrame(q):
#     # Start the connection to the camera
#     # try:
#     #     # self.cam = cv2.VideoCapture(self.cameraIdx, cv2.CAP_V4L)
#     #     # self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
#     #     cam = cv2.VideoCapture(0)
#     #     cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
#     #     cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
#     #     cam.set(cv2.CAP_PROP_FPS, 30)
#     #     cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
#     #     print('Camera start')
#     # except:
#     #     print('Camera setup failed')

#     counter = 0
    
#     # Process data until closed
#     while(True):
#         # Check if we can end the process
#         try:
#             data = q.get(False)  
#             if data == -1:
#                 break
#         except queue.Empty:
#             pass
        
#         # Capture a frame
#         # _, frame = cam.read()

#         # # Process the frame
#         # self.getPose()

#         # some infinite data    
#         counter += 1
#         q.put(counter)

#     # Release the camera connection
#     # cam.release()
#     # print('Camera closed')


# def main():    
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
#             print('blocking')
#             time.sleep(1)  
#         except queue.Empty:
#             data = None

#     # Run test
#     print('Test running for 1 secound\n')
#     startTime = time.time()

#     while (time.time() < startTime+1):
#         print(q.get())

#     q.put(-1)
#     q.put(-1)


#     p.join()

#     print('Complete')
#     exit()

        
def f(q):
    # Camera set up 
    cam = cv2.VideoCapture(0, cv2.CAP_V4L)
    cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cam.set(cv2.CAP_PROP_FPS, 30)
    cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    
    # Initialize
    counter = 0

    # loop until broken 
    while(True):
        # Check if we can end the process
        # try:
        #     data = q.get(False)  
        #     if data == -1:
        #         break
        # except queue.Empty:
        #     pass
        
        # data
        _, frame = cam.read()
        q.put(frame)

        # some infinite data    
        # counter += 1
        # q.put(counter)

    # end 
    cam.release()

def main():
    q = Queue()
    p = Process(target=f, args=(q,))
    p.start()

    startTime = time.time()
    while(time.time() < startTime+10):
        data = q.get()
        actualHeight, actualWidth, _ = data.shape  
        print(actualHeight, actualWidth)
        time.sleep(0.5)
    
    # for ii in range(10):
    q.put(-1)
    
    print('hello world')
    p.join()
    print('hello world 2')
    

# Main loop
if __name__ == '__main__':
    main()
