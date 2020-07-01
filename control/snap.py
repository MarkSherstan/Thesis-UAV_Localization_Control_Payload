# import numpy as np
# import cv2

# desiredWidth=1280
# desiredHeight=720
# desiredFPS=30
# cameraIdx=0

# cam = cv2.VideoCapture(0, cv2.CAP_V4L)
# cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
# cam.set(cv2.CAP_PROP_FRAME_WIDTH, desiredWidth)
# cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desiredHeight)
# cam.set(cv2.CAP_PROP_FPS, desiredFPS)
# cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)
# print('Camera start')

# for _ in range(10):
#     _, frame = cam.read()

# frame = cv2.rotate(frame, cv2.ROTATE_180)
# cv2.imwrite('100mm.png',frame)

# cam.release()


import cv2

# Camera properties
desiredWidth=1280
desiredHeight=720
desiredFPS=30

# Set capture device
cam = cv2.VideoCapture(0, cv2.CAP_V4L)
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))
cam.set(cv2.CAP_PROP_FRAME_WIDTH, desiredWidth)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desiredHeight)
cam.set(cv2.CAP_PROP_FPS, desiredFPS)
cam.set(cv2.CAP_PROP_AUTOFOCUS, 0)

# Define the codec and create VideoWriter object
fourcc = cv2.VideoWriter_fourcc('M','J','P','G')
out = cv2.VideoWriter('Flight.avi', fourcc, 30.0, (1280,720))

# Have some sort of counter 
ii = 0

try: 
    while(True):
        # Get a frame and rotate it
        _, frame = cam.read()
        frame = cv2.rotate(frame, cv2.ROTATE_180)
        
        # Write the frame
        out.write(frame)
        
        # Increment timer
        ii += 1
        print(ii)

except KeyboardInterrupt:
    # Print final remarks
    print('Closing')
    
finally:    
    # Release everything if job is finished
    cam.release()
    out.release()
    print('Closed')
