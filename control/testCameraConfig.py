import cv2


cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)


            


while (True):
	try:
        _, frame = cam.read()
        # print(frame)
	except KeyboardInterrupt:
        break




print 'Frames: ', V.frameCount, 'Time: ', time.time()-tt
print 'FPS: ', V.frameCount / (time.time() - tt)
