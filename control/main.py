from vision import *

# import threading

# # import threading
# # import queue
# # import time
# # import numpy as np
#
#
#
# def worker(num):
#     """thread worker function"""
#     print 'Worker: %s' % num
#     return
#
#
#     t = threading.Thread(target=worker, args=(i,))
#     t.start()



def main():
	# Start capture frame class and get a frame
	CF = CaptureFrame()
	CF.acquireFrameStart()

	while (CF.frame is None):
		time.sleep(0.1)

	# Start processing frame class
	PF = ProcessFrame(CF.frame)
	PF.processFrameStart()

	# Run for T secounds
	T = int(input('Input time to run: '))

	tt = time.time()
	while time.time() < tt + T:
		PF.frame = CF.frame

	# Close the threads and other connections
	PF.close()
	CF.close()


# Main loop
if __name__ == '__main__':
	main()
