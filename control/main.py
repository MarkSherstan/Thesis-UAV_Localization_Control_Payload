from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import pandas as pd
import math
from controller import *
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
	# Start capture frame class and dont start until we receive a frame
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
		print(round(PF.North,2), round(PF.East,2), round(PF.Down,2), round(math.degrees(PF.Yaw),2))

	# Close the threads and any other connections
	PF.close()
	CF.close()


# Main loop
if __name__ == '__main__':
	main()
