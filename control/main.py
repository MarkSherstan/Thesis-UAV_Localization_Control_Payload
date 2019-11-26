from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import pandas as pd
import math
from controller import *
from vision import *


def main():
	# Connect to the Vehicle
	connection_string = "/dev/ttyS1"
	print('Connecting to vehicle on: %s\n' % connection_string)
	vehicle = connect(connection_string, wait_ready=["attitude"], baud=57600)

	# Start capture frame class and thread
	CF = CaptureFrame()
	CF.acquireFrameStart()

	# Do not proceed until there is a frame
	while(CF.frame is None):
		time.sleep(0.1)

	# Start processing frame class and thread
	PF = ProcessFrame(CF.frame)
	PF.processFrameStart()

	# Do not proceed until vision processor is ready
	while(PF.isReady != True):
		time.sleep(0.1)

	# Start controller and thread
	c = Controller(vehicle)
	c.control()


	# Run for T secounds
	T = int(input('Input time to run: '))

	tt = time.time()
	while time.time() < tt + T:
		PF.frame = CF.frame
		print(round(PF.North,2), round(PF.East,2), round(PF.Down,2), round(math.degrees(PF.Yaw),2))

	# Close the threads and any other connections
	C.close()
	PF.close()
	CF.close()
	vehicle.close()

# Main loop
if __name__ == '__main__':
	main()
