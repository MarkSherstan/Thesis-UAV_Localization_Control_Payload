from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
from controller import *
from vision import *
import pandas as pd
import datetime
import math

def dispData(V, CF, PF, C):
	# Display data to user:
	print('Actual ->\t', \
		'\tN: ', round(PF.North,2), \
		'\tE: ', round(PF.East,2), \
		'\tD: ', round(PF.Down,2), \
		'\tY: ', round(math.degrees(PF.Yaw),2))

	print('Controller ->\t', \
		'\tR: ', round(math.degrees(C.rollAngle),2), \
		'\tP: ', round(math.degrees(C.pitchAngle),2), \
		'\tY: ', round(math.degrees(C.yawRate),2), \
		'\tD: ', round(C.thrust,2))

	print('Attitude ->\t', \
	  	'\tR: ', round(math.degrees(V.attitude.roll),2), \
	  	'\tP: ', round(math.degrees(V.attitude.pitch),2), \
	  	'\tY: ', round(math.degrees(V.attitude.yaw),2), '\n')

def main():
	# Connect to the Vehicle
	connection_string = "/dev/ttyS0"
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
	C = Controller(vehicle)
	C.controllerStart()

	# Run for T secounds
	print('\nStarting...\n')
	tempData = []
	printTimer = time.time()
	logTimer = time.time()

	try:
		while(True):
			# Share data between classes and threads
			PF.frame = CF.frame

			C.North  = PF.North
			C.East   = PF.East
			C.Down   = PF.Down
			C.Yaw    = PF.Yaw

			# Print data evert second
			if (time.time() > printTimer + 1):
				dispData(vehicle, CF, PF, C)
				printTimer = time.time()

			# Log data at 50 Hz
			if (time.time() > logTimer + 0.02):
				tempData.append([vehicle.mode.name, time.time(), \
					vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw, \
					C.North, C.East, C.Down, C.Yaw, \
					C.rollAngle, C.pitchAngle, C.yawRate, C.thrust])
				logTimer = time.time()

	except KeyboardInterrupt:
		# Close the threads and any other connections
		C.close()
		PF.close()
		CF.close()
		vehicle.close()

		# Create file name
		now = datetime.datetime.now()
		fileName = now.strftime("%Y-%m-%d %H:%M:%S") + ".csv"

		# Write data to a data frame
		df = pd.DataFrame(tempData, columns=['Mode', 'Time',
							'Roll', 'Pitch', 'Yaw',
							'northCurrentPos', 'eastCurrentPos','downCurrentPos', 'yawCurrentAngle',
							'rollControl', 'pitchControl', 'yawControl', 'thrustControl'])

		# Save as CSV and display to user
		df.to_csv(fileName, index=None, header=True)
		print('File saved to:\t' + fileName)

# Main loop
if __name__ == '__main__':
	main()
