from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
from controller import *
from vision import *
import pandas as pd
import datetime
import math

def dispData(V, C, vehicle):
	# Display data to user:
	print('Vision ->\t', \
		'\tN: ', round(V.North,1), \
		'\tE: ', round(V.East,1), \
		'\tD: ', round(V.Down,1), \
		'\tY: ', round(math.degrees(V.Yaw),1))

	print('Controller ->\t', \
		'\tR: ', round(math.degrees(C.rollAngle),1), \
		'\tP: ', round(math.degrees(C.pitchAngle),1), \
		'\tY: ', round(math.degrees(C.yawRate),1), \
		'\tT: ', round(C.thrust,2))

	print('Attitude ->\t', \
	  	'\tR: ', round(math.degrees(vehicle.attitude.roll),1), \
	  	'\tP: ', round(math.degrees(vehicle.attitude.pitch),1), \
	  	'\tY: ', round(math.degrees(vehicle.attitude.yaw),1), '\n')

def main():
	# Flags and data rates
	printFlag = True
	printRate = 0.75
	logFlag   = True
	logRate   = 1/60

	# Connect to the Vehicle
	connection_string = "/dev/ttyS0"
	print('Connecting to vehicle on: %s\n' % connection_string)
	vehicle = connect(connection_string, wait_ready=["attitude"], baud=57600)

	# Start vision class and the capture and pose threads
	V = Vision()
	V.startFrameThread()
	V.startPoseThread()

	# Do not proceed until vision processor is ready
	while(V.isReady != True):
		time.sleep(0.1)

	# Start controller and thread
	northDesired = 200
	eastDesired  = 0
	downDesired  = 0
	C = Controller(vehicle, northDesired, eastDesired, downDesired)
	C.controllerStart()

	# Run until broken by user
	print('\nStarting...\n')
	tempData = []
	printTimer = time.time()
	logTimer = time.time()

	try:
		while(True):
			# Share data between the two classes
			C.North = V.North
			C.East  = V.East
			C.Down  = V.Down
			C.Yaw   = V.Yaw

			# Print data
			if (time.time() > printTimer + printRate) and (printFlag is True):
				dispData(V, C, vehicle)
				printTimer = time.time()

			# Log data
			if (time.time() > logTimer + logRate) and (logFlag is True):
				tempData.append([vehicle.mode.name, time.time(), \
					math.degrees(vehicle.attitude.roll), math.degrees(vehicle.attitude.pitch), math.degrees(vehicle.attitude.yaw), \
					C.North, C.East, C.Down, math.degrees(C.Yaw), \
					northDesired, eastDesired, downDesired, \
					math.degrees(C.rollAngle), math.degrees(C.pitchAngle), math.degrees(C.yawRate), C.thrust])
				logTimer = time.time()

	except KeyboardInterrupt:
		# Close the threads and any other connections
		C.close()
		V.close()
		vehicle.close()

		# Write data to a file based on flag
		if (logFlag is True):
			# Create file name
			now = datetime.datetime.now()
			fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"

			# Write data to a data frame
			df = pd.DataFrame(tempData, columns=['Mode', 'Time',
								'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
								'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
								'North-Desired', 'East-Desired', 'Down-Desired',
								'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control'])

			# Save as CSV and display to user
			df.to_csv(fileName, index=None, header=True)
			print('File saved to:' + fileName)

# Main loop
if __name__ == '__main__':
	main()
