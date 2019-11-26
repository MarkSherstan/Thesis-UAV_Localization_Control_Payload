from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
from controller import *
from vision import *
import math

def logData(V, CF, PF, C):
	# Display data to user:
	# Actual
	print('Actual ->\t', \
		'\tN: ', round(PF.North,2), \
		'\tE: ', round(PF.East,2), \
		'\tD: ', round(PF.Down,2), \
		'\tY: ', round(PF.Yaw,2))
	# Controller
	print('Controller ->\t', \
		'\tN: ', round(C.pitchAngle,2), \
		'\tE: ', round(C.rollAngle,2), \
		'\tD: ', round(C.thrust,2), \
		'\tY: ', round(C.yawRate))
	# Print actual roll and pitch
	print('Attitude ->\t', \
	  	'\tR: ', round(math.degrees(V.attitude.roll),2), \
	  	'\tP: ', round(math.degrees(V.attitude.pitch),2), \
	  	'\tY: ', round(math.degrees(V.attitude.yaw),2), '\n')

	# # Log data
	# self.tempData.append([vehicle.mode.name, (time.time() - self.startTime), \
	# 	vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw, \
	# 	northCurrentPos, eastCurrentPos, downCurrentPos, self.heading, \
	# 	errorNorth, errorEast, errorDown, self.heading, \
	# 	self.rollAngle, self.pitchAngle, self.yawRate, self.thrust])
	#
	# # Create file name
	# now = datetime.datetime.now()
	# fileName = now.strftime("%Y-%m-%d %H:%M:%S") + ".csv"
	#
	# # Write data to a data frame
	# df = pd.DataFrame(self.tempData, columns=['Mode', 'Time',
	# 					'Roll', 'Pitch', 'Yaw',
	# 					'northCurrentPos', 'eastCurrentPos','downCurrentPos', 'yawCurrentAngle',
	# 					'errorNorth', 'errorEast', 'errorDown', 'errorYaw',
	# 					'rollControl', 'pitchControl', 'yawControl', 'thrustControl'])
	#
	# # Save as CSV and display to user
	# df.to_csv(fileName, index=None, header=True)
	# print('File saved to:\t' + fileName)

def main():
	# Connect to the Vehicle
	connection_string = "/dev/cu.usbserial-DN04J8B4"
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
	T = int(input('Input time to run: '))

	tt = time.time()
	printTimer = time.time()

	while time.time() < tt + T:
		# Share data between classes and threads
		PF.frame = CF.frame

		C.North  = PF.North
		C.East   = PF.East
		C.Down   = PF.Down
		C.Yaw    = PF.Yaw

		# Print data
		if (time.time() > printTimer + 0.75):
			logData(vehicle, CF, PF, C)
			printTimer = time.time()

	# Close the threads and any other connections
	C.close()
	PF.close()
	CF.close()
	vehicle.close()

# Main loop
if __name__ == '__main__':
	main()
