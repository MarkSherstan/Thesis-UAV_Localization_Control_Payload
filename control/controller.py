from threading import Thread
import numpy as np
import time
import math

class Controller:
	def __init__(self, drone):
		# Pass vehicle class
		self.drone = drone

		# Threading parameters
		self.isReceiving = False
		self.isRun = True
		self.thread = None
		self.North = 0
		self.East  = 0
		self.Down  = 0
		self.Yaw   = 0

		# Initial conditions
		self.rollAngle = 0.0
		self.pitchAngle = 0.0
		self.yawAngle = 0.0
		self.yawRate = 0.0
		self.thrust = 0.5
		self.duration = 0.08
		self.heading = 0

		# Constraints for roll, pitch, yaw and thrust
		self.minValNE = -3.1415/8
		self.maxValNE = 3.1415/8
		self.minValD = -1
		self.maxValD = 1
		self.minValYaw = -3.1415/4
		self.maxValYaw = 3.1415/4

		# Controller PD Gains
		self.kp = 0.3
		self.ki = 0
		self.kd = 0.2

		# PID variables
		self.northPreviousError = 0
		self.eastPreviousError = 0
		self.northI = 0
		self.eastI = 0

		# Controller
		self.northDesired = None
		self.eastDesired = None
		self.downDesired = None
		self.startTime = None

		# Thrust Control
		self.kThrottle = 0.5
		self.one2one = [-1, -0.04, 0.04, 1]
		self.zero2one = [0, 0.5, 0.5, 1]

		# Yaw control
		self.kYaw = 2
		self.yawConstrained = [-3.1415/4, -3.1415/90, 3.1415/90, 3.1415/4]
		self.yawRateInterp = [-3.1415/2, 0, 0, 3.1415/2]

	def controllerStart(self):
		# Create a thread
		if self.thread == None:
			self.thread = Thread(target=self.control)
			self.thread.start()
			print('Controller thread start')

			# Block till we start receiving values
			while (self.isReceiving != True):
				time.sleep(0.1)

	def control(self):
		# Run control until closed
		while(self.isRun):
			self.positionControl(self.drone)

	def sendAttitudeTarget(self, vehicle):
		# https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
		#
		# thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
		#         Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
		#         the code for maintaining current altitude.
		#             Thrust >  0.5: Ascend
		#             Thrust == 0.5: Hold the altitude
		#             Thrust <  0.5: Descend
		#
		# Mappings: If any of these bits are set, the corresponding input should be ignored.
		# bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate.
		# bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

		# Prevent none type error on yaw
		self.yawAngle = vehicle.attitude.yaw

		# Create the mavlink message
		msg = vehicle.message_factory.set_attitude_target_encode(
			0, # time_boot_ms
			0, # Target system
			0, # Target component
			0b00000000, # If bit is set corresponding input ignored (mappings)
			self.euler2quaternion(self.rollAngle, self.pitchAngle, self.yawAngle), # Quaternion
			0, # Body roll rate in radian
			0, # Body pitch rate in radian
			self.yawRate, # Body yaw rate in radian/second
			self.thrust # Thrust
		)

		# Send the constructed message
		vehicle.send_mavlink(msg)

	def euler2quaternion(self, roll, pitch, yaw):
		# Euler angles (rad) to quaternion
		yc = math.cos(yaw * 0.5)
		ys = math.sin(yaw * 0.5)
		rc = math.cos(roll * 0.5)
		rs = math.sin(roll * 0.5)
		pc = math.cos(pitch * 0.5)
		ps = math.sin(pitch * 0.5)

		q0 = yc * rc * pc + ys * rs * ps
		q1 = yc * rs * pc - ys * rc * ps
		q2 = yc * rc * ps + ys * rs * pc
		q3 = ys * rc * pc - yc * rs * ps

		return [q0, q1, q2, q3]

	def constrain(self, val, minVal, maxVal):
		return max(min(maxVal, val), minVal)

	def PID(self, error, errorPrev, I, dt):
		# Run the PD controller
		P = self.kp * error
		I = self.ki * (I + error * dt)
		D = self.kd * ((error - errorPrev) / dt)
		return (P + I + D), I

	def positionControl(self, vehicle):
		# Set parameters
		self.northDesired = 0.4
		self.eastDesired = 0.4
		self.downDesired = 0.4

		# Start a timer
		self.startTime = time.time()

		while(True):
			# Update thread flag
			self.isReceiving = True

			# Get current values
			northCurrentPos = self.North
			eastCurrentPos  = self.East
			downCurrentPos  = self.Down
			self.heading	= self.Yaw

			# Error calculations
			errorNorth = self.northDesired - northCurrentPos
			errorEast = self.eastDesired - eastCurrentPos
			errorDown = self.downDesired - downCurrentPos

			# Get time delta
			dt = time.time() - self.startTime

			# Run some control
			rollControl, self.eastI = self.PID(errorEast, self.eastPreviousError, self.eastI, dt)
			pitchControl, self.northI = self.PID(errorNorth, self.northPreviousError, self.northI, dt)
			yawControl = self.constrain(self.heading * self.kYaw, self.minValYaw, self.maxValYaw)
			thrustControl = self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

			# Update previous error
			self.northPreviousError = errorNorth
			self.eastPreviousError = errorEast

			# Set the controller values
			self.rollAngle = -self.constrain(rollControl, self.minValNE, self.maxValNE)
			self.pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
			self.yawRate = -np.interp(yawControl, self.yawConstrained, self.yawRateInterp)
			self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)

			# Send the command with small buffer
			self.sendAttitudeTarget(vehicle)
			time.sleep(self.duration)

			# Thread breakout protocal
			if (self.isRun is False):
				break

	# def trajectoryControl(self, vehicle, s):
	# 	# Initialize variables
	# 	T = 5			# Trajectory time
	# 	N = 50			# Total data points for initial position
	# 	counter = 0		# Counter to track trajectory location
	# 	self.startTime = time.time()
	# 	northCurrentPos = 0
	# 	eastCurrentPos = 0
	#
	# 	# Set the desired NED locations
	# 	self.northDesired = 0.4
	# 	self.eastDesired = 0.4
	# 	self.downDesired = 0.4
	#
	# 	try:
	# 		# Wait until mode has changed
	# 		while not vehicle.mode.name=='GUIDED_NOGPS':
	# 			data = [ii * 0.01 for ii in s.dataOut]
	# 			print ' Waiting for GUIDED_NOGPS', data
	# 			self.logData(vehicle, data, 0, 0)
	# 			time.sleep(0.2)
	#
	# 		# Take N readings of data at 100 Hz once GUIDED_NOGPS mode is active
	# 		for ii in range(N):
	# 			data = [ii * 0.01 for ii in s.dataOut]
	# 			northCurrentPos += (data[0] + data[1]) / 2
	# 			eastCurrentPos += (data[2] + data[3]) / 2
	# 			self.logData(vehicle, data, 0, 0)
	# 			time.sleep(0.01)
	#
	# 		# Find the average position
	# 		northCurrentPos /= N
	# 		eastCurrentPos /= N
	#
	# 		# Generate a trajectory that should take T seconds
	# 		northIC = [northCurrentPos, self.northDesired, 0, 0, 0, 0]
	# 		eastIC = [eastCurrentPos, self.eastDesired, 0, 0, 0, 0]
	#
	# 		pN = self.trajectoryGen(northIC, T, self.duration)
	# 		pE = self.trajectoryGen(eastIC, T, self.duration)
	#
	# 		# Start timer
	# 		self.startTime = time.time()
	# 		printTimer = time.time()
	# 		prevTime = time.time()
	#
	# 		# Run until stopped
	# 		while (True):
	# 			# Get current values
	# 			data = [ii * 0.01 for ii in s.dataOut]
	# 			north1 = data[0]; north2 = data[1];
	# 			east1 = data[2];  east2 = data[3];
	# 			downCurrentPos = data[4]
	#
	# 			# Average the distances
	# 			northCurrentPos = (north1 + north2) / 2
	# 			eastCurrentPos = (east1 + east2) / 2
	#
	# 			# Heading calculations
	# 			if ((abs(north1 - north2) < 0.2286) and (abs(east1 - east2) < 0.2445)):
	# 				yawNorth = -math.asin((north1 - north2) / 0.2286)
	# 				yawEast = -math.asin((east1 - east2) / 0.2445)
	# 				self.heading = (yawNorth + yawEast) / 2
	# 			elif abs(north1 - north2) < 0.2286:
	# 				self.heading = -math.asin((north1 - north2) / 0.2286)
	# 			elif abs(east1 - east2) < 0.2445:
	# 				self.heading = -math.asin((east1 - east2) / 0.2445)
	# 			else:
	# 				pass
	#
	# 			# Set the desired position based on time counter index
	# 			if counter < len(pN):
	# 				desiredN = pN[counter]
	# 				desiredE = pE[counter]
	# 			else:
	# 				desiredN = self.northDesired
	# 				desiredE = self.eastDesired
	#
	# 			# Error caculations
	# 			errorNorth = desiredN - northCurrentPos
	# 			errorEast = desiredE - eastCurrentPos
	# 			errorDown = self.downDesired - downCurrentPos
	#
	# 			# Time elapsed
	# 			dt = time.time() - prevTime
	# 			prevTime = time.time()
	#
	# 			# Run some control
	# 			rollControl, self.eastI = self.PID(errorEast, self.eastPreviousError, self.eastI, dt)
	# 			pitchControl, self.northI = self.PID(errorNorth, self.northPreviousError, self.northI, dt)
	# 			yawControl = self.constrain(self.heading * self.kYaw, self.minValYaw, self.maxValYaw)
	# 			thrustControl = self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)
	#
	# 			# Update previous error
	# 			self.northPreviousError = errorNorth
	# 			self.eastPreviousError = errorEast
	#
	# 			# Set the controller values
	# 			self.rollAngle = -self.constrain(rollControl, self.minValNE, self.maxValNE)
	# 			self.pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
	# 			self.yawRate = -np.interp(yawControl, self.yawConstrained, self.yawRateInterp)
	# 			self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)
	#
	# 			# Print data every 0.75 seconds
	# 			if (time.time() > printTimer + 0.75):
	# 				print 'N: ', round(north1, 2), round(north2, 2), round(northCurrentPos, 2)
	# 				print 'E: ', round(east1, 2), round(east2, 2), round(eastCurrentPos, 2)
	# 				print 'D: ', round(downCurrentPos, 2)
	# 				print 'Heading: ', round(math.degrees(self.heading), 2)
	#
	# 				print 'Roll: ', round(math.degrees(self.rollAngle), 2), round(math.degrees(vehicle.attitude.roll), 2)
	# 				print 'Pitch: ', round(math.degrees(self.pitchAngle), 2), round(math.degrees(vehicle.attitude.pitch), 2)
	# 				print 'Yaw: ', round(math.degrees(self.yawRate), 2), round(math.degrees(vehicle.attitude.yaw), 2), vehicle.heading
	# 				print 'Thrust: ', round(self.thrust, 2), '\n'
	#
	# 				printTimer = time.time()
	#
	# 			# Send the command, sleep, and increase counter
	# 			self.sendAttitudeTarget(vehicle)
	# 			self.logData(vehicle, data, desiredN, desiredE)
	# 			time.sleep(self.duration)
	# 			counter += 1
	#
	# 	except KeyboardInterrupt:
	# 		# Close thread and serial connection
	# 		s.close()
	#
	# 		# Create file name
	# 		now = datetime.datetime.now()
	# 		fileName = now.strftime("%Y-%m-%d %H:%M:%S") + ".csv"
	#
	# 		# Write data to CSV and display to user
	# 		df = pd.DataFrame(self.tempData, columns=['Mode', 'Time', 'YawSP', 'Roll', 'Pitch', 'Yaw', 'Heading',
	# 			'N1', 'N2', 'E1', 'E2', 'D', 'northCurrentPos', 'eastCurrentPos', 'calcHeading',
	# 			'desiredN', 'desiredE', 'desiredD', 'rollInput', 'pitchInput', 'yawInput', 'thrustInput'])
	#
	# 		df.to_csv(fileName, index=None, header=True)
	#
	# 		print('File saved to:\t' + fileName)
	
	# def trajectoryGen(self, IC, T, sampleRate):
	# 	# Define time array and storage variables
	# 	tt = np.linspace(0, T, round(T/sampleRate), endpoint=True)
	# 	pos = []; vel = []; acc = [];
	#
	# 	# Find coeffcients of 5th order polynomial using matrix operations
	# 	A = np.array([[0, 0, 0, 0, 0, 1],
	# 				[np.power(T,5), np.power(T,4), np.power(T,3), np.power(T,2), T, 1],
	# 				[0, 0, 0, 0, 1, 0],
	# 				[5*np.power(T,4), 4*np.power(T,3), 3*np.power(T,2), 2*T, 1, 0],
	# 				[0, 0, 0, 2, 0, 0],
	# 				[20*np.power(T,3), 12*np.power(T,2), 6*T, 2, 0, 0]])
	#
	# 	b = np.array([IC[0], IC[1], IC[2], IC[3], IC[4], IC[5]])
	#
	# 	x = np.linalg.solve(A, b)
	#
	# 	# Unpack coeffcients
	# 	A = x[0]; B = x[1]; C = x[2]; D = x[3]; E = x[4]; F = x[5];
	#
	# 	# Calculate the trajectory properties for each time step and store
	# 	for t in tt:
	# 		pos.append(A*np.power(t,5) + B*np.power(t,4) + C*np.power(t,3) + D*np.power(t,2) + E*t + F)
	# 		vel.append(5*A*np.power(t,4) + 4*B*np.power(t,3) + 3*C*np.power(t,2) + 2*D*t + E)
	# 		acc.append(20*A*np.power(t,3) + 12*B*np.power(t,2) + 6*C*t + 2*D)
	#
	# 	# Return the resulting position
	# 	return pos

	def close(self):
		# Close the processing thread
		self.isRun = False
		self.thread.join()
		print('Controller thread closed')
