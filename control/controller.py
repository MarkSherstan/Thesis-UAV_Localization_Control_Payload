from threading import Thread
import numpy as np
import time
import math

class Controller:
	def __init__(self, vehicle, northDesired, eastDesired, downDesired):
		# Connect to the vehicle class
		self.UAV = vehicle

		# Desired position
		self.northDesired = northDesired
		self.eastDesired = eastDesired
		self.downDesired = downDesired

		# Threading parameters
		self.isReceiving = False
		self.isRun = True
		self.thread = None

		# Real time data from vision class
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

		# Update rate to flight controller
		self.duration = 1/30

		# Constraints for roll, pitch, yaw and thrust
		self.minValNE = -3.1415/12
		self.maxValNE = 3.1415/12
		self.minValD = -1
		self.maxValD = 1
		self.minValYaw = -3.1415/4
		self.maxValYaw = 3.1415/4

		# Controller PID Gains
		self.kp = 0.3
		self.ki = 0
		self.kd = 0.2

		# PID variables
		self.northPreviousError = 0
		self.eastPreviousError = 0
		self.northI = 0
		self.eastI = 0
		self.startTime = None

		# Thrust Control
		self.kThrottle = 0.5
		self.one2one = [-1, -0.04, 0.04, 1]
		self.zero2one = [0, 0.5, 0.5, 1]

		# Yaw control
		self.kYaw = 2
		self.yawConstrained = [-3.1415/4, -3.1415/90, 3.1415/90, 3.1415/4]
		self.yawRateInterp  = [-3.1415/2, 0, 0, 3.1415/2]

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
		# Start a timer
		self.startTime = time.time()

		# Run control until closed
		while(self.isRun):
			self.positionControl()
			self.isReceiving = True

	def sendAttitudeTarget(self):
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
		self.yawAngle = self.UAV.attitude.yaw

		# Create the mavlink message
		msg = self.UAV.message_factory.set_attitude_target_encode(
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
		self.UAV.send_mavlink(msg)

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

	def positionControl(self):
		# Error calculations and conversion to meters
		errorNorth = (self.northDesired - self.North) * 0.01
		errorEast = (self.eastDesired - self.East) * 0.01
		errorDown = (self.downDesired - self.Down) * 0.01

		# Get time delta
		dt = time.time() - self.startTime

		# Run some control
		rollControl, self.eastI = self.PID(errorEast, self.eastPreviousError, self.eastI, dt)
		pitchControl, self.northI = self.PID(errorNorth, self.northPreviousError, self.northI, dt)
		yawControl = self.constrain(self.Yaw * self.kYaw, self.minValYaw, self.maxValYaw)
		thrustControl = -self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

		# Update previous error
		self.northPreviousError = errorNorth
		self.eastPreviousError = errorEast

		# Set the controller values
		self.rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
		self.pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
		self.yawRate = np.interp(yawControl, self.yawConstrained, self.yawRateInterp)
		self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)

		# Send the command with small buffer
		self.sendAttitudeTarget()
		time.sleep(self.duration)

	def close(self):
		# Close the processing thread
		self.isRun = False
		self.thread.join()
		print('Controller thread closed')
