import numpy as np
import asyncio
import time
import math

class Controller:
	def __init__(self, northDesired, eastDesired, downDesired):
		# Desired position
		self.northDesired = northDesired
		self.eastDesired = eastDesired
		self.downDesired = downDesired

		# Constraints for North, East and Yaw
		self.minValNE = -3.1415/12		# -15 Deg
		self.maxValNE = 3.1415/12	    # +15 Deg
		self.minValYaw = -3.1415/4		# -45 Deg/s
		self.maxValYaw = 3.1415/4		# +45 Deg/s

		# Controller PID Gains: NORTH
		self.kp_NORTH = 0.0
		self.ki_NORTH = 0.0
		self.kd_NORTH = 0.0

		# Controller PID Gains: EAST
		self.kp_EAST = self.kp_NORTH
		self.ki_EAST = self.ki_NORTH
		self.kd_EAST = self.kd_NORTH

		# Controller PID Gains: DOWN
		self.kp_DOWN = 0.0010
		self.ki_DOWN = 0.0005
		self.kd_DOWN = 0.0005

		# Controller PID Gains: DOWN
		self.kp_YAW = 0.0
		self.ki_YAW = 0.0
		self.kd_YAW = 0.0

		# PID variables (NED)
		self.northPrevError = 0
		self.eastPrevError = 0
		self.downPrevError = 0
		self.northI = 0
		self.eastI = 0
		self.downI = 0
		self.timer = None

		# Yaw control -> Fix this 
		self.kYaw = 2
		self.yawConstrained = [-3.1415/4, -3.1415/90, 3.1415/90, 3.1415/4] 	# [-45, -2, 2, 45] deg/s
		self.yawRateInterp  = [-3.1415/2, 0, 0, 3.1415/2]					# [-90,  0, 0, 90] deg/s	

	def constrain(self, val, minVal, maxVal):
		return max(min(maxVal, val), minVal)

	def PID(self, error, errorPrev, I, dt, kp, ki, kd, debug=False):
		# Run the PID controller
		P = error
		I = I + error * dt
		D = (error - errorPrev) / dt
		PID = (kp * P) + (ki * I) + (kd * D)

		# Debugging
		if debug is True:
			print('{:<8.0f} {:<8.2f} {:<8.2f} {:<8.2f} {:<8.2f}'.format(error, P*kp, I*ki, D*kd, (PID)))
		
		# Return values 
		return PID, I

	def positionControl(self, northActual, eastActual, downActual, yawActual):
		# Error calculations [mm]
		errorNorth = (self.northDesired - northActual)
		errorEast = (self.eastDesired - eastActual)
		errorDown = (self.downDesired - downActual)

		# Get time delta
		dt = time.time() - self.timer
		self.timer = time.time()

		# Run some control
		rollControl, self.eastI = self.PID(errorEast, self.eastPrevError, self.eastI, dt, self.kp_EAST, self.ki_EAST, self.kd_EAST)
		pitchControl, self.northI = self.PID(errorNorth, self.northPrevError, self.northI, dt, self.kp_NORTH, self.ki_NORTH, self.kd_NORTH)
		thrustControl, self.downI = self.PID(errorDown, self.downPrevError, self.downI, dt, self.kp_DOWN, self.ki_DOWN, self.kd_DOWN)
		yawControl = self.constrain(yawActual * self.kYaw, self.minValYaw, self.maxValYaw)

		# Update previous error
		self.northPrevError = errorNorth
		self.eastPrevError = errorEast
		self.downPrevError = errorDown

		# Set and constrain the controller values
		rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
		pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
		thrust = self.constrain(thrustControl, 0, 1)
		yawRate = np.interp(yawControl, self.yawConstrained, self.yawRateInterp)

		# Return the values
		return rollAngle, pitchAngle, yawRate, thrust
