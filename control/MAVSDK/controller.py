import numpy as np
import time
import math

class Controller:
	def __init__(self, northDesired, eastDesired, downDesired):
		# Desired position
		self.northDesired = northDesired
		self.eastDesired = eastDesired
		self.downDesired = downDesired

		# Constraints for roll, pitch, yaw and thrust
		self.minValNE = -3.1415/12		# -15 Deg
		self.maxValNE = 3.1415/12	    # +15 Deg
		self.minValD = -1				# m
		self.maxValD = 1				# m
		self.minValYaw = -3.1415/4		# -45 Deg/s
		self.maxValYaw = 3.1415/4		# +45 Deg/s

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
		self.yawConstrained = [-3.1415/4, -3.1415/90, 3.1415/90, 3.1415/4] 	# [-45, -2, 2, 45] deg/s
		self.yawRateInterp  = [-3.1415/2, 0, 0, 3.1415/2]					# [-90,  0, 0, 90] deg/s	

	def constrain(self, val, minVal, maxVal):
		return max(min(maxVal, val), minVal)

	def PID(self, error, errorPrev, I, dt):
		# Run the PD controller
		P = self.kp * error
		I = self.ki * (I + error * dt)
		D = self.kd * ((error - errorPrev) / dt)
		return (P + I + D), I

	def positionControl(self, northActual, eastActual, downActual, yawActual):
		# Error calculations and conversion to meters
		errorNorth = (self.northDesired - northActual) * 0.01
		errorEast = (self.eastDesired - eastActual) * 0.01
		errorDown = (self.downDesired - downActual) * 0.01

		# Get time delta
		dt = time.time() - self.startTime

		# Run some control
		rollControl, self.eastI = self.PID(errorEast, self.eastPreviousError, self.eastI, dt)
		pitchControl, self.northI = self.PID(errorNorth, self.northPreviousError, self.northI, dt)
		yawControl = self.constrain(yawActual * self.kYaw, self.minValYaw, self.maxValYaw)
		thrustControl = -self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

		# Update previous error
		self.northPreviousError = errorNorth
		self.eastPreviousError = errorEast

		# Set the controller values
		rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
		pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
		yawRate = np.interp(yawControl, self.yawConstrained, self.yawRateInterp)
		thrust = np.interp(thrustControl, self.one2one, self.zero2one)

		# Return the value
		return rollAngle, pitchAngle, yawRate, thrust