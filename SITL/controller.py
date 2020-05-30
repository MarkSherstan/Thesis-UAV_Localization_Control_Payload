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

		# Constraints for Down
		self.minValD = -5				# [ ]
		self.maxValD =  5				# [ ]
		self.outputD = [0, 1]			# [ ]

		# Controller PID Gains
		self.kp = 0.002 		# 0.0002, 0.0025, 0.007  w/ -1,1
		self.ki = 0 
		self.kd = 0.00			# 0.0002, 0.0006 w/ -1,1

		# PID variables (NED)
		self.northPreviousError = 0
		self.eastPreviousError = 0
		self.downPreviousError = 0
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

	def PID(self, error, errorPrev, I, dt):
		# Run the PD controller
		P = self.kp * error
		I = self.ki * (I + error * dt)
		D = self.kd * ((error - errorPrev) / dt)
		return (P + I + D), I

	async def positionControl(self, northActual, eastActual, downActual, yawActual):
		# Error calculations [mm]
		errorNorth = (self.northDesired - northActual)
		errorEast = (self.eastDesired - eastActual)
		errorDown = (self.downDesired - downActual)

		# Get time delta
		dt = time.time() - self.timer
		self.timer = time.time()

		# Run some control
		rollControl, self.eastI = self.PID(errorEast, self.eastPreviousError, self.eastI, dt)
		pitchControl, self.northI = self.PID(errorNorth, self.northPreviousError, self.northI, dt)
		thrustControl, self.downI = self.PID(errorDown, self.downPreviousError, self.downI, dt)
		yawControl = self.constrain(yawActual * self.kYaw, self.minValYaw, self.maxValYaw)

		# Update previous error
		self.northPreviousError = errorNorth
		self.eastPreviousError = errorEast
		self.downPreviousError = errorDown

		# Set and constrain the controller values
		rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
		pitchAngle = self.constrain(pitchControl, self.minValNE, self.maxValNE)
		thrust = np.interp(self.constrain(thrustControl, self.minValD, self.maxValD), [self.minValD, self.maxValD], self.outputD)
		yawRate = np.interp(yawControl, self.yawConstrained, self.yawRateInterp)

		# Print values for error hacking
		print(round(1/dt), round(errorDown,2), round(thrustControl,2), round(thrust,2))

		# Return the value
		return rollAngle, pitchAngle, yawRate, thrust
