from pymavlink import mavutil
from threading import Thread
import numpy as np
import time
import math

class Controller:
	def __init__(self, northDesired, eastDesired, downDesired):
		# Desired position
		self.northDesired = northDesired
		self.eastDesired = eastDesired
		self.downDesired = downDesired

		# Real time data from vision class
		self.North = 0
		self.East  = 0
		self.Down  = 0
		self.Yaw   = 0

		# Initial conditions
		self.rollAngle = 0.0
		self.pitchAngle = 0.0
		self.yawAngle = 0.0
		self.thrust = 0.5

		# Constraints for roll, pitch, yaw and thrust
		self.minValNE = -3.1415/12		# - 15 Degrees
		self.maxValNE = 3.1415/12	    # + 15 Degrees
		self.minValD = -1				# 
		self.maxValD = 1				#
		self.minValYaw = -3.1415/4		#
		self.maxValYaw = 3.1415/4		#

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
