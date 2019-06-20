from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
from threading import Thread
import pandas as pd
import numpy as np
import datetime
import time
import math
import struct
import copy
import serial

class Controller:
	def __init__(self):
		# Initial conditions (rads)
		self.rollAngle = 0.0
		self.pitchAngle = 0.0
		self.yawAngle = None
		self.yawRate = 0.0
		self.useYawRate = True
		self.thrust = 0.5
		self.duration = 0.1

		# Constraints for roll, pitch, and thrust
		self.minValNE = -3.1415/12
		self.maxValNE = 3.1415/12
		self.minValD = -1
		self.maxValD = 1

		# Controller PD Gains
		self.kp = 0.2
		self.kd = 5.4

		# Controller
		self.northDesired = None
		self.northPreviousPos = None
		self.eastDesired = None
		self.eastPreviousPos = None
		self.downDesired = None
		self.startTime = None

		# Thrust Control
		self.kThrottle = 0.5
		self.one2one = [-1,0,1]
		self.zero2one = [0, 0.25, 0.5, 0.75, 1]

		# Data Logging
		self.tempData = []

	def sendAttitudeTarget(self, vehicle):
		# https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
		#
		# useYawRate: the yaw can be controlled using yawAngle OR yawRate.
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

	def setAttitude(self, vehicle):
		# Send the command
		self.sendAttitudeTarget(vehicle)

		# SET_ATTITUDE_TARGET has a timeout of 1s for Arducopter 3.3 and higher so keep it running
		start = time.time()
		while time.time() - start < self.duration:
			self.sendAttitudeTarget(vehicle)
			time.sleep(0.025)

		# Print the attitude (and set point)
		print 'Roll: ', round(math.degrees(vehicle.attitude.roll),3), \
			'\tPitch: ', round(math.degrees(vehicle.attitude.pitch),3), \
			'\tYaw: ', round(math.degrees(vehicle.attitude.yaw),3), \
			'\tYaw SP', round(math.degrees(self.yawAngle),3)

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

	def PD(self, error, current, previous, deltaT):
		# Run the PD controller
		P = self.kp * error
		D = self.kd * ((current - previous) / deltaT)
		return(P + D)

	def positionControl(self, vehicle, s):
		# Set parameters
		self.northDesired = 0.4
		self.eastDesired = 0.4
		self.downDesired = 0.4
		self.startTime = time.time()

		try:
			while(True):
				# Get current values
				north1 = s.dataOut[0] * 0.01
				north2 = s.dataOut[1] * 0.01
				east1 = s.dataOut[2] * 0.01
				east2 = s.dataOut[3] * 0.01
				downCurrentPos = s.dataOut[4] * 0.01
				deltaT = time.time() - self.startTime

				# Average the distances
				northCurrentPos = (north1 + north2) / 2
				eastCurrentPos = (east1 + east2) / 2

				# Angle calculations
				if abs(north1 - north2) < 0.2286:
					yawNorth = -math.asin((north1 - north2) / 0.2286)
				else:
					print 'North Error'
					print round(north1, 2), round(north2, 2)
					continue

				if abs(east1 - east2) < 0.2445:
					yawEast = -math.asin((east1 - east2) / 0.2445)
				else:
					print 'East Error'
					print round(east1, 2), round(east2, 2)
					continue

				yawAvg = (yawNorth + yawEast) / 2

				# Print data
				print 'N: ', round(north1, 2), round(north2, 2), round(northCurrentPos, 2)
				print 'E: ', round(east1, 2), round(east2, 2), round(eastCurrentPos, 2)
				print 'D: ', round(downCurrentPos, 2)
				print 'Angle: ', round(math.degrees(yawNorth),2), round(math.degrees(yawEast),2), round(math.degrees(yawAvg),2)
				print 'Yaw sign: ', math.degrees(vehicle.attitude.yaw), '\n'

				# yaw rate in rad/s
				time.sleep(0.75)

				continue

				# Error caculations
				errorNorth = northCurrentPos - self.northDesired
				errorEast = eastCurrentPos - self.eastDesired
				errorDown = downCurrentPos - self.downDesired

				# Update previous position(s) if none
				if self.northPreviousPos is None:
					self.northPreviousPos = northCurrentPos

				if self.eastPreviousPos is None:
					self.eastPreviousPos = eastCurrentPos

				# Run some control
				pitchControl = self.PD(errorNorth, northCurrentPos, self.northPreviousPos, deltaT)
				rollControl = self.PD(errorEast, eastCurrentPos, self.eastPreviousPos, deltaT)
				thrustControl = -self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

				# Save the previous position
				self.northPreviousPos = northCurrentPos
				self.eastPreviousPos = eastCurrentPos

				# Calculate the control
				self.pitchAngle = -self.constrain(pitchControl, self.minValNE, self.maxValNE)
				self.rollAngle = self.constrain(rollControl, self.minValNE, self.maxValNE)
				self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)

				# Command the controller to execute
				self.setAttitude(vehicle)

				# Display data to user
				# Actual
				print 'Actual ->\t', \
					'\tN: ', round(northCurrentPos,2), \
					'\tE: ', round(eastCurrentPos,2), \
					'\tD: ', round(downCurrentPos,2)
				# Error
				print 'Error ->\t', \
					'\tN: ', round(errorNorth,2), \
					'\tE: ', round(errorEast,2), \
					'\tD: ', round(errorDown,2)
				# Controller
				print 'Controller ->\t', \
					'\tN: ', round(self.pitchAngle,2), \
					'\tE: ', round(self.rollAngle,2), \
					'\tD: ', round(self.thrust,2)
				print(" ")

				# Log data
				self.tempData.append([vehicle.mode.name, (time.time() - self.startTime), self.yawAngle, \
					vehicle.attitude.roll, vehicle.attitude.pitch, vehicle.attitude.yaw, \
					northCurrentPos, eastCurrentPos, downCurrentPos, \
					errorNorth, errorEast, errorDown, \
					self.pitchAngle, self.rollAngle, self.thrust])

		except KeyboardInterrupt:
			# Close thread and serial connection
			s.close()

			# Create file name
			now = datetime.datetime.now()
			fileName = now.strftime("%Y-%m-%d %H:%M:%S") + ".csv"

			# Write data to CSV and display to user
			df = pd.DataFrame(self.tempData, columns=['Mode', 'Time', 'Yaw_SP', 'Roll', 'Pitch', 'Yaw', 'northCurrentPos', 'eastCurrentPos',
									   'downCurrentPos', 'errorNorth', 'errorEast', 'errorDown', 'self.pitchAngle', 'self.rollAngle', 'self.thrust'])

			df.to_csv(fileName, index=None, header=True)

			print('File saved to:\t' + fileName)

	def altitudeTest(self, vehicle, s):
		# Set desired parameters
		self.downDesired = 0.4
		self.Angle = [-3.1415/8, 0, 0, 3.1415/8]
		self.startTime = time.time()

		try:
			while(True):
				# Get current values
				rollRC = vehicle.channels['2']
				pitchRC = vehicle.channels['3']
				downCurrentPos = s.dataOut[4] * 0.01

				# Run thrust calculations
				errorDown = downCurrentPos - self.downDesired
				thrustControl = -self.constrain(errorDown * self.kThrottle, self.minValD, self.maxValD)

				# Set controller input
				self.thrust = np.interp(thrustControl, self.one2one, self.zero2one)
				self.rollAngle = -np.interp(rollRC, [1018, 1500, 1560, 2006], self.Angle)
				self.pitchAngle = -np.interp(pitchRC, [982, 1440, 1500, 1986], self.Angle)

				# Send the command with small buffer
				self.sendAttitudeTarget(vehicle)
				time.sleep(0.08)

				# Print data to the user
				print 'Roll RC: ', rollRC, ' Angle: ', round(math.degrees(self.rollAngle),1), round(math.degrees(vehicle.attitude.roll),1)
				print 'Pitch RC: ', pitchRC, ' Angle: ', round(math.degrees(self.pitchAngle),1), round(math.degrees(vehicle.attitude.pitch),1)
				print 'Down: ', downCurrentPos, ' Thrust: ', self.thrust, '\n'

				# Log data
				self.tempData.append([vehicle.mode.name, (time.time() - self.startTime), \
					rollRC, self.rollAngle, vehicle.attitude.roll,
					pitchRC, self.pitchAngle, vehicle.attitude.pitch,
					self.thrust, downCurrentPos])

		except KeyboardInterrupt:
			# Close thread and serial connection
			s.close()

			# Create file name
			now = datetime.datetime.now()
			fileName = now.strftime("%Y-%m-%d %H:%M:%S") + ".csv"

			# Write data to CSV and display to user
			df = pd.DataFrame(self.tempData, columns=['Mode', 'Time', 'RC Roll', 'Roll Control', 'Roll Actual',
				'RC Pitch', 'Pitch Control', 'Pitch Actual', 'Thrust', 'Down Pos'])

			df.to_csv(fileName, index=None, header=True)

			print('File saved to:\t' + fileName)

	def deskTest(self, vehicle):
		# Set desired parameters
		self.Angle = [-3.1415/8, 0, 0, 3.1415/8]
		self.startTime = time.time()
		printTimer = time.time()

		try:
			while(True):
				# Get current values
				thrustRC = vehicle.channels['1']
				rollRC = vehicle.channels['2']
				pitchRC = vehicle.channels['3']

				# Set controller input
				self.thrust = np.interp(thrustRC, [982, 2006], [0, 1])
				self.rollAngle = -np.interp(rollRC, [1018, 1500, 1560, 2006], self.Angle)
				self.pitchAngle = -np.interp(pitchRC, [982, 1440, 1500, 1986], self.Angle)

				# Send the command with small buffer
				self.sendAttitudeTarget(vehicle)
				time.sleep(0.08)

				# Print data to the user every half second
				if time.time() > printTimer + 0.5:
					print 'Thrust RC: ', thrustRC, ' Input: ', round(self.thrust,3)
					print 'Roll RC: ', rollRC, ' Input: ', round(math.degrees(self.rollAngle),1)
					print 'Pitch RC: ', pitchRC, ' Input: ', round(math.degrees(self.pitchAngle),1), '\n'
					printTimer = time.time()

		except KeyboardInterrupt:
			# Close thread and serial connection
			print 'Closing...\n'

	def commandTest(self, vehicle):
		# Wait to be in the correct mode
		while (vehicle.mode.name != 'GUIDED_NOGPS'):
			print('Waiting for mode')
			time.sleep(1)

		# Set thrust to a hover
		self.thrust = 0.5

		# Run twice
		for ii in range(2):
			# 15 Degrees
			self.rollAngle = math.radians(15)
			print('Starting: ',math.degrees(self.rollAngle))
			for ii in range(5):
				self.sendAttitudeTarget(vehicle)
				print('\t',math.degrees(self.rollAngle))
				time.sleep(0.2)

			# 0 Degrees
			self.rollAngle = 0
			print('Starting: ',math.degrees(self.rollAngle))
			for ii in range(5):
				self.sendAttitudeTarget(vehicle)
				print('\t',math.degrees(self.rollAngle))
				time.sleep(0.2)

			# -15 Degrees
			self.rollAngle = math.radians(-15)
			print('Starting: ',math.degrees(self.rollAngle))
			for ii in range(5):
				self.sendAttitudeTarget(vehicle)
				print('\t',math.degrees(self.rollAngle))
				time.sleep(0.2)

			# 0 Degrees
			self.rollAngle = 0
			print('Starting: ',math.degrees(self.rollAngle))
			for ii in range(5):
				self.sendAttitudeTarget(vehicle)
				print('\t',math.degrees(self.rollAngle))
				time.sleep(0.2)

class DAQ:
	def __init__(self, serialPort, serialBaud, dataNumBytes, numSignals):
		# Class / object / constructor setup
		self.port = serialPort
		self.baud = serialBaud
		self.dataNumBytes = dataNumBytes
		self.numSignals = numSignals
		self.rawData = bytearray(numSignals * dataNumBytes)
		self.dataType = 'h'
		self.isRun = True
		self.isReceiving = False
		self.thread = None
		self.dataOut = []

		# Connect to serial port
		print('Trying to connect to: ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
		try:
			self.serialConnection = serial.Serial(serialPort, serialBaud, timeout=4)
			print('Connected to ' + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')
		except:
			print("Failed to connect with " + str(serialPort) + ' at ' + str(serialBaud) + ' BAUD.')

	def readSerialStart(self):
		# Create a thread
		if self.thread == None:
			self.thread = Thread(target=self.backgroundThread)
			self.thread.start()

			# Block till we start receiving values
			while self.isReceiving != True:
				time.sleep(0.1)

	def backgroundThread(self):
		# Pause and clear buffer to start with a good connection
		time.sleep(2)
		self.serialConnection.reset_input_buffer()

		# Read until closed
		while (self.isRun):
			self.getSerialData()
			self.isReceiving = True

	def getSerialData(self):
		# Initialize data out
		tempData = []

		# Check for header bytes and then read bytearray if header satisfied
		if (struct.unpack('B', self.serialConnection.read())[0] is 0x9F) and (struct.unpack('B', self.serialConnection.read())[0] is 0x6E):
			self.rawData = self.serialConnection.read(self.numSignals * self.dataNumBytes)

			# Copy raw data to new variable and set up the data out variable
			privateData = copy.deepcopy(self.rawData[:])

			# Loop through all the signals and decode the values
			for i in range(self.numSignals):
				data = privateData[(i*self.dataNumBytes):(self.dataNumBytes + i*self.dataNumBytes)]
				value, = struct.unpack(self.dataType, data)
				tempData.append(value)

		# Check if data is usable otherwise repeat (recursive function)
		if tempData:
			self.dataOut = tempData
		else:
			return self.getSerialData()

	def close(self):
		# Close the serial port connection
		self.isRun = False
		self.thread.join()
		self.serialConnection.close()

		print('Disconnected from Arduino...')

def main():
	# Connect to the Vehicle
	connection_string = "/dev/ttyS1"
	print('Connecting to vehicle on: %s\n' % connection_string)
	vehicle = connect(connection_string, wait_ready=["attitude"], baud=57600)

	# Connect to serial port Arduino
	portName = '/dev/ttyACM0'
	baudRate = 9600
	dataNumBytes = 2
	numSignals = 5

	# Set up serial port class
	# s = DAQ(portName, baudRate, dataNumBytes, numSignals)
	# s.readSerialStart()

	# Set up controller class
	C = Controller()

	# Run a test
	# C.altitudeTest(vehicle, s)
	# C.positionControl(vehicle, s)
	# C.commandTest(vehicle)
	C.deskTest(vehicle)

# Main loop
if __name__ == '__main__':
	main()
