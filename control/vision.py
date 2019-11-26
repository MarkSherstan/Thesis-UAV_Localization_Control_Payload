from threading import Thread
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

class CaptureFrame:
	def __init__(self):
		self.isReceiving = False
		self.isRun = True
		self.thread = None
		self.frame = None

		try:
			self.cam = cv2.VideoCapture(0)
			self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
			self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
			print('Camera start')
		except:
			print('Camera setup failed')

	def acquireFrameStart(self):
		# Create a thread
		if self.thread == None:
			self.thread = Thread(target=self.acquireFrame)
			self.thread.start()
			print('Camera thread start')

			# Block till we start receiving values
			while self.isReceiving != True:
				time.sleep(0.1)

	def acquireFrame(self):
		# Acquire until closed
		while(self.isRun):
			_, self.frame = self.cam.read()
			self.isReceiving = True

	def close(self):
		# Close the thread and camera connection
		self.isRun = False
		self.thread.join()
		print('Camera thread closed')
		self.cam.release()
		print('Camera closed')

class ProcessFrame:
	def __init__(self, firstFrame):
		# Threading parameters
		self.isReceiving = False
		self.isRun = True
		self.thread = None
		self.frame = firstFrame

		# Aruco dictionary to be used
		self.arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

		# Processing parameters
		self.parm = aruco.DetectorParameters_create()
		self.parm.adaptiveThreshConstant = 10

		# Calibration values
		self.mtx = np.array([[1.01477877e+03, 0.00000000e+00, 6.59914048e+02], [0.00000000e+00, 1.01195343e+03, 3.81079689e+02], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
		self.dist = np.array([[ 1.51665143e-01, 9.43194746e-01, 1.00876376e-02, 4.75462514e-03, -1.07961076e+01]])

		# Output values
		self.North = 0
		self.East  = 0
		self.Down  = 0
		self.Yaw   = 0
		self.isReady = False

	def processFrameStart(self):
		# Create a thread
		if self.thread == None:
			self.thread = Thread(target=self.processFrame)
			self.thread.start()
			print('Frame processing thread start')

			# Block till we start receiving values
			while (self.isReceiving != True):
				time.sleep(0.1)

	def processFrame(self):
		# Process data until closed
		while(self.isRun):
			self.getPose()
			self.isReceiving = True

	def getPose(self):
		# Get frame and convert to gray
		gray = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)

		# lists of ids and corners belonging to each id
		corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=self.parm)

		# Only continue if a marker was found
		if np.all(ids != None):
			if (len(ids) != 1):
				pass
			else:
				# Estimate the pose
				rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)

				# Save the distances
				self.North = tvec[0][0][2]*100
				self.East  = tvec[0][0][0]*100
				self.Down  = tvec[0][0][1]*100

				# Convert to rotation matrix and extract yaw
				R, _ = cv2.Rodrigues(rvec)
				eulerAngles = self.rotationMatrixToEulerAngles(R)
				self.Yaw = eulerAngles[1]

		# Change state for controller class
		self.isReady = True

	def isRotationMatrix(self, R):
		# Checks if matrix is valid
		Rt = np.transpose(R)
		shouldBeIdentity = np.dot(Rt, R)
		I = np.identity(3, dtype = R.dtype)
		n = np.linalg.norm(I - shouldBeIdentity)
		return n < 1e-6

	def rotationMatrixToEulerAngles(self, R):
		# Check if rotation matrix is valid
		assert(self.isRotationMatrix(R))

		# Check if singular
		sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])

		if (sy < 1e-6):
			# Singular
			x = math.atan2(-R[1,2], R[1,1])
			y = math.atan2(-R[2,0], sy)
			z = 0
		else:
			# Not singular
			x = math.atan2(R[2,1] , R[2,2])
			y = math.atan2(-R[2,0], sy)
			z = math.atan2(R[1,0], R[0,0])

		# Return roll, pitch, and yaw
		return np.array([x, y, z])

	def close(self):
		# Close the processing thread
		self.isRun = False
		self.thread.join()
		print('Frame processing thread closed')
