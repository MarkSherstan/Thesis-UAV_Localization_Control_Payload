from threading import Thread
import numpy as np
import math
import time
import cv2
import cv2.aruco as aruco

class Vision:
	def __init__(self):
		# Threading parameters
		self.isReceivingFrame = False
		self.isReceivingPose = False

		self.isRunFrame = True
		self.isRunPose = True

		self.frameThread = None
		self.poseThread = None

		# Frame
		self.frame = None

		# Aruco dictionary to be used and pose processing parameters
		self.arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)
		self.parm = aruco.DetectorParameters_create()
		self.parm.adaptiveThreshConstant = 10

		# Calibration values
		self.mtx = np.array([[1.40106056e+03, 0.00000000e+00, 6.36194296e+02],
							 [0.00000000e+00, 1.39929991e+03, 4.73179102e+02],
							 [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
		self.dist = np.array([[0.00770591, 0.24889461, 0.00463238, 0.00254009, 0.28752805]])
		self.lengthMarker = 0.106

		# Output values
		self.North = 0
		self.East  = 0
		self.Down  = 0
		self.Yaw   = 0
		self.isReady = False

		# Start the connection to the camera
		try:
			self.cam = cv2.VideoCapture(0)
			print('Camera start')
		except:
			print('Camera setup failed')

	def startFrameThread(self):
		# Create a thread
		if self.frameThread == None:
			self.frameThread = Thread(target=self.acquireFrame)
			self.frameThread.start()
			print('Camera thread start')

			# Block till we start receiving values
			while self.isReceivingFrame != True:
				time.sleep(0.1)

	def acquireFrame(self):
		# Acquire until closed
		while(self.isRunFrame):
			_, self.frame = self.cam.read()
			self.isReceivingFrame = True

	def startPoseThread(self):
		# Block until a frame has been acquired
		while self.frame is None:
			time.sleep(0.1)

		# Create a thread
		if self.poseThread == None:
			self.poseThread = Thread(target=self.processFrame)
			self.poseThread.start()
			print('Frame processing thread start')

			# Block till we start receiving values
			while (self.isReceivingPose != True):
				time.sleep(0.1)

	def processFrame(self):
		# Process data until closed
		while(self.isRunPose):
			self.getPose()
			self.isReceivingPose = True

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
				rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, self.lengthMarker, self.mtx, self.dist)

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
		# Close the pose processing thread
		self.isRunPose = False
		self.poseThread.join()
		print('\nFrame processing thread closed')

		# Close the capture thread
		self.isRunFrame = False
		self.frameThread.join()
		print('Camera thread closed')

		# Rlease the camera connection
		self.cam.release()
		print('Camera closed')
