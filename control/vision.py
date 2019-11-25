from threading import Thread
import numpy as np
import time
import pickle
import glob
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
		# Close the serial port connection
		self.isRun = False
		self.thread.join()
		print('Camera thread closed')
		self.cam.release()
		print('Camera closed')

class ProcessFrame:
	def __init__(self, arucoDict=aruco.DICT_5X5_1000):
		# Aruco dictionary to be used
		self.arucoDict = aruco.Dictionary_get(arucoDict)

		# Processing parameters
		self.parm = aruco.DetectorParameters_create()
		self.parm.adaptiveThreshConstant = 10

		# Calibration values
		self.mtx = np.array([[1.01477877e+03, 0.00000000e+00, 6.59914048e+02], [0.00000000e+00, 1.01195343e+03, 3.81079689e+02], [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]])
		self.dist = np.array([[ 1.51665143e-01, 9.43194746e-01, 1.00876376e-02, 4.75462514e-03, -1.07961076e+01]])

	def getPose(self, frame):
		# Get frame and convert to gray
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

		# lists of ids and corners belonging to each id
		corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=self.parm)

		# Only continue if a marker was found
		if np.all(ids != None):
			# Estimate the pose
			rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, self.mtx, self.dist)

			# Print ids found in top left
			idz = ''
			for ii in range(0, ids.size):
				idz += str(ids[ii][0])+' '
				x = round(tvec[ii][0][0]*100,2)
				y = round(tvec[ii][0][1]*100,2)
				z = round(tvec[ii][0][2]*100,2)
				print(x,y,z)

def main():
	# Start capture frame class
	CF = CaptureFrame()
	CF.acquireFrameStart()

	# Start processing frame class
	PF = ProcessFrame()

	# Run for T secounds
	T = int(input('Input time to run: '))

	tt = time.time()
	while time.time() < tt + T:
		# frame =

		PF.getPose(CF.frame)
		# cv2.imshow('frame', frame)
		# if cv2.waitKey(1) & 0xFF == ord('q'):
		# 	return

	# Clean up
	# cv2.destroyAllWindows()
	CF.close()

# Main loop
if __name__ == '__main__':
	main()
