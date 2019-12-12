import numpy as np
import pickle
import glob
import cv2
import cv2.aruco as aruco

class calibrateCamera:
	def __init__(self):
		self.arucoDict = aruco.Dictionary_get(aruco.DICT_5X5_1000)

		try:
			self.cam = cv2.VideoCapture(0)
			self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
			self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 960)
			print('Camera start')
		except:
			print('Camera setup failed')

		self.mtx = None
		self.dist = None

		self.calibrationDir = 'calibrationImgs/'
		self.imgExtension = '.jpg'

	def generateCharucoBoard(self, rows=7, columns=5):
		# Create the board
		board = aruco.CharucoBoard_create(
					squaresX=columns,
					squaresY=rows,
					squareLength=0.025,
					markerLength=0.0125,
					dictionary=self.arucoDict)
		img = board.draw((100*columns, 100*rows))

		# Save it to a file
		cv2.imwrite('CharucoBoard.png', img)

	def generateArucoMarker(self, ID=7, size=700):
		# Create an image from the marker
		img = aruco.drawMarker(self.arucoDict, ID, size)

		# Save and display (press any key to exit)
		cv2.imwrite('ARUCO_'+str(ID)+'.png', img)
		cv2.imshow('Marker ID: ' + str(ID), img)
		cv2.waitKey(0)
		cv2.destroyAllWindows()

	def captureCalibrationImages(self):
		# Initialize snapshot counter
		index = 0

		while(True):
			# Get frame and show
			_, frame = self.cam.read()
			cv2.imshow('Frame', frame)

			# Check keyboard commands
			#   'space' -> Snapshot
			#   'q' -> Quit
			key = cv2.waitKey(1)

			if key != -1:
				if key & 0xFF == ord(' '):
					indexedFile = self.calibrationDir + 'IMG_' + str(index) + self.imgExtension
					print(indexedFile)
					cv2.imwrite(indexedFile, frame)
					index += 1
				elif key & 0xFF == ord('q'):
					break

		# Clear connections and window
		self.cam.release()
		cv2.destroyAllWindows()

	def calibrateCamera(self, rows=7, columns=5, lengthSquare=0.035, lengthMarker=0.0175):
		# Create charuco board with actual measured dimensions from print out
		board = aruco.CharucoBoard_create(
					squaresX=columns,
					squaresY=rows,
					squareLength=lengthSquare,
					markerLength=lengthMarker,
					dictionary=self.arucoDict)

		# Storage variables
		cornerList = []
		idList = []

		# Get image paths from calibration folder
		paths = glob.glob(self.calibrationDir + '*' + self.imgExtension)

		# Empty imageSize variable to be determined at runtime
		imageSize = None

		# Loop through all images
		for filePath in paths:
			# Read image and convert to gray
			img = cv2.imread(filePath)
			gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

			# Find aruco markers in the query image
			corners, ids, _ = aruco.detectMarkers(image=gray, dictionary=self.arucoDict)

			# Outline the aruco markers found in the query image
			img = aruco.drawDetectedMarkers(image=img, corners=corners)

			# Get charuco corners and ids from detected aruco markers
			response, charucoCorners, charucoIDs = aruco.interpolateCornersCharuco(
					markerCorners=corners,
					markerIds=ids,
					image=gray,
					board=board)

			# If at least 20 corners were found
			if response > 20:
				# Add the corners and ids to calibration list
				cornerList.append(charucoCorners)
				idList.append(charucoIDs)

				# Draw the Charuco board detected to show calibration results
				img = aruco.drawDetectedCornersCharuco(
						image=img,
						charucoCorners=charucoCorners,
						charucoIds=charucoIDs)

				# If image size is still None, set it to the image size
				if not imageSize:
					imageSize = gray.shape[::-1]

				# Display each image until a key is pressed
				cv2.imshow(str(filePath), img)
				cv2.waitKey(0)
			else:
				# Error message
				print('Error in: ' + str(filePath))
				cv2.imshow('ERROR: ' + str(filePath), img)
				cv2.waitKey(0)

		# Destroy any open windows
		cv2.destroyAllWindows()

		# Make sure at least one image was found
		if len(paths) < 1:
			print('No images of charucoboards were found.')
			return

		# Make sure at least one charucoboard was found
		if not imageSize:
			print('Images supplied were not regonized by calibration')
			return

		# Run calibration
		_, self.mtx, self.dist, _, _ = aruco.calibrateCameraCharuco(
				charucoCorners=cornerList,
				charucoIds=idList,
				board=board,
				imageSize=imageSize,
				cameraMatrix=None,
				distCoeffs=None)

		# Display matrix and distortion coefficients
		print(self.mtx)
		print(self.dist)

		# Pickle the results
		f = open('resources/calibration.pckl', 'wb')
		pickle.dump((self.mtx, self.dist), f)
		f.close()

	def getCalibration(self):
		# Open file, retrieve variables, and close
		file = open('resources/calibration.pckl', 'rb')
		self.mtx, self.dist = pickle.load(file)
		file.close()

	def trackAruco(self, lengthMarker=0.106):
		# Get calibration data
		try:
			self.getCalibration()
		except:
			print('Calibration not found!')

		# Font and color for screen writing
		font = cv2.FONT_HERSHEY_SIMPLEX
		fontColor = (0, 255, 0)

		# Set parameters
		parameters = aruco.DetectorParameters_create()
		parameters.adaptiveThreshConstant = 10

		while(True):
			# Get frame and convert to gray
			_, frame = self.cam.read()
			gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

			# lists of ids and corners belonging to each id
			corners, ids, _ = aruco.detectMarkers(gray, self.arucoDict, parameters=parameters)

			# Only continue if a marker was found
			if np.all(ids != None):
				# Estimate the pose
				rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, lengthMarker, self.mtx, self.dist)

				# Draw axis for each aruco marker found
				for ii in range(0, ids.size):
					aruco.drawAxis(frame, self.mtx, self.dist, rvec[ii], tvec[ii], 0.1)

				# Draw square around markers
				aruco.drawDetectedMarkers(frame, corners)

				# Print ids found in top left and to the screen
				idz = ''
				for ii in range(0, ids.size):
					idz += str(ids[ii][0])+' '
					x = round(tvec[ii][0][0]*100,1)
					y = round(tvec[ii][0][1]*100,1)
					z = round(tvec[ii][0][2]*100,1)

					cv2.putText(frame, "X: " + str(x), (0, 50), font, 1, fontColor, 2)
					cv2.putText(frame, "Y: " + str(y), (0, 75), font, 1, fontColor, 2)
					cv2.putText(frame, "Z: " + str(z), (0, 100), font, 1, fontColor, 2)

					print(x,y,z)

				cv2.putText(frame, "ID: " + idz, (0, 25), font, 1, fontColor, 2)

			# display the resulting frame
			cv2.imshow('frame', frame)
			if cv2.waitKey(1) & 0xFF == ord('q'):
				break

		# When complete close everything down
		self.cam.release()
		cv2.destroyAllWindows()


def main():
	CC = calibrateCamera()

	# CC.generateCharucoBoard()

	# CC.generateArucoMarker(ID=97, size=90)
	# CC.generateArucoMarker(ID=35, size=300)
	# CC.generateArucoMarker(ID=17, size=500)

	# CC.captureCalibrationImages()

	# CC.calibrateCamera()

	# CC.getCalibration()
	# print(CC.mtx)
	# print(CC.dist)

	# CC.trackAruco()

# Main loop
if __name__ == '__main__':
	main()
