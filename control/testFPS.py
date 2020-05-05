from vision import *
import pandas as pd

def main():
	# Start vision class and the capture and pose threads
	V = Vision()
	V.startFrameThread()
	V.startPoseThread()

	# Do not proceed until vision processor is ready
	while(V.isReady != True):
		time.sleep(0.1)

	# Start a timer
	tt = time.time()

	# Log data for some time
	try:
		while(True):
			pass
			# print(V.frameCount)
			# print(V.frameTime)
	except KeyboardInterrupt:
		# Close the threads and any other connections
		V.close()


	print 'Frames: ', V.frameCount, 'Time: ', time.time()-tt
	print 'FPS: ', V.frameCount / (time.time() - tt)

# Main loop
if __name__ == '__main__':
	main()
