from vision import *

# import threading
# import queue
# import time
# import numpy as np

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
		PF.getPose(CF.frame)

	CF.close()


# Main loop
if __name__ == '__main__':
	main()
