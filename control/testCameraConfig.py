import cv2
import time

def main():
    # Set desired parameters
    desiredWidth  = 640     # 1920, 1280, 800, 640
    desiredHeight = 480     # 1080, 720, 600, 480
    desiredFPS    = 30
    autoFocus     = False
    showFrame     = False

    # Camera properties 
    cam = cv2.VideoCapture(1)
    cam.set(cv2.CAP_PROP_FRAME_WIDTH, desiredWidth)
    cam.set(cv2.CAP_PROP_FRAME_HEIGHT, desiredHeight)
    cam.set(cv2.CAP_PROP_FPS, desiredFPS)
    cam.set(cv2.CAP_PROP_AUTOFOCUS, autoFocus)

    # Counting variables
    frameCount = 0
    startTime = time.time()

    # Run test
    print('Test running for 10 secounds')
    while (time.time() < startTime+10):
        _, frame = cam.read()
        frameCount += 1

        if showFrame is True:
            cv2.imshow('Test', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    # Record final time and frame size
    endTime = time.time()
    actualHeight, actualWidth, _ = frame.shape 

    # Print results
    print('Frame Width\tD: ', desiredWidth, '\tA: ', actualWidth)
    print('Frame Height\tD: ', desiredHeight, '\tA: ', actualHeight)
    print('FPS\t\tD: ', desiredFPS, '\t\tA: ', round(frameCount / (endTime - startTime),2))

# Main loop
if __name__ == '__main__':
	main()
