from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from multiprocessing import Process, Queue
from pymavlink import mavutil
from VisionMultiCore import *
from controller import *
import multiprocessing as mp
import pandas as pd
import datetime
import queue
import math

def dispData(C, vehicle):
    # Display data to user:
    print("Armed: ", vehicle.armed, " Mode: ", vehicle.mode.name)

    print('Vision ->\t', \
        '\tN: ', round(C.North,1), \
        '\tE: ', round(C.East,1), \
        '\tD: ', round(C.Down,1), \
        '\tY: ', round(math.degrees(C.Yaw),1))

    print('Controller ->\t', \
        '\tR: ', round(math.degrees(C.rollAngle),1), \
        '\tP: ', round(math.degrees(C.pitchAngle),1), \
        '\tY: ', round(math.degrees(C.yawRate),1), \
        '\tT: ', round(C.thrust,2))

    print('Attitude ->\t', \
        '\tR: ', round(math.degrees(vehicle.attitude.roll),1), \
        '\tP: ', round(math.degrees(vehicle.attitude.pitch),1), \
        '\tY: ', round(math.degrees(vehicle.attitude.yaw),1), '\n')

def main():
    # Flags and data rates
    printFlag = True
    printRate = 0.75
    logFlag   = True
    logRate   = 1/60

    # Connect to the Vehicle
    connection_string = "/dev/ttyS1"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=921600, rate=30)

    # Initialize the vision class
    v = VisionMultiCore()

    # Create an exit event for vision
    quitVision = mp.Event()

    # Initialize queue and start the computer vision core
    q = Queue()
    p = Process(target=v.processFrame, args=(q, quitVision))
    p.start()

    # Start controller and thread
    northDesired = 225
    eastDesired  = 0
    downDesired  = 0
    C = Controller(vehicle, northDesired, eastDesired, downDesired)
    C.controllerStart()
    
    time.sleep(0.1)
    C.setDataRate()
    time.sleep(0.1)

    # Run until broken by user
    print('\nStarting...\n')
    tempData = []
    printTimer = time.time()
    logTimer = time.time()

    try:
        while(True):
            # Unpack the data from vision
            visionData = q.get()

            C.North = visionData[0]
            C.East  = visionData[1]
            C.Down  = visionData[2]
            C.Yaw   = visionData[3]

            # Add small sleep 
            time.sleep(0.05)

            # Print data
            if (time.time() > printTimer + printRate) and (printFlag is True):
                dispData(C, vehicle)
                printTimer = time.time()

            # Log data
            if (time.time() > logTimer + logRate) and (logFlag is True):
                tempData.append([vehicle.mode.name, time.time(), \
                    math.degrees(vehicle.attitude.roll), math.degrees(vehicle.attitude.pitch), math.degrees(vehicle.attitude.yaw), \
                    C.North, C.East, C.Down, math.degrees(C.Yaw), \
                    northDesired, eastDesired, downDesired, \
                    math.degrees(C.rollAngle), math.degrees(C.pitchAngle), math.degrees(C.yawRate), C.thrust])
                logTimer = time.time()
    
    except KeyboardInterrupt:
        # Close the threads
        C.close()
        vehicle.close()

        # Join the cores
        quitVision.set()
        p.join()

        # Write data to a file based on flag
        if (logFlag is True):
            # Create file name
            now = datetime.datetime.now()
            fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"

            # Write data to a data frame
            df = pd.DataFrame(tempData, columns=['Mode', 'Time',
                                'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                                'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                                'North-Desired', 'East-Desired', 'Down-Desired',
                                'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control'])

            # Save as CSV and display to user
            df.to_csv(fileName, index=None, header=True)
            print('File saved to:' + fileName)

# Main loop
if __name__ == '__main__':
    main()
