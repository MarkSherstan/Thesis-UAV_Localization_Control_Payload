from filter import MovingAverage, KalmanFilter1x, KalmanFilter2x
from multiprocessing import Process, Queue
from dronekit import connect, VehicleMode
from controller import Controller
from setpoints import SetPoints
from pymavlink import mavutil
from vision import Vision
import pandas as pd
import numpy as np
import statistics
import datetime
import math
import time

printFlag = False

def getVision(Q):
    # Vision Data
    temp = Q.get()
    posTemp = [temp[0], temp[1], temp[2]]
    velTemp = [temp[3], temp[4], temp[5]]
    psiTemp = [temp[6], temp[7]]
    return posTemp, velTemp, psiTemp

def getVehicleAttitude(UAV):
    # Actual vehicle attitude
    roll  = math.degrees(UAV.attitude.roll)
    pitch = math.degrees(UAV.attitude.pitch)
    yaw   = math.degrees(UAV.attitude.yaw)
    return roll, pitch, yaw

def main():
    # Connect to the Vehicle
    connection_string = "/dev/ttyS1" #/dev/ttyTHS1
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=1500000)

    # Set attitude request message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)
    time.sleep(0.5)

    # Connect to vision, create the queue, and start the core
    V = Vision()
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()

    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    SP = SetPoints(10, 20, 125)

    # Create low pass filters
    nAvg = MovingAverage(3)
    eAvg = MovingAverage(3)
    dAvg = MovingAverage(3)
    yAvg = MovingAverage(3)

    # Place holder
    yKF = KalmanFilter2x(1.0, 2.0, 10.0)
    nKF = KalmanFilter1x(1.0, 10.0)
    eKF = KalmanFilter1x(1.0, 10.0)
    dKF = KalmanFilter1x(1.0, 10.0)
    kalmanTimer = time.time()

    # Logging variables
    freqList = []
    data = []

    # Wait till we switch modes to prevent integral windup and keep everything happy
    while(vehicle.mode.name != 'GUIDED_NOGPS'):
        # Current mode
        print(vehicle.mode.name)

        # Keep vision queue empty
        pos, vel, psi = getVision(Q)
        
        # Start Kalman filter to limit start up error
        _ = yKF.update(time.time() - kalmanTimer, np.array([psi[0], psi[1]]).T)
        _ = nKF.update(time.time() - kalmanTimer, np.array([pos[0]]))
        _ = eKF.update(time.time() - kalmanTimer, np.array([pos[1]]))
        _ = dKF.update(time.time() - kalmanTimer, np.array([pos[2]]))
        kalmanTimer = time.time()

    # Select set point method
    SP.selectMethod(Q, trajectory=True)
    modeState = 0

    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()
    C.startController()

    try:
        while(True):
            # Get vision data
            pos, vel, psi = getVision(Q)

            # Smooth vision data with moving average low pass filter and/or kalman filter
            # northV = nAvg.update(northVraw)
            # eastV  = eAvg.update(eastVraw)
            # downV  = dAvg.update(downVraw)
            # yawV   = yAvg.update(yawVraw)
            
            yawV = yKF.update(time.time() - kalmanTimer, np.array([psi[0], psi[1]]).T)
            northV = nKF.update(time.time() - kalmanTimer, np.array([pos[0]]))
            eastV = eKF.update(time.time() - kalmanTimer, np.array([pos[1]]))
            downV = dKF.update(time.time() - kalmanTimer, np.array([pos[2]]))
            kalmanTimer = time.time()

            # Calculate control and execute
            actual = [northV, eastV, downV, yawV]
            desired = SP.getDesired()
            rollControl, pitchControl, yawControl, thrustControl = C.positionControl(actual, desired)
            C.sendAttitudeTarget(rollControl, pitchControl, yawControl, thrustControl)
            
            # Get actual vehicle attitude
            roll, pitch, yaw = getVehicleAttitude(vehicle)

            # Print data
            freqLocal = (1 / (time.time() - loopTimer))
            freqList.append(freqLocal)

            if printFlag is True:
                print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))
                # print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
                # print('N: {:<8.1f} {:<8.1f} E: {:<8.1f} {:<8.1f} D: {:<8.1f} {:<8.1f} Y: {:<8.1f} {:<8.1f}  '.format(pos[0], vel[0], pos[1], vel[1], pos[2], vel[2], psi[0], psi[1]))

            loopTimer = time.time()

            # Log data
            data.append([vehicle.mode.name, time.time()-startTime, freqLocal, 
                        northV, eastV, downV, yawV, 
                        desired[0], desired[1], desired[2], 
                        roll, pitch, yaw, 
                        rollControl, pitchControl, yawControl, thrustControl,
                        pos[0], pos[1], pos[2], 
                        vel[0], vel[1], vel[2],
                        psi[0], psi[1], Q.qsize()])
            
            # Reset integral and generate new trajectory whenever there is a mode switch 
            if (vehicle.mode.name == "STABILIZE"):
                modeState = 1
            
            if (vehicle.mode.name == "GUIDED_NOGPS") and (modeState == 1):
                modeState = 0
                C.resetIntegral()
                SP.selectMethod(Q, trajectory=True)
                
    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')
        
    finally:        
        # Post main loop rate
        print("Average loop rate: ", round(statistics.mean(freqList),2), "+/-", round(statistics.stdev(freqList), 2))

        # Write data to a data frame
        df = pd.DataFrame(data, columns=['Mode', 'Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'northVraw', 'eastVraw', 'downVraw', 
                            'N-Velocity', 'E-Velocity', 'D-Velocity'
                            'yawVraw', 'yawRate', 'Q-Size'])

        # Save data to CSV
        now = datetime.datetime.now()
        fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
