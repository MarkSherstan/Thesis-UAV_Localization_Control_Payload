from filter import MovingAverage, KalmanFilter2x
from payloads import SerialComs, QuickConnect
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
    accTemp = [temp[6], temp[7], temp[8]]
    psiTemp = [temp[9], temp[10]]
    return posTemp, velTemp, accTemp, psiTemp

def getVehicleAttitude(UAV):
    # Actual vehicle attitude
    roll  = math.degrees(UAV.attitude.roll)
    pitch = math.degrees(UAV.attitude.pitch)
    yaw   = math.degrees(UAV.attitude.yaw)
    return roll, pitch, yaw

def main():
    # Connect to the Vehicle
    connection_string = '/dev/ttyTHS1'
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=['attitude'], baud=1500000)

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
    P = Process(target=V.run, args=(Q, ))
    P.start()

    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    SP = SetPoints(-10, 40, 0)

    # Connect to quick connect 
    s = SerialComs()
    s.serialThreadStart()
    qc = QuickConnect(s)
    qc.release()
    
    # Moving average for velocity and acceleration (trajectory generation)
    winSizeVel = 5
    winSizeAcc = 10
    nVelAvg = MovingAverage(winSizeVel); nAccAvg = MovingAverage(winSizeAcc)
    eVelAvg = MovingAverage(winSizeVel); eAccAvg = MovingAverage(winSizeAcc)
    dVelAvg = MovingAverage(winSizeVel); dAccAvg = MovingAverage(winSizeAcc)
    
    # Kalman filter
    nKF = KalmanFilter2x(3.0, 5.0, 10.0)
    eKF = KalmanFilter2x(3.0, 5.0, 10.0)
    dKF = KalmanFilter2x(3.0, 5.0, 10.0)
    yKF = KalmanFilter2x(3.0, 5.0, 10.0)
    tempKalmanTime = None
    kalmanTimer = time.time()

    # Logging variables
    freqList = []
    data = []

    # Wait till we switch modes to prevent integral windup and keep everything happy
    while(vehicle.mode.name != 'GUIDED_NOGPS'):
        # Current mode
        print(vehicle.mode.name)

        # Get vision and IMU data
        pos, vel, acc, psi = getVision(Q)
        
        # Estimate yaw
        tempKalmanTime = time.time()
        yawV = yKF.update(tempKalmanTime - kalmanTimer, np.array([psi[0], psi[1]]).T)
                   
        # Fuse vision and IMU sensor data   
        northV = nKF.update(tempKalmanTime - kalmanTimer, np.array([pos[0], vel[0]]).T)
        eastV = eKF.update(tempKalmanTime - kalmanTimer, np.array([pos[1], vel[1]]).T)
        downV = dKF.update(tempKalmanTime - kalmanTimer, np.array([pos[2], vel[2]]).T)
        kalmanTimer = time.time()
        
        # Create moving average for velocity and acceleration
        velAvg = [nVelAvg.update(vel[0]), eVelAvg.update(vel[1]), dVelAvg.update(vel[2])]
        accAvg = [nAccAvg.update(acc[0]), eAccAvg.update(acc[1]), dAccAvg.update(acc[2])]
        
    # Create a trajectory to follow
    SP.createTrajectory([northV, eastV, downV], velAvg, accAvg)
    modeState = 0

    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()
    C.startController()

    try:
        while(True):
            # Get vision and IMU data
            pos, vel, acc, psi = getVision(Q)
            
            # Estimate yaw
            tempKalmanTime = time.time()
            yawV = yKF.update(tempKalmanTime - kalmanTimer, np.array([psi[0], psi[1]]).T)

            # Fuse vision and IMU sensor data   
            northV = nKF.update(tempKalmanTime - kalmanTimer, np.array([pos[0], vel[0]]).T)
            eastV = eKF.update(tempKalmanTime - kalmanTimer, np.array([pos[1], vel[1]]).T)
            downV = dKF.update(tempKalmanTime - kalmanTimer, np.array([pos[2], vel[2]]).T)
            kalmanTimer = time.time()
            
            # Create moving average for velocity and acceleration
            velAvg = [nVelAvg.update(vel[0]), eVelAvg.update(vel[1]), dVelAvg.update(vel[2])]
            accAvg = [nAccAvg.update(acc[0]), eAccAvg.update(acc[1]), dAccAvg.update(acc[2])]
            
            # Calculate control and execute
            actual = [northV, eastV, downV, yawV]
            desired = SP.getDesired()
            rollControl, pitchControl, yawControl, thrustControl, landState = C.positionControl(actual, desired)
            C.sendAttitudeTarget(rollControl, pitchControl, yawControl, thrustControl)
            
            # Get actual vehicle attitude
            roll, pitch, yaw = getVehicleAttitude(vehicle)

            # If landed, engange the quick connect
            if (landState == True):
                qc.engage()
                # SP.updateSetPoints(-10, 40, 100)
                # SP.createTrajectory([northV, eastV, downV], velAvg, accAvg)
                # C.resetController()
            
            # Print data
            freqLocal = (1 / (time.time() - loopTimer))
            freqList.append(freqLocal)

            if printFlag is True:
                print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))
                # print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
                # print('N: {:<8.1f} {:<8.1f} {:<8.1f} E: {:<8.1f} {:<8.1f} {:<8.1f} D: {:<8.1f} {:<8.1f} {:<8.1f} Y: {:<8.1f} {:<8.1f}  '.format(pos[0], vel[0], acc[0], pos[1], vel[1], acc[1], pos[2], vel[2], acc[2], psi[0], psi[1]))
                
            loopTimer = time.time()

            # Log data
            data.append([vehicle.mode.name, time.time()-startTime, freqLocal, 
                        northV, eastV, downV, yawV, 
                        desired[0], desired[1], desired[2], 
                        roll, pitch, yaw, 
                        rollControl, pitchControl, yawControl, thrustControl,
                        pos[0], pos[1], pos[2], 
                        vel[0], vel[1], vel[2],
                        acc[0], acc[1], acc[2],
                        psi[0], psi[1], landState, Q.qsize()])
            
            # Reset controller and generate new trajectory whenever there is a mode switch 
            if (vehicle.mode.name == 'STABILIZE'):
                modeState = 1
            
            if (vehicle.mode.name == 'GUIDED_NOGPS') and (modeState == 1):
                modeState = 0
                C.resetController()
                SP.createTrajectory([northV, eastV, downV], velAvg, accAvg)
                
    except KeyboardInterrupt:
        # Print final remarks and close payload connection
        print('Closing')
        s.close()
        C.logData()
        
    finally:        
        # Post main loop rate
        print('Average loop rate: ', round(statistics.mean(freqList),2), '+/-', round(statistics.stdev(freqList), 2))

        # Write data to a data frame
        df = pd.DataFrame(data, columns=['Mode', 'Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'northVraw', 'eastVraw', 'downVraw', 
                            'N-Velocity', 'E-Velocity', 'D-Velocity',
                            'N-Acceleration', 'E-Acceleration', 'D-Acceleration',
                            'yawVraw', 'yawRate', 'Landing-State', 'Q-Size'])

        # Save data to CSV
        now = datetime.datetime.now()
        fileName = 'flightData/' + now.strftime('%Y-%m-%d__%H-%M-%S') + '.csv'
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
