from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from filter import MovingAverage, KalmanFilterRot, KalmanFilterPos
from multiprocessing import Process, Queue
from controller import Controller
from setpoints import SetPoints
from pymavlink import mavutil
from IMU import MyVehicle
import pandas as pd
import numpy as np
import statistics
import datetime
import math
import time

printFlag = True

def gains(C):
    # PID Gains: NORTH (pitch)
    C.kp_NORTH = 0.001
    C.ki_NORTH = 0.0001
    C.kd_NORTH = 0.0001

    # PID Gains: EAST (roll)
    C.kp_EAST = C.kp_NORTH
    C.ki_EAST = C.ki_NORTH
    C.kd_EAST = C.kd_NORTH

    # PID Gains: DOWN (thrust)
    C.kp_DOWN = 0.001
    C.ki_DOWN = 0.0
    C.kd_DOWN = 0.0

    # PID Gains: YAW (yaw rate)
    C.kp_YAW = 0.7
    C.ki_YAW = 0.1
    C.kd_YAW = 0.5

def startSim(vehicle, targetAltitude=1.5):
    # Wait till vehicle is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialize...")
        time.sleep(1)
    print("Arming motors")

    # Set vehicle mode
    vehicle.mode = VehicleMode("GUIDED")

    # Arm the vehicle
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    # Take off
    print("Taking off!")
    vehicle.simple_takeoff(targetAltitude)

    # Wait until actual altitude is achieved
    while abs(vehicle.location.local_frame.down) <= (0.8):
        print(round(vehicle.location.local_frame.down,3))
        time.sleep(0.2)
    
def falseVisionData(Q, UAV):
    A = UAV.location.local_frame.north * 100.0
    B = UAV.location.local_frame.east * 100.0
    C = UAV.location.local_frame.down * -100.0
    D = math.degrees(UAV.attitude.yaw)

    Q.put([A, B, C, D])
    time.sleep(1/30)
    
def getVision(Q):
    # Vision Data
    temp = Q.get()
    return temp[0], temp[1], temp[2], temp[3]

def getVehicleAttitude(UAV):
    # Actual vehicle attitude
    roll  = math.degrees(UAV.attitude.roll)
    pitch = math.degrees(UAV.attitude.pitch)
    yaw   = math.degrees(UAV.attitude.yaw)
    return roll, pitch, yaw

def main():
    # Connect to the Vehicle
    connection_string = "127.0.0.1:14551"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], vehicle_class=MyVehicle)

    # Set attitude request message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)
    time.sleep(0.5)

    # Set raw IMU data message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)
    time.sleep(0.5)

    # Connect to vision, create the queue, and start the core
    Q = Queue()
    falseVisionData(Q, vehicle)

    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    gains(C)
    SP = SetPoints(100, 200, 125)

    # Create low pass filters
    nAvg = MovingAverage(5)
    eAvg = MovingAverage(5)
    dAvg = MovingAverage(5)
    yAvg = MovingAverage(5)

    # Create a Kalman filters
    nKF = KalmanFilterPos()
    eKF = KalmanFilterPos()
    dKF = KalmanFilterPos()
    yKF = KalmanFilterRot()

    # Logging variables
    freqList = []
    data = []

    # Start sim
    startSim(vehicle)

    # Select set point method
    for _ in range(15):
        falseVisionData(Q, vehicle)
        
    SP.selectMethod(Q, trajectory=True)
    modeState = 0

    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()
    kalmanTimer = time.time()
    C.startController()

    try:
        while(True):
            # Get vision data
            falseVisionData(Q, vehicle)
            northVraw, eastVraw, downVraw, yawVraw = getVision(Q)

            # Get IMU data and convert to deg/s
            zGyro = vehicle.raw_imu.zgyro * (180 / (1000 * np.pi))

            # Smooth vision data with moving average low pass filter and/or kalman filter
            # northV = nAvg.update(northVraw)
            # eastV  = eAvg.update(eastVraw)
            # downV  = dAvg.update(downVraw)
            # yawV   = yAvg.update(yawVraw)
            yawV   = yKF.update(time.time() - kalmanTimer, np.array([yawVraw, zGyro]).T)
            northV = nKF.update(time.time() - kalmanTimer, np.array([northVraw]))
            eastV = eKF.update(time.time() - kalmanTimer, np.array([eastVraw]))
            downV = dKF.update(time.time() - kalmanTimer, np.array([downVraw]))
            kalmanTimer = time.time()

            # Calculate control and execute
            actual = [northV, eastV, downV, yawV]
            desired = SP.getDesired()
            rollControl, pitchControl, yawControl, thrustControl = C.positionControl(actual, desired)
            rollControl = -rollControl
            C.sendAttitudeTarget(rollControl, pitchControl, yawControl, thrustControl)
            
            # Get actual vehicle attitude
            roll, pitch, yaw = getVehicleAttitude(vehicle)

            # Print data
            freqLocal = (1 / (time.time() - loopTimer))
            freqList.append(freqLocal)

            if printFlag is True:
                print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))
                # print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
            
            loopTimer = time.time()

            # Log data
            data.append([vehicle.mode.name, time.time()-startTime, freqLocal, 
                        northV, eastV, downV, yawV, 
                        desired[0], desired[1], desired[2], 
                        roll, pitch, yaw, 
                        rollControl, pitchControl, yawControl, thrustControl,
                        northVraw, eastVraw, downVraw, yawVraw, zGyro])

            # Reset integral and generate new trajectory whenever there is a mode switch 
            if (vehicle.mode.name == "STABILIZE"):
                modeState = 1
            
            if (vehicle.mode.name == "GUIDED") and (modeState == 1):
                modeState = 0
            
                for _ in range(15):
                    falseVisionData(Q, vehicle)
                SP.selectMethod(Q, trajectory=True)
                
                C.resetIntegral()
                
    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')
        
        # Land the UAV and close connection
        print("Closing vehicle connection and land\n")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()

    finally:        
        # Post main loop rate
        print("Average loop rate: ", round(statistics.mean(freqList),2), "+/-", round(statistics.stdev(freqList), 2))

        # Write data to a data frame
        df = pd.DataFrame(data, columns=['Mode', 'Time', 'Freq',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'northVraw', 'eastVraw', 'downVraw', 'yawVraw', 'zGyro'])

        # Save data to CSV
        now = datetime.datetime.now()
        fileName = "flightData/SIM_" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
