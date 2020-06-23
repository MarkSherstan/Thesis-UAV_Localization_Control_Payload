from filter import MovingAverage, KalmanFilter
from multiprocessing import Process, Queue
from dronekit import connect, VehicleMode
from controller import Controller
from setpoints import SetPoints
from pymavlink import mavutil
from vision import Vision
from IMU import MyVehicle
import pandas as pd
import numpy as np
import statistics
import datetime
import math
import time 

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
    connection_string = "/dev/ttyS1"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=1500000, vehicle_class=MyVehicle)

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
    V = Vision()
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()
    
    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    SP = SetPoints(300, 0, 50)

    # Create low pass filters
    nAvg = MovingAverage(5)
    eAvg = MovingAverage(3)
    dAvg = MovingAverage(3)
 
    # Create a Kalman filter 
    yKF  = KalmanFilter()
    
    # Logging variables
    freqList = []
    data = []
    
    # Wait till we switch modes to prevent integral windup and keep vision queue empty
    while(vehicle.mode.name != 'GUIDED_NOGPS'):
        print(vehicle.mode.name)
        northV, eastV, downV, yawV = getVision(Q)

    # Select set point method
    SP.selectMethod(Q, trajectory=False)
        
    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()
    kalmanTimer = time.time()
    C.startController()
    
    try:
        while(True):
            # Get vision data
            northV, eastV, downV, yawV = getVision(Q)

            # Get IMU data and convert to deg/s
            zGyro = vehicle.raw_imu.zgyro * (180 / (1000 * np.pi))
                        
            # Smooth vision data with moving average low pass filter and a kalman filter
            northV = nAvg.update(northV)
            eastV  = eAvg.update(eastV)
            downV  = dAvg.update(downV)
            yawV   = yKF.update(time.time() - kalmanTimer, np.array([yawV, zGyro]).T)
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
            # print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))
            # print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
            loopTimer = time.time()

            # Log data
            data.append([vehicle.mode.name, time.time()-startTime, freqLocal, 
                        northV, eastV, downV, yawV, 
                        desired[0], desired[1], desired[2], 
                        roll, pitch, yaw, 
                        rollControl, pitchControl, yawControl, thrustControl,
                        zGyro])
            
            # Reset integral whenever there is a mode change 
            if (vehicle.mode.name == "STABILIZE"):
                C.resetIntegral()
            
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
                            'zGyro'])

        # Save data to CSV
        now = datetime.datetime.now()
        fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
