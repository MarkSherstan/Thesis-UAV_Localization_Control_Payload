from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from filter import MovingAverage, KalmanFilterRot, KalmanFilterPos
from multiprocessing import Process, Queue
from controller import Controller
from setpoints import SetPoints
import matplotlib.pyplot as plt
from pymavlink import mavutil
from IMU import MyVehicle
import pandas as pd
import numpy as np
import statistics
import datetime
import math
import time

printFlag = False

def gains(C):
    # PID Gains: NORTH (pitch)
    C.kp_NORTH = 0.025
    C.ki_NORTH = 0.0055
    C.kd_NORTH = 0.065

    # PID Gains: EAST (roll)
    C.kp_EAST = 0.035
    C.ki_EAST = 0.00525
    C.kd_EAST = 0.075

    # PID Gains: DOWN (thrust)
    C.kp_DOWN = 0.0015
    C.ki_DOWN = 0.0
    C.kd_DOWN = 0.0

    # PID Gains: YAW (yaw rate)
    C.kp_YAW = 0.7
    C.ki_YAW = 0.1
    C.kd_YAW = 0.5

    # Maximum controller output constraints
    C.rollConstrain  = [-3, 3]           # Deg
    C.pitchConstrain = C.rollConstrain   # Deg
    C.thrustConstrain = [-0.5, 0.5]	     # Normalized
    C.yawRateConstrain = [-10, 10]       # Deg / s
    
def startSim(vehicle, targetAltitude=1.0):
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
    while abs(vehicle.location.local_frame.down) <= (targetAltitude*0.95):
        print(round(vehicle.location.local_frame.down,3))
        time.sleep(0.2)
    
def falseVisionData(Q, UAV, delay=1/30):
    A = UAV.location.local_frame.north * 100.0
    B = UAV.location.local_frame.east * 100.0
    C = UAV.location.local_frame.down * -100.0
    D = math.degrees(UAV.attitude.yaw)

    Q.put([A, B, C, D])
    time.sleep(delay)
    
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

def setRate(vehicle):
    # Set position data message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        50, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)

    # Set attitude request message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)

    # Set raw IMU data message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)

    vehicle.flush()

def main():
    # Connect to the Vehicle
    connection_string = "127.0.0.1:14551"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], vehicle_class=MyVehicle)

    # Set position data message rate
    setRate(vehicle)

    # Connect to vision, create the queue, and start the core
    Q = Queue()

    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    gains(C)
    SP = SetPoints(75, 50, 125)

    # Create low pass filters
    nAvg = MovingAverage(2)
    eAvg = MovingAverage(2)
    dAvg = MovingAverage(2)
    yAvg = MovingAverage(2)

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
    for _ in range(10):
        falseVisionData(Q, vehicle, delay=0)
        
    SP.selectMethod(Q, trajectory=True)
    modeState = 0

    # Loop timer(s)
    startTime = time.time()
    loopTimer = time.time()
    kalmanTimer = time.time()
    C.startController()

    try:
        while(time.time() < startTime + 15):
            # Get vision data
            falseVisionData(Q, vehicle)
            northVraw, eastVraw, downVraw, yawVraw = getVision(Q)
            setRate(vehicle)

            # Get IMU data and convert to deg/s
            zGyro = vehicle.raw_imu.zgyro * (180 / (1000 * np.pi))

            # Smooth vision data with moving average low pass filter and/or kalman filter
            northV = nAvg.update(northVraw)
            eastV  = eAvg.update(eastVraw)
            downV  = dAvg.update(downVraw)
            yawV   = yAvg.update(yawVraw)
            # yawV   = yKF.update(time.time() - kalmanTimer, np.array([yawVraw, zGyro]).T)
            # northV = nKF.update(time.time() - kalmanTimer, np.array([northVraw]))
            # eastV = eKF.update(time.time() - kalmanTimer, np.array([eastVraw]))
            # downV = dKF.update(time.time() - kalmanTimer, np.array([downVraw]))
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

    except KeyboardInterrupt:
        # Print final remarks
        print('Closing')

    finally:   
        # Land the UAV and close connection
        print("Closing vehicle connection and land\n")
        vehicle.mode = VehicleMode("LAND")
        vehicle.close()

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
        # now = datetime.datetime.now()
        # fileName = "flightData/SIM_" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        # df.to_csv(fileName, index=None, header=True)
        # print('File saved to:' + fileName)

        ##########################################################################################
        fig = plt.figure()

        # ax3 = plt.gca()

        # df.plot(kind='line', x='Time', y='North-Vision', color='#700CBC', style='-',  ax=ax3)
        # df.plot(kind='line', x='Time', y='North-Desired', color='#700CBC', style='--',  ax=ax3)

        # df.plot(kind='line', x='Time', y='East-Vision',  color='#FB8604', style='-',  ax=ax3)
        # df.plot(kind='line', x='Time', y='East-Desired',  color='#FB8604', style='--',  ax=ax3)
        
        # df.plot(kind='line', x='Time', y='Down-Vision',  color='#7FBD32', style='-',  ax=ax3)        
        # df.plot(kind='line', x='Time', y='Down-Desired',  color='#7FBD32',  style='--',  ax=ax3)

        # ax3.set_xlabel('Time [s]', fontweight='bold')
        # ax3.set_ylabel('Position [cm]', fontweight='bold')

        # ax3.legend(('North Actual','North Desired', 'East Actual', 'East Desired', 'Down Actual', 'Down Desired'), ncol=3, loc='lower center')
        # ax3.grid()
        # plt.show()

        ##########################################################################################

        # Plot 
        ########################
        # Master
        ########################
        fig = plt.figure()

        ########################
        # Roll and Pitch
        ########################
        plt.subplot(2, 2, 1)
        ax0 = plt.gca()

        df.plot(kind='line', x='Time', y='Roll-Control',  color='#FB8604', style='-', ax=ax0)
        df.plot(kind='line', x='Time', y='Pitch-Control', color='#700CBC', style='-', ax=ax0)

        df.plot(kind='line', x='Time', y='Roll-UAV',  color='#FB8604', style='--',  ax=ax0)
        df.plot(kind='line', x='Time', y='Pitch-UAV', color='#700CBC', style='--',  ax=ax0)

        ax0.set_title('Roll & Pitch Control', fontsize=14, fontweight='bold')
        ax0.set_xlabel('Time [s]', fontweight='bold')
        ax0.set_ylabel('Angle [deg]', fontweight='bold')

        ########################
        # Yaw
        ########################
        plt.subplot(2, 2, 2)
        ax1 = plt.gca()

        df.plot(kind='line', x='Time', y='Yaw-Vision', color='tab:blue', style='-', ax=ax1)
        df.plot(kind='line', x='Time', y='Yaw-Control', color='tab:blue', style='--', ax=ax1)

        ax1.set_title('Yaw Control', fontsize=14, fontweight='bold')
        ax1.set_xlabel('Time [s]', fontweight='bold')
        ax1.set_ylabel('Angle [Deg or Deg/s]', fontweight='bold')

        ########################
        # North East
        ########################
        plt.subplot(2, 2, 3)
        ax3 = plt.gca()

        df.plot(kind='line', x='Time', y='North-Vision', color='#700CBC', style='-',  ax=ax3)
        df.plot(kind='line', x='Time', y='East-Vision',  color='#FB8604', style='-',  ax=ax3)
        df.plot(kind='line', x='Time', y='Down-Vision',  color='#7FBD32', style='-',  ax=ax3)

        df.plot(kind='line', x='Time', y='North-Desired', color='#700CBC', style='--',  ax=ax3)
        df.plot(kind='line', x='Time', y='East-Desired',  color='#FB8604', style='--',  ax=ax3)
        df.plot(kind='line', x='Time', y='Down-Desired',  color='#7FBD32',  style='--',  ax=ax3)

        ax3.set_title('NED Position', fontsize=14, fontweight='bold')
        ax3.set_xlabel('Time [s]', fontweight='bold')
        ax3.set_ylabel('Position [cm]', fontweight='bold')

        ax3.legend(['N', 'E', 'D'])

        ########################
        # Thrust
        ########################
        plt.subplot(2, 2, 4)
        ax4 = plt.gca()

        df.plot(kind='line', x='Time', y='Thrust-Control', color='#7FBD32', style='-', ax=ax4)

        ax4.set_title('Thrust Control', fontsize=14, fontweight='bold')
        ax4.set_xlabel('Time [s]', fontweight='bold')
        ax4.set_ylabel('Normalized Thrust Command', fontweight='bold')
        ax4.get_legend().remove()

        ########################
        # Show the plots
        ########################
        plt.show()

if __name__ == "__main__":
    main()
