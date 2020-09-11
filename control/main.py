from payloads import SerialComs, QuickConnect
from filter import MovingAverage, TimeSync
from multiprocessing import Process, Queue
from dronekit import connect, VehicleMode
from vision import Vision, VisionData
from controller import Controller
from setpoints import SetPoints
from pymavlink import mavutil
import pandas as pd
import numpy as np
import statistics
import datetime
import math
import time

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

    # Connect to vision, create the queue, start the core, and start the retrieval thread
    V = Vision()
    Q = Queue()
    P = Process(target=V.run, args=(Q, ))
    P.start()
    vData = VisionData(Q)

    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    SP = SetPoints(-10, 40, 0)

    # Connect to quick connect
    # s = SerialComs()
    # s.serialThreadStart()
    # qc = QuickConnect(s)
    # qc.release()

    # Moving average for velocity and acceleration (trajectory generation)
    winSizeVel = 5;                      winSizeAcc = 10
    nVelAvg = MovingAverage(winSizeVel); nAccAvg = MovingAverage(winSizeAcc)
    eVelAvg = MovingAverage(winSizeVel); eAccAvg = MovingAverage(winSizeAcc)
    dVelAvg = MovingAverage(winSizeVel); dAccAvg = MovingAverage(winSizeAcc)

    # Loop rate stabilization
    sync = TimeSync(1/30)
    sync.startTimer()

    # Data logging
    data = []

    # Wait till we switch modes to prevent integral windup and keep everything happy
    while(vehicle.mode.name != 'GUIDED_NOGPS'):
        # Current mode
        print(vehicle.mode.name)

        # Stabilize rate
        _ = sync.stabilize()

        # Get vision and IMU data
        vData.update()
        
        # Create moving average for velocity and acceleration
        velAvg = [nVelAvg.update(vData.N.Vel), eVelAvg.update(vData.E.Vel), dVelAvg.update(vData.D.Vel)]
        accAvg = [nAccAvg.update(vData.N.Acc), eAccAvg.update(vData.E.Acc), dAccAvg.update(vData.D.Acc)]

    # Create a trajectory to follow
    # SP.createTrajectory([vData.N.Pos, vData.E.Pos, vData.D.Pos], velAvg, accAvg)
    # SP.createStep([vData.N.Pos, vData.E.Pos, vData.D.Pos])
    SP.createWave(testState='Y')
    modeState = 0

    # Timers
    startTime = time.time()
    loopTimer = time.time()
    sync.startTimer()
    C.startController()

    try:
        while(True):
            # Stabilize rate
            sleepTimer = time.time()
            time2delay = sync.stabilize()
            actualDelay = time.time() - sleepTimer

            # Get vision and IMU data
            vData.update()
        
            # Create moving average for velocity and acceleration
            velAvg = [nVelAvg.update(vData.N.Vel), eVelAvg.update(vData.E.Vel), dVelAvg.update(vData.D.Vel)]
            accAvg = [nAccAvg.update(vData.N.Acc), eAccAvg.update(vData.E.Acc), dAccAvg.update(vData.D.Acc)]

            # Calculate control and execute
            actual = [vData.N.Pos, vData.E.Pos, vData.D.Pos, vData.Y.Ang]
            desired = SP.getDesired()
            rollControl, pitchControl, yawControl, thrustControl, landState = C.positionControl(actual, desired)

            rollControl = desired[1]; pitchControl = desired[0]; yawControl = desired[3]; thrustControl = desired[2]; # Only for testing
            C.sendAttitudeTarget(rollControl, pitchControl, yawControl, thrustControl)

            # Get actual vehicle attitude
            roll, pitch, yaw = getVehicleAttitude(vehicle)

            # If landed, engange the quick connect
            # if (landState == True):
            #     qc.engage()
            #     # SP.updateSetPoints(-10, 40, 100)
            #     # SP.createTrajectory([vData.N.Pos, vData.E.Pos, vData.D.Pos], velAvg, accAvg)
            #     # C.resetController()

            # Calculate the sample rate
            tempTime = time.time()
            freqLocal = (1.0 / (tempTime - loopTimer))
            loopTimer = tempTime

            # Print data
            # # if printFlag is True:
            #     print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, vData.N.Pos, vData.E.Pos, vData.D.Pos, vData.Y.Ang))
            #     print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
            #     print('N: {:<8.1f} {:<8.1f} {:<8.1f} E: {:<8.1f} {:<8.1f} {:<8.1f} D: {:<8.1f} {:<8.1f} {:<8.1f} Y: {:<8.1f} {:<8.1f}'.format(vData.N.Pos, vData.N.Vel, vData.N.Acc, vData.E.Pos, vData.E.Vel, vData.E.Acc, vData.D.Pos, vData.D.Vel, vData.D.Acc, vData.Y.Ang, vData.Y.Vel)) 

            # Log data
            data.append([vehicle.mode.name, time.time()-startTime, 
                        freqLocal, time2delay, actualDelay,
                        desired[0], desired[1], desired[2],
                        roll, pitch, yaw,
                        rollControl, pitchControl, yawControl, thrustControl,
                        vData.N.Pos, vData.E.Pos, vData.D.Pos, vData.Y.Ang,
                        vData.N.Vel, vData.E.Vel, vData.D.Vel, vData.Y.Vel,
                        vData.N.Acc, vData.E.Acc, vData.D.Acc,
                        vData.N.Raw, vData.E.Raw, vData.D.Raw, vData.Y.Raw,
                        vData.N.Dif, vData.E.Dif, vData.D.Dif, vData.Y.Dif,
                        landState, Q.qsize()])

            # Reset controller and generate new trajectory whenever there is a mode switch
            if (vehicle.mode.name == 'STABILIZE'):
                modeState = 1

            if (vehicle.mode.name == 'GUIDED_NOGPS') and (modeState == 1):
                modeState = 0
                C.resetController()
                # SP.createTrajectory([vData.N.Pos, vData.E.Pos, vData.D.Pos], velAvg, accAvg)
                # SP.createStep([vData.N.Pos, vData.E.Pos, vData.D.Pos])
                SP.createWave(testState='Y')

    except KeyboardInterrupt:
        # Print final remarks and close connections and threads
        print('Closing')
        C.logData()
        # s.close()
        GV.close()

    finally:
        # Write data to a data frame
        df = pd.DataFrame(data, columns=['Mode', 'Time', 
                            'Freq', 'time2delay', 'actualDelay',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'North-Pos', 'East-Pos', 'Down-Pos', 'Yaw-Ang',
                            'North-Vel', 'East-Vel', 'Down-Vel', 'Yaw-Vel',
                            'North-Acc', 'East-Acc', 'Down-Acc', 
                            'North-Raw', 'East-Raw', 'Down-Raw', 'Yaw-Raw',
                            'North-Dif', 'East-Dif', 'Down-Dif', 'Yaw-Dif',
                            'Landing-State', 'Q-Size'])
        
        # Print sampling rate
        Print('Sampling Frequency\n' + '{:<4.3f} +/- {:<0.3f} '.format(df['Freq'].mean(), df['Freq'].std()))
        
        # Save data to CSV
        now = datetime.datetime.now()
        fileName = 'flightData/' + now.strftime('%Y-%m-%d__%H-%M-%S') + '.csv'
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
