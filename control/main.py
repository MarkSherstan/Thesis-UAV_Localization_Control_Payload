from payloads import SerialComs, QuickConnect
from filter import MovingAverage, TimeSync
from multiprocessing import Process, Queue
from dronekit import connect, VehicleMode
from vision import Vision, GetVision
from controller import Controller
from setpoints import SetPoints
from pymavlink import mavutil
import pandas as pd
import numpy as np
import statistics
import datetime
import math
import time

printFlag = False

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

    # Connect to vision, create the queue, start the core, and start the thread
    V = Vision()
    Q = Queue()
    P = Process(target=V.run, args=(Q, ))
    P.start()
    GV = GetVision(Q)

    # Connect to control scheme and prepare setpoints
    C = Controller(vehicle)
    SP = SetPoints(-10, 40, 0)

    # Connect to quick connect
    # s = SerialComs()
    # s.serialThreadStart()
    # qc = QuickConnect(s)
    # qc.release()

    # Moving average for velocity and acceleration (trajectory generation)
    winSizeVel = 5
    winSizeAcc = 10
    nVelAvg = MovingAverage(winSizeVel); nAccAvg = MovingAverage(winSizeAcc)
    eVelAvg = MovingAverage(winSizeVel); eAccAvg = MovingAverage(winSizeAcc)
    dVelAvg = MovingAverage(winSizeVel); dAccAvg = MovingAverage(winSizeAcc)

    # Loop rate stabilization
    sync = TimeSync(1/30)
    sync.startTimer()

    # Logging variables
    freqList = []
    data = []

    # Wait till we switch modes to prevent integral windup and keep everything happy
    while(vehicle.mode.name != 'GUIDED_NOGPS'):
        # Current mode
        print(vehicle.mode.name)

        # Stabilize rate
        _ = sync.stabilize()

        # Get vision and IMU data
        pos, _, vel, acc, _, _, _ = GV.getVision()
        N = pos[0]
        E = pos[1]
        D = pos[2]

        # Create moving average for velocity and acceleration
        velAvg = [nVelAvg.update(vel[0]), eVelAvg.update(vel[1]), dVelAvg.update(vel[2])]
        accAvg = [nAccAvg.update(acc[0]), eAccAvg.update(acc[1]), dAccAvg.update(acc[2])]

    # Create a trajectory to follow
    # SP.createTrajectory([N, E, D], velAvg, accAvg)
    # SP.createStep([N, E, D])
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
            actualDelay = time.time() - time2delay

            # Get vision and IMU data
            pos, raw, vel, acc, psi, dif, camDt = GV.getVision()

            # Create moving average for velocity and acceleration
            velAvg = [nVelAvg.update(vel[0]), eVelAvg.update(vel[1]), dVelAvg.update(vel[2])]
            accAvg = [nAccAvg.update(acc[0]), eAccAvg.update(acc[1]), dAccAvg.update(acc[2])]

            # Calculate control and execute
            actual = [pos[0], pos[1], pos[2], psi[0]]
            desired = SP.getDesired()
            rollControl, pitchControl, yawControl, thrustControl, controlDeltaT, landState = C.positionControl(actual, desired)

            rollControl = desired[1]; pitchControl = desired[0]; yawControl = desired[3]; thrustControl = desired[2]; # Only for testing
            C.sendAttitudeTarget(rollControl, pitchControl, yawControl, thrustControl)

            # Get actual vehicle attitude
            roll, pitch, yaw = getVehicleAttitude(vehicle)

            # If landed, engange the quick connect
            # if (landState == True):
            #     qc.engage()
            #     # SP.updateSetPoints(-10, 40, 100)
            #     # SP.createTrajectory([northV, eastV, downV], velAvg, accAvg)
            #     # C.resetController()

            # Calcualte and log sample rate
            tempTime = time.time()
            freqLocal = (1.0 / (tempTime - loopTimer))
            loopTimer = tempTime
            freqList.append(freqLocal)

            # Print data
            # # if printFlag is True:
            #     print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.1f}'.format(freqLocal, northV, eastV, downV, yawV))
            #     print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
            #     print('N: {:<8.1f} {:<8.1f} {:<8.1f} E: {:<8.1f} {:<8.1f} {:<8.1f} D: {:<8.1f} {:<8.1f} {:<8.1f} Y: {:<8.1f} {:<8.1f}  '.format(pos[0], vel[0], acc[0], pos[1], vel[1], acc[1], pos[2], vel[2], acc[2], psi[0], psi[1]))

            # Log data
            data.append([vehicle.mode.name, time.time()-startTime, 
                        freqLocal, time2delay, actualDelay,
                        pos[0], pos[1], pos[2], psi[0],
                        desired[0], desired[1], desired[2],
                        roll, pitch, yaw,
                        rollControl, pitchControl, yawControl, thrustControl,
                        raw[0], raw[1], raw[2],
                        vel[0], vel[1], vel[2],
                        acc[0], acc[1], acc[2],
                        psi[1], psi[2], landState, Q.qsize(),
                        dif[0], dif[1], dif[2], dif[3]])

            # Reset controller and generate new trajectory whenever there is a mode switch
            if (vehicle.mode.name == 'STABILIZE'):
                modeState = 1

            if (vehicle.mode.name == 'GUIDED_NOGPS') and (modeState == 1):
                modeState = 0
                C.resetController()
                # SP.createTrajectory([northV, eastV, downV], velAvg, accAvg)
                # SP.createStep([northV, eastV, downV])
                SP.createWave(testState='Y')

    except KeyboardInterrupt:
        # Print final remarks and close connections and threads
        print('Closing')
        C.logData()
        # s.close()
        GV.close()

    finally:
        # Post main loop rate
        print('Average loop rate: ', round(statistics.mean(freqList),2), '+/-', round(statistics.stdev(freqList), 2))

        # Write data to a data frame
        df = pd.DataFrame(data, columns=['Mode', 'Time', 
                            'Freq', 'time2delay', 'actualDelay',
                            'North-Vision',  'East-Vision',  'Down-Vision', 'Yaw-Vision',
                            'North-Desired', 'East-Desired', 'Down-Desired',
                            'Roll-UAV', 'Pitch-UAV', 'Yaw-UAV',
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control',
                            'northVraw', 'eastVraw', 'downVraw',
                            'N-Velocity', 'E-Velocity', 'D-Velocity',
                            'N-Acceleration', 'E-Acceleration', 'D-Acceleration',
                            'yawVraw', 'yawRate', 'Landing-State', 'Q-Size',
                            'N-diff', 'E-diff', 'D-diff', 'Y-diff'])

        # Save data to CSV
        now = datetime.datetime.now()
        fileName = 'flightData/' + now.strftime('%Y-%m-%d__%H-%M-%S') + '.csv'
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
