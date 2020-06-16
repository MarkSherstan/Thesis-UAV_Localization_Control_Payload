from multiprocessing import Process, Queue
from dronekit import connect, VehicleMode
from controller import Controller
from pymavlink import mavutil
from vision import Vision
import pandas as pd
import statistics
import datetime
import math
import time 

class movingAverage:
    def __init__(self, windowSize=3):
        self.windowSize = windowSize
        self.values = []
        self.sum = 0

    def avg(self, value):
        self.values.append(value)
        self.sum += value
        if len(self.values) > self.windowSize:
            self.sum -= self.values.pop(0)
        return float(self.sum) / len(self.values)

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
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=1500000)

    # Set attitude request message rate (everything else is default 4 Hz)
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        100, # Rate (Hz)
        1)  # Turn on
    vehicle.send_mavlink(msg)    
    
    # Connect to vision, create the queue, and start the core
    V = Vision()
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()
    
    # Connect to control scheme
    northDesired = 300
    eastDesired = 0
    downDesired = 30
    C = Controller(northDesired, eastDesired, downDesired, vehicle)

    # Create a low pass filter
    nAvg = movingAverage()
    eAvg = movingAverage()
    dAvg = movingAverage()
    yAvg = movingAverage(windowSize=6)
    
    # Logging variables
    freqList = []
    data = []
    
    # Wait till we switch modes to prevent integral windup
    while(vehicle.mode.name != 'GUIDED_NOGPS'):
        print(vehicle.mode.name)
        time.sleep(0.2)

    # Loop timer
    startTime = time.time()
    loopTimer = time.time()
    C.startController()
    
    try:
        while(True):
            # Get vision data
            northV, eastV, downV, yawV = getVision(Q)
            
            # Smooth vision data with moving average low pass filter
            northV = nAvg.avg(northV)
            eastV  = eAvg.avg(eastV)
            downV  = dAvg.avg(downV)
            yawV   = yAvg.avg(yawV)
            
            # Calculate control and execture
            rollControl, pitchControl, yawControl, thrustControl = C.positionControl(northV, eastV, downV, yawV)         

            # Get actual vehicle data
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
                        northDesired, eastDesired, downDesired, 
                        roll, pitch, yaw, 
                        rollControl, pitchControl, yawControl, thrustControl])
            
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
                            'Roll-Control', 'Pitch-Control', 'Yaw-Control', 'Thrust-Control'])

        # Save data to CSV
        now = datetime.datetime.now()
        fileName = "flightData/" + now.strftime("%Y-%m-%d__%H-%M-%S") + ".csv"
        df.to_csv(fileName, index=None, header=True)
        print('File saved to:' + fileName)

if __name__ == "__main__":
    main()
