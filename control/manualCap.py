from payloads import SerialComs, QuickConnect
from filter import MovingAverage, TimeSync
from dronekit import connect, VehicleMode
from pymavlink import mavutil
import math
import time

def main():
    # Connect to the vehicle
    connectionString = '/dev/ttyTHS1'
    print('Connecting to vehicle on: %s\n' % connectionString)
    vehicle = connect(connectionString, wait_ready=['attitude'], baud=1500000)

    # Set attitude request message rate
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)
    time.sleep(0.5)

    # Speed up reading of RC channels (test w/ and w/out)
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS,
        150, # Rate (Hz)
        1)   # Turn on
    vehicle.send_mavlink(msg)
    time.sleep(0.5)

    # Loop rate stabilization
    sync = TimeSync(1/50)
    sync.startTimer()

    # Connect to serial port and quick connect
    s = SerialComs()
    q = QuickConnect(s)
    c = CAP(s)
    
    # Engage 
    print('Engage quick conenct in 3 seconds')
    time.sleep(3)    
    q.engage()
    
    # Timers
    print('Start')
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
            
            # Monitor the switch
            state = vehicle.channels['7']
            if state > 1600:
                c.openJaws()
            elif state < 1400:
                c.closeJaws()

            # Clear the data line
            for _ in range(5):
                s.writeSerialData(q.EXIT)
                time.sleep(0.01)
   
    except KeyboardInterrupt:
        # Print final remarks and close connections/threads
        print('Closing')
        s.close()
        
    finally:
        # Final comments 
        print('I hope it worked')

if __name__ == "__main__":
    main()
