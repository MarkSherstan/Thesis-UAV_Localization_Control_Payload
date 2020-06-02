from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from multiprocessing import Process, Queue
from pymavlink import mavutil
from controller import *
import time



def main():
    # Connect to the Vehicle
    connection_string = "/dev/ttyS1"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=115200, rate=60)

    # Start controller and thread
    C = Controller(vehicle, 0, 0, 0)

    # Run until broken by user
    print('\nStarting...\n')

    # Arm 
    while not vehicle.armed:
        print("Waiting for arming...")
        print(vehicle.armed)
        time.sleep(1)
        
    # Run run run
    try:
        while(True):
            # Simulate the camera process overhead
            time.sleep(0.05)
            
            # Simulate an additional control delay
            time.sleep(1/30)
            
            # Values to be executed 
            C.rollAngle = math.radians(30)
            C.pitchAngle = math.radians(0)
            C.yawRate = 0
            C.thrust = 0.5
            
            # Execute the command
            C.sendAttitudeTarget()
            print("Sent")
            
    
    except KeyboardInterrupt:
        # Close the vehicle
        vehicle.close()
        print('Finished...')

# Main loop
if __name__ == '__main__':
    main()


