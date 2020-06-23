from dronekit import connect, VehicleMode
from controller import Controller
from pymavlink import mavutil
import numpy as np 
import time
import math
import pandas as pd

from IMU import MyVehicle

class TEST:
    def __init__(self, vehicle):
        # Vehicle class
        self.UAV = vehicle
                
    def euler2quaternion(self, roll, pitch, yaw):
        # Convert degrees to radians 
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        # Euler angles (rad) to quaternion
        yc = math.cos(yaw * 0.5)
        ys = math.sin(yaw * 0.5)
        rc = math.cos(roll * 0.5)
        rs = math.sin(roll * 0.5)
        pc = math.cos(pitch * 0.5)
        ps = math.sin(pitch * 0.5)

        q0 = yc * rc * pc + ys * rs * ps
        q1 = yc * rs * pc - ys * rc * ps
        q2 = yc * rc * ps + ys * rs * pc
        q3 = ys * rc * pc - yc * rs * ps

        return [q0, q1, q2, q3]

    def sendAttitudeTarget(self, roll, pitch, yawRate, thrustRate):
        # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
        # Set yaw to current yaw value 
        #                                       (try changing this later to just 0)
        yaw = 0 #self.UAV.attitude.yaw
        yawRate = math.radians(yawRate)

        # Create the mavlink message
        msg = self.UAV.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            0, # Target system
            0, # Target component
            0b00000000, # If bit is set corresponding input ignored (mappings)
            self.euler2quaternion(roll, pitch, yaw), # Quaternion
            0, # Body roll rate in radian
            0, # Body pitch rate in radian
            yawRate,   # Body yaw rate in rad/s
            thrustRate # Thrust
        )
        
        # Send the constructed message
        self.UAV.send_mavlink(msg)

        print(roll, pitch)
        
def main():
    # Connect to the Vehicle
    connection_string = "/dev/ttyS1" # "/dev/cu.usbmodem14201"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=1500000, vehicle_class=MyVehicle)

    # Set attitude request message rate (everything else is default 4 Hz)
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
        150, # Rate (Hz)
        1)  # Turn on
    vehicle.send_mavlink(msg)    
    time.sleep(0.5)
    # msg = self.UAV.message_factory.command_long_encode(
    # 	0, 0, #target system, target component
    # 	mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, #command
    # 	0, #confirmation
    # 	27, #param 1
    # 	300000, #param 2
    # 	0, 0, 0, 0, 0) #param 3-7 not used
    
    msg = vehicle.message_factory.request_data_stream_encode(
        0, 0,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
        150, # Rate (Hz)
        1)  # Turn on
    vehicle.send_mavlink(msg)  
    time.sleep(0.5)

    # Connect to class 
    t = TEST(vehicle)
    
    # Wait till we switch modes to prevent integral windup
    # while(vehicle.mode.name != 'GUIDED_NOGPS'):
    #     print(vehicle.mode.name)
    #     time.sleep(0.2)
    currentValue = None
    previousValue = 0
    count = 0
    
    # Start timers
    startTime = time.time()
    
    # Try this
    try:
        while(time.time() < startTime + 10):
            # print(vehicle.attitude.roll)
            # print(vehicle.raw_imu.zgyro * (180 / (1000 * np.pi)))
            currentValue = vehicle.raw_imu.xacc
            print(currentValue)
            if (currentValue != previousValue):
                count += 1
                
            previousValue = currentValue
            # time.sleep(1/40)
                    
    except KeyboardInterrupt:
        
        # Print final remarks
        print('Closing')
    finally:     
        endTime = time.time()
        # End and disconnect          
        vehicle.close()
        print('Vehicle closed')
        print('Rate: ', count / (endTime - startTime))

if __name__ == "__main__":
    main()
