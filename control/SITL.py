from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import time 

# Connect to the Vehicle
connection_string = "127.0.0.1:14551"
print('Connecting to vehicle on: %s\n' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

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
targetAltitude = 3.0
print("Taking off!")
vehicle.simple_takeoff(targetAltitude)

# Wait until actual altitude is achieved
while abs(vehicle.location.local_frame.down) <= (targetAltitude*0.9):
    print(round(vehicle.location.local_frame.down,3))
    time.sleep(0.2)

# Land the UAV and close connection
print("Closing vehicle connection\n")
vehicle.mode = VehicleMode("LAND")
vehicle.close()

