from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import serial
import time

# Connect to the Vehicle
connection_string = "/dev/ttyS1"
print('Connecting to vehicle on: %s\n' % connection_string)
vehicle = connect(connection_string, wait_ready=["attitude"], baud=115200)

# Quick sleep then reboot
time.sleep(1)
vehicle.reboot()
time.sleep(1)

# End message
print('Rebooting...')
