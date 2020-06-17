from dronekit import connect, VehicleMode
from pymavlink import mavutil
import serial
import time

# Connect to the Vehicle
connection_string = "/dev/ttyS1"
print('Connecting to vehicle on: %s\n' % connection_string)
vehicle = connect(connection_string, wait_ready=["attitude"], baud=1500000)

# Quick sleep then reboot
time.sleep(1)
vehicle.reboot()
time.sleep(1)

# End message
print('Rebooting...')
