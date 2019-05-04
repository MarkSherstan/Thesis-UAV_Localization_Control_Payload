from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import matplotlib.pyplot as plt
import matplotlib
import time
import math

##################################
# Function definitions
##################################

def arm_and_takeoff_nogps(aTargetAltitude):
    # Give the user some info
    print("Basic pre-arm checks")
    print("--------------------------------------------------")

    # Wait till vehicle is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")

    # Set vehicle mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Arm the vehicle
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)

    # Take off
    print("Taking off!")

    vehicle.simple_takeoff(aTargetAltitude)
    time.sleep(2)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5):
    # use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
    #
    # thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
    #         Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
    #         the code for maintaining current altitude.
    #             Thrust >  0.5: Ascend
    #             Thrust == 0.5: Hold the altitude
    #             Thrust <  0.5: Descend
    #
    # https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_LOCAL_NED

    if yaw_angle is None:
        yaw_angle = vehicle.attitude.yaw

    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        euler2quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )

    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0, yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5, duration = 0):
    # ATTITUDE_TARGET order has a timeout of 1s for Arducopter 3.3 and higher
    send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
    print 'Roll: ', round(vehicle.attitude.roll,4), ' Pitch: ', round(vehicle.attitude.pitch,4), ' Yaw: ', round(vehicle.attitude.yaw,4)

    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle, yaw_angle, yaw_rate, False, thrust)
        time.sleep(0.1)

    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0, 0, 0, True, thrust)

def euler2quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    # Euler angles (deg) to quaternion
    yc = math.cos(math.radians(yaw * 0.5))
    ys = math.sin(math.radians(yaw * 0.5))
    rc = math.cos(math.radians(roll * 0.5))
    rs = math.sin(math.radians(roll * 0.5))
    pc = math.cos(math.radians(pitch * 0.5))
    ps = math.sin(math.radians(pitch * 0.5))

    q0 = yc * rc * pc + ys * rs * ps
    q1 = yc * rs * pc - ys * rc * ps
    q2 = yc * rc * ps + ys * rs * pc
    q3 = ys * rc * pc - yc * rs * ps

    return [q0, q1, q2, q3]

##################################
# Main Function
##################################

# Connect to the Vehicle
connection_string = "127.0.0.1:14551"
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

# Take off to 2.5 m
print("Make connection")
arm_and_takeoff_nogps(2.5)
print("Take off to 2.5 m")
print 'N: ', round(vehicle.location.local_frame.north,4), ' E: ', round(vehicle.location.local_frame.east,4), ' D: ', round(vehicle.location.local_frame.down,4)
print(" ")

# Hold the position for 1 seconds
print("Hold position for 1 seconds")
set_attitude(duration = 1)
print 'N: ', round(vehicle.location.local_frame.north,4), ' E: ', round(vehicle.location.local_frame.east,4), ' D: ', round(vehicle.location.local_frame.down,4)
print(" ")

# Desired
northDesired = 2
kp = 2
actualList = []
desiredList = []
errorList = []
timeList = []
startTime = time.time()

# Run a quick controller
for ii in range(50):
    error = vehicle.location.local_frame.north - northDesired

    timeList.append(time.time() - startTime)
    actualList.append(vehicle.location.local_frame.north)
    desiredList.append(northDesired)
    errorList.append(error)

    P = kp * error
    D = 0
    controller = P + D

    set_attitude(pitch_angle = controller, thrust = 0.5, duration = 0.4)

    print 'N: ', round(vehicle.location.local_frame.north,4), ' E: ', round(vehicle.location.local_frame.east,4), ' D: ', round(vehicle.location.local_frame.down,4)
    print 'Actual: ', round(vehicle.location.local_frame.north,4), 'Error: ', round(error,4), 'Controller: ', round(controller,4)
    print(" ")

# Land the drone
print("\nSetting LAND mode...")
vehicle.mode = VehicleMode("LAND")

# Plot the results
fig, ax = plt.subplots()
ax.plot(timeList,actualList,timeList,desiredList)

ax.set(xlabel='Time (s)', ylabel='position (m)', title='Position Control North')

plt.gca().legend(('actual','desired'))
ax.grid()

fig.savefig("test.png")
plt.show()

# Close vehicle object
print("Close vehicle and complete")
vehicle.close()
