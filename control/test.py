from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import math
import time

def euler2quaternion(roll, pitch, yaw):
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

def sendAttitudeTarget(UAV, var):
    # https://mavlink.io/en/messages/common.html#SET_ATTITUDE_TARGET
    #
    # thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
    #         Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
    #         the code for maintaining current altitude.
    #             Thrust >  0.5: Ascend
    #             Thrust == 0.5: Hold the altitude
    #             Thrust <  0.5: Descend
    #
    # Mappings: If any of these bits are set, the corresponding input should be ignored.
    # bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate.
    # bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude

    # Prevent none type error on yaw
    yawAngle = UAV.attitude.yaw

    # Create the mavlink message
    msg = UAV.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        0, # Target system
        0, # Target component
        0b00000000, # If bit is set corresponding input ignored (mappings)
        euler2quaternion(var, 0.3, yawAngle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        0.3, # Body yaw rate in radian/second
        0.4 # Thrust
    )

    # Send the constructed message
    UAV.send_mavlink(msg)
    

def main():
    # Connect to the Vehicle
    connection_string = "/dev/ttyS1"
    print('Connecting to vehicle on: %s\n' % connection_string)
    vehicle = connect(connection_string, wait_ready=["attitude"], baud=1500000) #, source_system=1, source_component=1)

    # Set attitude request message rate (everything else is default 4 Hz)
    msg = vehicle.message_factory.request_data_stream_encode(
    0, 0,
    mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
    100, # Rate (Hz)
    1)  # Turn on
    vehicle.send_mavlink(msg)


    # Previous variables
    pRoll = 0
    pPitch = 0 
    pYaw = 0
    
    # Performance vars
    attitudeCount = 0
    loopCount = 0
    startTime = time.time()
    var = -0.1

    try:
        while(time.time() < startTime+10):
            print(var)
            cRoll = vehicle.attitude.roll
            cPitch = vehicle.attitude.pitch
            cYaw = vehicle.attitude.yaw

            if ((cRoll != pRoll) or (cPitch != pPitch) or (cYaw != pYaw)):
                attitudeCount += 1

            pRoll = cRoll
            pPitch = cPitch
            pYaw = cYaw

            sendAttitudeTarget(vehicle, var)
            loopCount += 1
            time.sleep(1/30)
            var += 0.001

            # print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f}'.format(math.degrees(cRoll), math.degrees(cPitch), math.degrees(cYaw)))
            # time.sleep(1/10)
    except KeyboardInterrupt:
        print('Exiting')
    finally:
        print("Attitude: ", round(attitudeCount / (time.time() - startTime)), " Hz")
        print("Loop: ", round(loopCount / (time.time() - startTime)), " Hz")
        vehicle.close()

# Main loop
if __name__ == '__main__':
    main()