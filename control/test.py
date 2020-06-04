from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative, LocationLocal
from pymavlink import mavutil
import math
import time

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


    # Previous variables
    pRoll = 0
    pPitch = 0 
    pYaw = 0
    
    # Performance vars
    count = 0
    startTime = time.time()

    try:
        while(time.time() < startTime+5):
            cRoll = vehicle.attitude.roll
            cPitch = vehicle.attitude.pitch
            cYaw = vehicle.attitude.yaw

            if ((cRoll != pRoll) or (cPitch != pPitch) or (cYaw != pYaw)):
                count += 1

            pRoll = cRoll
            pPitch = cPitch
            pYaw = cYaw

            # print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f}'.format(math.degrees(cRoll), math.degrees(cPitch), math.degrees(cYaw)))
            # time.sleep(1/10)
    except KeyboardInterrupt:
        print('Exiting')
    finally:
        print(round(count / (time.time() - startTime)), " Hz")
        vehicle.close()

# Main loop
if __name__ == '__main__':
    main()