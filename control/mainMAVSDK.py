import asyncio
import time 

from mavsdk import System
from mavsdk import (Attitude, OffboardError, Telemetry)

# Client code must specify a setpoint before starting offboard mode.
# Mavsdk automatically resends setpoints at 20Hz (PX4 Offboard mode requires that setpoints are minimally resent at 2Hz). If more precise control is required, clients can call the setpoint methods at whatever rate is required.

async def print_attitude(drone):
    async for bodyAttitude in drone.telemetry.attitude_euler():
        print(time.time(), ",", bodyAttitude.roll_deg, ",", bodyAttitude.pitch_deg, ",", bodyAttitude.yaw_deg)

async def run():
    """ Does Offboard control using attitude commands. """
    drone = System()
    await drone.connect(system_address="serial:///dev/cu.usbmodem14101:921600") #serial://[Dev_Node][:Baudrate]

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # print("-- Arming")
    # await drone.action.arm()

    print("-- Setting initial setpoint")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return


    # Set telemtry to 20 Hz 
    # await drone.telemetry.set_rate_attitude(20)
    # https://github.com/mavlink/MAVSDK-Python/blob/9ac7113a88f49281c698997c55e9d46f8a8fd02c/mavsdk/generated/telemetry.py#L3553
    # await drone.telemetry.set_rate_attitude(20)
    # await asyncio.sleep(2)
    
    
    await asyncio.sleep(2)
    asyncio.ensure_future(print_attitude(drone))

    print("-- Roll 30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)
        
    print("-- Roll -30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(-30.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)

    print("-- Roll 0 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())