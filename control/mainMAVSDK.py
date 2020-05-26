import asyncio

from mavsdk import System
from mavsdk import (Attitude, OffboardError, Telemetry)


async def print_attitude(drone):
    async for bodyAttitude in drone.telemetry.attitude_euler():
        print(bodyAttitude.roll_deg)
        

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

    print("-- Roll 30 at 60% thrust")
    await drone.offboard.set_attitude(Attitude(30.0, 0.0, 0.0, 0.6))
    await asyncio.sleep(2)
    
    
    for ii in range(10):
        asyncio.ensure_future(print_attitude(drone))
        await asyncio.sleep(0.2)
  

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