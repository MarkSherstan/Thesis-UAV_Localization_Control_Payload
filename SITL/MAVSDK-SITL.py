import asyncio
import time

from mavsdk import System
from mavsdk import (Attitude, AttitudeRate, OffboardError, Telemetry)


async def getAttitude(drone):
    async for bodyAttitude in drone.telemetry.attitude_euler():
        return bodyAttitude.roll_deg, bodyAttitude.pitch_deg, bodyAttitude.yaw_deg

async def getPosition(drone):
    async for pos in drone.telemetry.position():
        return pos.relative_altitude_m

async def run():
    # Connect to SITL
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # Connect to UAV
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("-- Arming")
    await drone.action.arm()

    print("-- Taking off")
    await drone.action.takeoff()

    # print("-- Setting initial setpoint")
    # await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    # print("-- Starting offboard")
    # try:
    #     await drone.offboard.start()
    # except OffboardError as error:
    #     print(f"Starting offboard mode failed with error code: \
    #           {error._result.result}")
    #     print("-- Disarming")
    #     await drone.action.disarm()
    #     return

    for _ in range(100):
        # Get current attitude
        roll, pitch, yaw = await asyncio.ensure_future(getAttitude(drone))
        h = await asyncio.ensure_future(getPosition(drone))
        print(h, roll, pitch, yaw)
        await asyncio.sleep(0.1)
    
    # await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
    # await asyncio.sleep(2)

    # https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position_body.html
    # https://mavsdk.mavlink.io/develop/en/api_reference/structmavsdk_1_1_telemetry_1_1_position_ned.html

    # Get current NED points 
    # Run control in while loop here
    # Once reached break... (look at adding the buffer idea here)
    # land 

    # time.sleep(4)
    # await asyncio.sleep(5)

    print("-- Landing")
    await drone.action.land()

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())