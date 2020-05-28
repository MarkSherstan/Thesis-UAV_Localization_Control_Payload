import asyncio
import time
import utm

from mavsdk import System
from mavsdk import (Attitude, AttitudeRate, OffboardError, Telemetry)
from controller import Controller

async def getAttitude(drone):
    async for bodyAttitude in drone.telemetry.attitude_euler():
        return bodyAttitude.roll_deg, bodyAttitude.pitch_deg, bodyAttitude.yaw_deg

async def getPos(drone, xCal=0, yCal=0):
    async for pos in drone.telemetry.position():
        lat, lon, z = pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
        x, y, _, _ = utm.from_latlon(lat, lon)
        return (x-xCal)*1000, (y-yCal)*1000, z*1000

async def run(drone):
    # Connect to drone
    await drone.connect(system_address="udp://:14540")

    # Connect to control scheme 
    C = Controller(250, 0, 0)

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
    await drone.action.set_takeoff_altitude(2)
    await drone.action.takeoff()
    await asyncio.sleep(5)

    # Zero initial position
    xZero = 0
    yZero = 0
    for _ in range(100):
        tempX, tempY, _ = await asyncio.ensure_future(getPos(drone))
        xZero += tempX
        yZero += tempY

    xZero /= 100
    yZero /= 100

    # # Start offboard
    # print("-- Starting offboard")
    # await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.5))
    # try:
    #     await drone.offboard.start()
    # except OffboardError as error:
    #     print(f"Starting offboard mode failed with error code: \
    #           {error._result.result}")
    #     print("-- Disarming")
    #     await drone.action.disarm()
    #     return

    # Start controller timer
    C.startTime = time.time()

    while(True):
        # Get data 
        _, _, yaw = await asyncio.ensure_future(getAttitude(drone))
        x, y, z = await asyncio.ensure_future(getPos(drone, xCal=xZero, yCal=yZero))

        # Run control 
        rollControl, pitchControl, yawControl, thrustControl = await C.positionControl(x, y, z, yaw)         

        # Execute control
        await drone.offboard.set_attitude(Attitude(rollControl, pitchControl, 0.0, thrustControl))
        await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, yawControl, thrustControl))
        
        # Get current flight mode
        flightMode = await drone.telemetry.flight_mode()
        
        # Print results
        print(flightMode)
        print(x, y, z)
        print(rollControl, pitchControl, yawControl, thrustControl)
        print()
        await asyncio.sleep(0.1)
        
    # Once desired (with tolerance) reached -> break... (look at adding the buffer idea here)
    # Double check sampling rates 
    
    # Set attitude
    # await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.5))
       
    # print("-- Landing")
    # await drone.action.land()
    # await asyncio.sleep(1)


if __name__ == "__main__":
    # Connect to SITL
    drone = System()

    # Run the main program
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(drone))
    except KeyboardInterrupt:
        print("Closing")    
