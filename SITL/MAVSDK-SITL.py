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
        return (x-xCal), (y-yCal), z

async def stabilizeLoop(stabilizeTimer):
    # Set parameters 
    totalLoopTime = 1/25

    # Find current loop time and execute corresponding delay
    timeDiff = time.time() - stabilizeTimer

    if (timeDiff >= totalLoopTime):
        pass
    else:
        await asyncio.sleep(totalLoopTime - timeDiff)

async def run(drone):
    # Connect to drone
    await drone.connect(system_address="udp://:14540")

    # Connect to control scheme 
    C = Controller(0, 0, 2000)

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

    # print("-- Taking off")
    # await drone.action.set_takeoff_altitude(2)
    # await drone.action.takeoff()
    # await asyncio.sleep(5)

    # Zero initial position
    xZero = 0
    yZero = 0
    # for _ in range(100):
    #     tempX, tempY, _ = await asyncio.ensure_future(getPos(drone))
    #     xZero += tempX
    #     yZero += tempY

    # xZero /= 100
    # yZero /= 100

    # Start offboard
    print("-- Starting offboard")
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Start timers
    C.timer = time.time()
    startTime = time.time()
    stabilizeTimer = time.time()

    while(time.time() < startTime+8):
        # Get data 
        _, _, yaw = await asyncio.ensure_future(getAttitude(drone))
        x, y, z = await asyncio.ensure_future(getPos(drone, xCal=xZero, yCal=yZero))

        # Run control 
        rollControl, pitchControl, yawControl, thrustControl = await C.positionControl(x*1000, y*1000, z*1000, yaw)         

        # Execute control
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, thrustControl))
        await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, 0.0, thrustControl))
                
        # Stabilize the sampling rate
        await stabilizeLoop(stabilizeTimer)
        stabilizeTimer = time.time()
        
    # Once desired (with tolerance) reached -> break... (look at adding the buffer idea here)
    # Double check sampling rates 
       
    print("-- Landing")
    await drone.action.land()
    await asyncio.sleep(2)


if __name__ == "__main__":
    # Connect to SITL
    drone = System()

    # Run the main program
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(drone))
    except KeyboardInterrupt:
        print("Closing")    
