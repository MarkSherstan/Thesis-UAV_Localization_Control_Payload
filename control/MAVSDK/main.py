import asyncio
import time 

from mavsdk import System
from mavsdk import (Attitude, OffboardError, Telemetry)

# Global storage variable
storage = []

async def getAttitude(drone):
    async for bodyAttitude in drone.telemetry.attitude_euler():
        return bodyAttitude.roll_deg, bodyAttitude.pitch_deg, bodyAttitude.yaw_deg

async def run():
    # Mac OS
    drone = System()
    await drone.connect(system_address="serial:///dev/cu.usbmodem14101:921600")

    # Linux
    # drone = System(mavsdk_server_address='localhost', port=50051)
    # await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # Set initial set point for attitiude (required)
    await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    # Start offboard (is this even required?) -> test
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Start timer 
    timer = time.time()
    global storage
    
    while(True):
        # Send attitude command
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.6))
                
        # Get current attitude
        roll, pitch, yaw = await asyncio.ensure_future(getAttitude(drone))

        # Log data and print
        print(1/(time.time()-timer), roll, pitch, yaw)
        timer = time.time()
        # storage.append([time.time(), 1/(time.time()-timer), roll, pitch, yaw])
        # time.sleep(0.001)
        
    # print("-- Stopping offboard")
    # try:
    #     await drone.offboard.stop()
    # except OffboardError as error:
    #     print(f"Stopping offboard mode failed with error code: \
    #           {error._result.result}")


if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("The end")    
        # print(storage)    
