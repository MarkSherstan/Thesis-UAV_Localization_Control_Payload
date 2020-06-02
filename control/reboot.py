import asyncio
from mavsdk import System
from mavsdk import (Action)

async def run():
    # Mac OS
    # drone = System()
    # await drone.connect(system_address="serial:///dev/cu.usbmodem14101:921600")

    # Ubuntu
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()

    # Connect to the drone
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break
    
    # Reboot it
    await drone.action.reboot()
    await asyncio.sleep(2)

    # Reboot complete
    print("Reboot complete")

if __name__ == "__main__":
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())