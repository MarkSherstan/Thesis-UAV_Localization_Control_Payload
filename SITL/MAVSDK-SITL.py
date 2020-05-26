#!/usr/bin/env python3

import asyncio

from mavsdk import System
from mavsdk import (Attitude, OffboardError)

# https://github.com/mavlink/MAVSDK-Python/issues/155

async def run():
    """ Does Offboard control using attitude commands. """
    
    # start_mavlink(connection_url="udp://127.0.0.1:14541")
    # drone = mavsdk_connect(host="127.0.0.1")

    # drone = System(mavsdk_server_address="127.0.0.1", port=14541)
    
    # drone = System(mavsdk_server_address="localhost", port=5760)
    drone = System()
    await drone.connect(system_address="udp://:14551") #system_address="tcp://:14541"            system_address="tcp://:5760"
    
    
    
    
    
    
    
    
    
    
    # drone = System()
    # await drone.connect(system_address="udp://:5502")
            #    - Serial: serial:///path/to/serial/dev[:baudrate]
            #    - UDP: udp://[bind_host][:bind_port]
            #    - TCP: tcp://[server_host][:server_port]

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    print("-- Arming")
    await drone.action.arm()

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