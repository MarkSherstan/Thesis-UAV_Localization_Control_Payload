import statistics
import asyncio
import time 

from mavsdk import System
from mavsdk import (Attitude, AttitudeRate, OffboardError, Telemetry)

from multiprocessing import Process, Queue
from controller import Controller
from vision import Vision

# Global data variable
data = []
freqList = []

async def getAttitude(drone):
    try:
        async for bodyAttitude in drone.telemetry.attitude_euler():
            return bodyAttitude.roll_deg, bodyAttitude.pitch_deg, bodyAttitude.yaw_deg
    except:
        print('ERROR: Get Attitude')
        return -1, -1, -1

def getVision(Q):
    temp = Q.get()
    return temp[0], temp[1], temp[2], temp[3]
    
async def run():
    # Connect to the drone
    drone = System(mavsdk_server_address='localhost', port=50051)
    await drone.connect()

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # Set message rate
    await drone.telemetry.set_rate_attitude(35)
    await asyncio.sleep(1) 
    
    # Connect to control scheme 
    C = Controller(250, 0, 0)
 
    # Connect to vision, create the queue, and start the core
    V = Vision()
    Q = Queue()
    P = Process(target=V.processFrame, args=(Q, ))
    P.start()

    # Client code must specify a setpoint before starting offboard mode.
    for _ in range(10):
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 0.0, 0.0))

    # Variables
    global data
    global freqList
    sleepRate = 0.01
    
    # Start timers
    startTime = time.time()
    loopTimer = time.time()
    C.timer = time.time()
    
    while(True):
        # Get vision data
        northV, eastV, downV, yawV = getVision(Q)
        
        # Calculate controller outputs
        rollControl, pitchControl, yawControl, thrustControl = C.positionControl(northV, eastV, downV, yawV)         

        # Execute control
        try:
            await drone.offboard.set_attitude(Attitude(rollControl, pitchControl, yaw, thrustControl))
            await asyncio.sleep(sleepRate)
            await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, yawControl, thrustControl))
            await asyncio.sleep(sleepRate)
        except:
            print('ERROR: Execute Control')
        
        # Get current attitude
        roll, pitch, yaw = await getAttitude(drone)
        await asyncio.sleep(sleepRate)

        # Print data
        freqLocal = (1 / (time.time() - loopTimer))
        freqList.append(freqLocal)
        print('f: {:<8.0f} N: {:<8.0f} E: {:<8.0f} D: {:<8.0f} Y: {:<8.0f}'.format(freqLocal, northV, eastV, downV, yawV))
		print('R: {:<8.2f} P: {:<8.2f} Y: {:<8.2f} r: {:<8.2f} p: {:<8.2f} y: {:<8.2f} t: {:<8.2f}'.format(roll, pitch, yaw, rollControl, pitchControl, yawControl, thrustControl))
        loopTimer = time.time()
        
        # Log data
        data.append([time.time-startTime, freqLocal, northV, eastV, downV, yawV ])
        
          
if __name__ == "__main__":
    loop = asyncio.get_event_loop()

    try:
        loop.run_until_complete(run())
    except KeyboardInterrupt:
        print("Closing...") 
        print("Average loop rate: ", round(statistics.mean(freqList),2))
        print("Standard dev: ", round(statistics.stdev(freqList), 2))
        
        # print(data) -> Add pandas logging here 


# Usage
#   tmux (or tmux attach)
#   cd /home/odroid/MAVSDK/build/default/src/backend/src
#   ./mavsdk_server -p 50051 serial:///dev/ttyS1:921600
#   cntrl+b 
#   d
#
#   workon cv
#   cd ~/UAV-Sampling-Control-System/control
#   python main.py
