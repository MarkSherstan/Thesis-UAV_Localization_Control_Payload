import statistics
import asyncio
import time
import utm

from mavsdk import System
from mavsdk import (Attitude, AttitudeRate, OffboardError, Telemetry)
from controller import Controller

# Global variable(s)
data = []

def plotter(rawData, desiredNorth, desiredEast, desiredDown):
    # Import required package
    import matplotlib.pyplot as plt

    # Local variables
    timeData = []
    north = []
    east = []
    down = []
    yawData = []

    # Unpack the data
    for ii in range(len(rawData)):
        timeData.append(rawData[ii][0]) 
        east.append(rawData[ii][1])
        north.append(rawData[ii][2])
        down.append(rawData[ii][3])
        yawData.append(rawData[ii][4])

    # Prep for plotting
    _, (ax1, ax2) = plt.subplots(2)

    # Plot NED
    ax1.axhline(y=desiredNorth/1000, color='tab:orange')
    ax1.plot(timeData, north, color='tab:orange', linestyle='--')

    ax1.axhline(y=desiredEast/1000, color='tab:green')
    ax1.plot(timeData, east, color='tab:green', linestyle='--')

    ax1.axhline(y=desiredDown/1000, color='tab:blue')
    ax1.plot(timeData, down, color='tab:blue', linestyle='--')

    # Format the plot
    ax1.legend(['N Desired', 'N Actual', 'E Desired', 'E Actual', 'D Desired', 'D Actual'], ncol=3, loc='upper center')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Position [m]')
    _, ymax = ax1.get_ylim()
    ax1.set_ylim(top=ymax+1)

    # Plot yaw
    ax2.axhline(y=0, color='k')
    ax2.plot(timeData, yawData, color='k', linestyle='--')
    ax2.legend(['Yaw Desired', 'Yaw Actual'])
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Angle [deg]')
    ax2.set_ylim(-10,10)

    # Show the plot
    plt.show()

async def getAttitude(drone):
    try:
        async for bodyAttitude in drone.telemetry.attitude_euler():
            return bodyAttitude.roll_deg, bodyAttitude.pitch_deg, bodyAttitude.yaw_deg
    except:
        print('ERROR: Get Attitude')
        return 0, 0, 0

async def getPos(drone, xCal=0, yCal=0):
    try:
        async for pos in drone.telemetry.position():
            lat, lon, z = pos.latitude_deg, pos.longitude_deg, pos.relative_altitude_m
            x, y, _, _ = utm.from_latlon(lat, lon)
            return (x-xCal), (y-yCal), z
    except:
        print('ERROR: Get Position')
        return 0, 0, 0

async def run(drone, setNorth, setEast, setDown):
    # Connect to drone
    await drone.connect(system_address="udp://:14540")

    # Connect to control scheme 
    C = Controller(setNorth, setEast, setDown)

    # Connect to UAV
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"Drone discovered with UUID: {state.uuid}")
            break

    # Set message rate
    await drone.telemetry.set_rate_attitude(35)
    await drone.telemetry.set_rate_position(35)

    # Connect to "camera" -> GPS
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok:
            print("Global position estimate ok")
            break

    print("-- Arming")
    await drone.action.arm()

    # print("-- Taking off")
    # await drone.action.set_takeoff_altitude(1.5)
    # await drone.action.takeoff()
    # time.sleep(5)

    # Zero initial position
    xZero = 0
    yZero = 0
    for _ in range(100):
        tempX, tempY, _ = await getPos(drone)
        xZero += tempX
        yZero += tempY

    xZero /= 100
    yZero /= 100

    # Start offboard
    print("-- Starting offboard")
    for _ in range(10):
        await drone.offboard.set_attitude(Attitude(0.0, 0.0, 5.0, 0.0))

    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: \
              {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Small pause before starting and set sleep rates
    await asyncio.sleep(1)
    sleepRate = 0.01

    # Data variables 
    global data
    freqList = []

    # Start timers
    C.timer = time.time()
    startTime = time.time()
    loopTimer = time.time()

    while(time.time() < startTime+10):
        # Get data 
        _, _, yaw = await getAttitude(drone)
        await asyncio.sleep(sleepRate)
        x, y, z = await getPos(drone, xCal=xZero, yCal=yZero)
        await asyncio.sleep(sleepRate)

        # Run control scheme (y is North, x is East at 0 heading)
        rollControl, pitchControl, yawControl, thrustControl = C.positionControl(y*1000, x*1000, z*1000, yaw)         

        # Execute control
        try:
            await drone.offboard.set_attitude(Attitude(rollControl, pitchControl, yaw, thrustControl))
            await asyncio.sleep(sleepRate)
            await drone.offboard.set_attitude_rate(AttitudeRate(0.0, 0.0, yawControl, thrustControl))
            await asyncio.sleep(sleepRate)
        except:
            print('ERROR: Execute Control')

        # Save data for plotting
        data.append([time.time()-startTime, x, y, z, yaw])

        # Record and display sampling rate
        freqLocal = (1 / (time.time() - loopTimer))
        freqList.append(freqLocal)
        print(round(freqLocal))
        loopTimer = time.time()

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
              {error._result.result}")

    print("-- Landing")
    await drone.action.land()
    await asyncio.sleep(2)

    # Print samling rate info
    print("Average loop rate: ", round(statistics.mean(freqList),2))
    print("Standard dev: ", round(statistics.stdev(freqList), 2))

if __name__ == "__main__":
    # Set desired position
    setNorth = 750
    setEast  = 500
    setDown  = 1000

    # Connect to SITL
    drone = System()

    # Run the main program
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run(drone, setNorth, setEast, setDown))
    except KeyboardInterrupt:
        # Display message
        print("Closing")
    finally:
        # Plot the results
        plotter(data, setNorth, setEast, setDown)
