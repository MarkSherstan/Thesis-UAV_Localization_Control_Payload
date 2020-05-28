# Notes

Client code must specify a setpoint before starting offboard mode.
Mavsdk automatically resends setpoints at 20Hz (PX4 Offboard mode requires that setpoints are minimally resent at 2Hz). If more precise control is required, clients can call the setpoint methods at whatever rate is required.
https://github.com/mavlink/MAVSDK/pull/134


## Run this on Ubuntu (had to build from source) - no pip
```
tmux (or tmux attach)
cd /home/odroid/MAVSDK/build/default/src/backend/src
./mavsdk_server -p 50051 serial:///dev/ttyS1:921600
cntrl+b 
d
```

Set parameter SRx_EXTRA1 and reboot, the following does not appear to do anything:
    # drone.telemetry.set_rate_attitude(20)
    # await asyncio.sleep(3)


TO DO:
    # Need to run a simulation 
    # YAPPI profile: https://github.com/sumerc/yappi

    # Startup shell script for server -> add to Python?

    # Crashing after 30 secounds... 
    # Old FC has very irregular data rates... 
    