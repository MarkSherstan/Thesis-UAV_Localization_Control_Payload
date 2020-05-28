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



## Install Steps (Clean this up)
https://mavsdk.mavlink.io/develop/en/contributing/build.html

sudo apt-get update -y
sudo apt-get install cmake build-essential colordiff git doxygen -y

cd
git clone https://github.com/mavlink/MAVSDK.git
cd MAVSDK


git checkout master
git submodule update --init --recursive

cmake -DCMAKE_BUILD_TYPE=Debug -DBUILD_SHARED_LIBS=ON -Bbuild/default -H.
cmake --build build/default




sudo cmake --build build/default --target install # sudo is required to install to system directories!

# First installation only
sudo ldconfig  # update linker cache






cd MAVSDK
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_BACKEND=ON -Bbuild/default -H.
cmake --build build/default



sudo cmake --build build/default --target install # sudo is required to install to system directories!
sudo ldconfig  # update linker cache

sudo reboot -h now 



tmux (or tmux attach)
cd /home/odroid/MAVSDK/build/default/src/backend/src
./mavsdk_server -p 50051 serial:///dev/ttyS1:921600
cntrl+b 
d

workon cv
cd /home/odroid/Desktop/UAV-Sampling-Control-System/control
python mainMAVSDK.py






cd
git clone https://github.com/mavlink/MAVSDK-Python --recursive
cd MAVSDK-Python

cd proto/pb_plugins
pip3 install -r requirements.txt
sudo pip3 install -e .

cd ../..
pip3 install -r requirements.txt -r requirements-dev.txt
(takes forever)

./other/tools/run_protoc.sh


cp /home/odroid/MAVSDK/build/default/src/backend/src/mavsdk_server /home/odroid/MAVSDK-Python/mavsdk/bin


# https://discuss.px4.io/t/talking-to-a-px4-fmu-with-a-rpi-via-serial-nousb/14119
# This doesent seem to work...   
python3 setup.py build

pip3 install -e .


Add it to our virtual environment:

workon cv
cd /home/odroid/MAVSDK-Python
pip install -e .




