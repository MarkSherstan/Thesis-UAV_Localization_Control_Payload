# Off Board (Guided No GPS) Control Python
Python 2.7 is required. Python 3 and higher is not fully supported. The following packages should be installed (additional may be required, follow any errors generated).

```
pip install dronekit
pip install dronekit-sitl  
pip install MAVProxy
pip install pymavlink
```

## Software In The Loop (SITL) Simulation
`dronekit-sitl` only has ArduCopter binaries up to version 3.3. As of May 2019 the ArduPilot GitHub repo has code up to 3.7-dev. Much of the current functionality from the new development is not captured in `dronekit-sitl` and therefore newer versions should be built to run an accurate simulation. The process outlined below primarily outlines the process for UNIX / MacOS operating systems, however links will be provided for other operating systems and more detailed instructions.

Start by cloning the [ArduPilot](https://github.com/ArduPilot/ardupilot) repo and navigate to new directory.

```
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot/
```

Build the most up to date [binary](https://github.com/ArduPilot/ardupilot/blob/master/BUILD.md). Further information can be found as follows.
* [https://www.youtube.com/watch?v=PLGezPQYMrs&t=331s](https://www.youtube.com/watch?v=PLGezPQYMrs&t=331s)
* [https://www.youtube.com/watch?v=wLK2wLwEXm4&t=22s](https://www.youtube.com/watch?v=wLK2wLwEXm4&t=22s)
* [http://ardupilot.org/dev/docs/building-the-code.html](http://ardupilot.org/dev/docs/building-the-code.html)

```
git submodule update --init --recursive
./waf configure
./waf copter
```

Navigate to the following directory through the command line and run the simulator.

```
cd ardupilot/Tools/autotest
python sim_vehicle.py -v ArduCopter
```

Once the simulator is running. Open another command line and navigate to the following directory and run the program to test your control scheme.

```
cd UAV-Sampling-Control-System/ultrasonicPositionControl/offboard-python/
python main-SITL.py
```

## Off Board Interfacing
A [companion computer](http://ardupilot.org/dev/docs/companion-computers.html#companion-computers) is used to connect with the flight controller to implement off board control. Follow the guide for the respective hardware for setting up. To develop code connect on a desired serial port and use any of the commands outlined [here](https://web.archive.org/web/20180803235025/http://python.dronekit.io/guide/copter/guided_mode.html#guided-mode-copter).
