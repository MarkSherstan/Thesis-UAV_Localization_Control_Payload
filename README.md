# UAV-Sampling-Control-System
Control, DAQ, and actuation system for autonomous UAV sampler.

## Setup
Clone the repo and install all the dependencies. For the first time call
`git submodule update --init --recursive`. Otherwise update using `git submodule update --recursive`

## Payloads
Open the `/MASTER` PlatformIO directory and build the project. All pinout is labeled in `src/main.cpp`. The servo library may need to be re-installed with the following command: `pio lib install "Servo"` in the PlatformIO command line. Build the individual payloads similarly.

## Software In The Loop (SITL) Simulation
`dronekit-sitl` only has ArduCopter binaries up to version 3.3. As of May 2019 the ArduPilot GitHub repo has code up to 3.7-dev. Much of the current functionality from the new development is not captured in the `dronekit-sitl` old prebuilt binaries and therefore newer versions should be built locally. The process outlined below primarily outlines the process for UNIX / MacOS operating systems, however links are provided for other operating systems.

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

Navigate to the following directory through the command line to run the simulator.

```
cd ardupilot/Tools/autotest
python sim_vehicle.py -v ArduCopter -L UChicago
```

Once the simulator is running. Open another command line and navigate to the following directory and run the program to test a control scheme and simulate in the loop.

```
cd UAV-Sampling-Control-System/ultrasonicPositionControl/offboard-python/
python main-SITL.py
```

## Off Board Interfacing
A [companion computer](http://ardupilot.org/dev/docs/companion-computers.html#companion-computers) is used to connect with the flight controller to implement off board control. Follow the guide for the respective hardware for setting up. The python API used is outlined [here](https://web.archive.org/web/20180803235025/http://python.dronekit.io/guide/copter/guided_mode.html#guided-mode-copter).

Connect on `/dev/ttyS1` which are pins 8 and 10 on the J1 expansion connector of the Odroid C2. Further information can be found [here](https://wiki.odroid.com/odroid-c2/hardware/expansion_connectors) and a wiring diagram can be found [here](http://ardupilot.org/dev/_images/RaspberryPi_Pixhawk_wiring1.jpg) (RPi and Odroid C2 can be wired the same).

## Off Board (Guided No GPS) Control Python
The following packages should be installed in a `virtualenv` (additional may be required, follow any errors generated):

```
Package               Version
--------------------- --------
DateTime              4.3     
dronekit              2.9.2   
future                0.18.2  
imutils               0.5.3   
lxml                  4.4.1   
monotonic             1.5     
numpy                 1.17.4  
opencv-contrib-python 4.1.0.25
pandas                0.25.3  
pip                   19.3.1  
pymavlink             2.4.0   
pyserial              3.4     
python-dateutil       2.8.1   
pytz                  2019.3  
setuptools            41.6.0  
six                   1.13.0  
wheel                 0.33.6  
zope.interface        4.7.1   
```

Once the repo is cloned on the microprocessor ensure to `workon cv` (run the code in a virtual environement) and run the main program with the controller and vision modules in the `\control` directory.

To plot data (once saved) navigate to the `\flightData` directory and enter the file name and flight mode of interest in the command line as such: `python plotter.py --input "flight1.csv" --mode "GUIDED_NOGPS"`.
