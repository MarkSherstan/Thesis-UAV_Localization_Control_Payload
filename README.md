# UAV-Sampling-Control-System
Control, DAQ, and actuation system for autonomous modular UAV system.

## Setup
Clone the repo and install all the dependencies. For the first time call
`git submodule update --init --recursive`. Otherwise update using `git submodule update --recursive`.

For information regarding building required packages refer to the [wiki](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki). Some helpful links:
* [Build OpenCV](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki/Build-OpenCV)
* [Build Pyrealsense2](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki/Build-Pyrealsense2)
* [Ubuntu Install & Setup](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki/Ubuntu-20.04-LTS-Set-Up)
* [Gazebo SITL-ArduPilot](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki/Gazebo-ArduPilot-SITL-Setup)

## Control 
### Off Board Interfacing
A [companion computer](http://ardupilot.org/dev/docs/companion-computers.html#companion-computers) is used to connect with the flight controller to implement off board control. I am currently using an Odroid C4 with Ubuntu 20.04. 

Connect on `/dev/ttyS1` which are pins 8 and 10 on the J1 expansion connector of the Odroid C4. Further information can be found [here](https://wiki.odroid.com/odroid-c2/hardware/expansion_connectors) and a wiring diagram can be found [here](http://ardupilot.org/dev/_images/RaspberryPi_Pixhawk_wiring1.jpg) (RPi and Odroid C4 can be wired the same).

### Off Board (Guided No GPS) Control Python
A `virtualenv` is used to house all the required dependincies. Once the repo is cloned on the microprocessor ensure to enter `workon cv` (run the code in a virtual environement) and run the main program with the controller and vision modules in the `\control` directory. Due to the T265 camera `sudo` must be used when running a script using the camera (e.g. `sudo python3 main.py`)

To plot data (once saved) navigate to the `\flightData` directory and enter the file name in the command line as such: `python3 plotter.py --input "flight1.csv"`.

## Payloads
Open the `/MASTER` PlatformIO directory and build the project. All pinout is labeled in `src/main.cpp`. The servo library may need to be re-installed with the following command: `pio lib install "Servo"` in the PlatformIO command line. Build the individual payloads similarly.

## PCB
The PCB's were designed with [KiCad 5](https://kicad-pcb.org/download/) and manufactured by [JLCPCB](https://jlcpcb.com).

## To Do
* Instructions on how to use (e.g. - set up FC)
* Move everything to wiki and/or explain better
