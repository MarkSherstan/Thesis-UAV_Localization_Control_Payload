# UAV-Sampling-Control-System
Control, DAQ, and actuation system for autonomous UAV sampler.

## Setup
Clone the repo and install all the dependencies. For the first time call
`git submodule update --init --recursive`. Otherwise update using `git submodule update --recursive`

## Payloads
Open the `/MASTER` PlatformIO directory and build the project. All pinout is labeled in `src/main.cpp`. The servo library may need to be re-installed with the following command: `pio lib install "Servo"` in the PlatformIO command line. Build the individual payloads similarly.

## Off Board Interfacing
A [companion computer](http://ardupilot.org/dev/docs/companion-computers.html#companion-computers) is used to connect with the flight controller to implement off board control. Follow the guide for the respective hardware for setting up. The python API used is outlined [here](https://web.archive.org/web/20180803235025/http://python.dronekit.io/guide/copter/guided_mode.html#guided-mode-copter).

Connect on `/dev/ttyS1` which are pins 8 and 10 on the J1 expansion connector of the Odroid C2. Further information can be found [here](https://wiki.odroid.com/odroid-c2/hardware/expansion_connectors) and a wiring diagram can be found [here](http://ardupilot.org/dev/_images/RaspberryPi_Pixhawk_wiring1.jpg) (RPi and Odroid C2 can be wired the same).

## Off Board (Guided No GPS) Control Python
A `virtualenv` is used to house all the code development. Once the repo is cloned on the microprocessor ensure to `workon cv` (run the code in a virtual environement) and run the main program with the controller and vision modules in the `\control` directory.

To plot data (once saved) navigate to the `\flightData` directory and enter the file name and flight mode of interest in the command line as such: `python plotter.py --input "flight1.csv" --mode "GUIDED_NOGPS"`.

### Other
Install Tmux with `sudo apt install tmux`

Enter `tmux` to creat a session. 
Enter `control+b` then `d` to exit the session 
Enter `tmux attach` to rejoin a session 

To see memory and storage use `free -m` and `df -h` respectively. 

## To Do 
* Clean up documentation 
