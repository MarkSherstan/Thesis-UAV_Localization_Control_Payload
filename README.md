# Thesis-UAV_Localization_Control_Payload
MSc thesis code for Localization and Control of a Quadcopter Universal Payload System.

## Abstract
There is a growing trend of using unmanned aerial vehicles (also known as UAVs, uncrewed aerial vehicles, or drones) to manipulate and interact with their surroundings. The algorithms and tools used are typically unique to the different tasks performed by UAVs; however, the fundamental UAV system largely remains consistent. This thesis offers a general quadrotor UAV system capable of performing multiple tasks dependent on available payloads. The system is empirically developed and performs localization using ArUco fiducial markers, an inertial measurement unit, and a Kalman filter. Navigation is achieved using a proportional integral derivative controller, and interfacing with various payloads is accomplished with a custom-developed universal payload manipulator. The system is described in detail and demonstrated with real-world experimentation, including the deployment of an example payload for removing twist caps off sump lubricant reservoirs. Empirical results show the developed system as an initial functional prototype in ideal conditions, and further work is required to increase system reliability and functionality for non-ideal scenarios.

## Setup
Clone the repo and install all the dependencies. For the first time call
`git submodule update --init --recursive`. Otherwise update using `git submodule update --recursive`.

Most of the developed code uses Python where a list of all the imported packages can be viewed in the below table. A majority of the Python packages come preinstalled; however, additional packages should be installed with `pip` (denoted with *). Two packages must be built directly from source code (denoted with **). All software ran on a Jetson Nano (quadcore ARM A57 processor) with Python 3.6.9 and Ubuntu 18.04.5 LTS.

| Package         |Version | \| | Package        |Version |
|-----------------|--------|----|----------------|--------|
| datetime        | 3.6.9  | \| | pymavlink*     | 2.4.9  |
| DroneKit*       | 2.9.2  | \| | pyrealsense2** | 2.36.0 |
| math            | 3.6.9  | \| | pySerial*      | 3.4    |
| Matplotlib*     | 3.3.1  | \| | simdkalman*    | 1.0.1  |
| multiprocessing | 3.6.9  | \| | statistics     | 3.6.9  |
| NumPy*          | 1.18.4 | \| | struct         | 3.6.9  |
| openCV**        | 4.3.0  | \| | threading      | 3.6.9  |
| pandas*         | 1.1.0  | \| | time           | 3.6.9  |

For information regarding the building of non standard packages refer to the [wiki](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki).
* [Build OpenCV](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki/Build-OpenCV)
* [Build Pyrealsense2](https://github.com/MarkSherstan/UAV-Sampling-Control-System/wiki/Build-Pyrealsense2)

## Use  
Run the bash script `run.sh` and once the UAV is placed into `GUIDED_NOGPS` mode the UAV will be in full control. Use `control + c` to terminate the program and complete data logging.

## Flight Logs 
Flight logs are saved after each flight in `.csv` format. To plot data and analyze the log, navigate to the `/flightData` directory and enter the file name in the command line as such: `python3 plotter.py --input "flight1.csv"`.

## Payloads
Open the `/payloads` directory and navigate to either the `/cap` or `/MASTER` directory and build using PlatformIO. All pinout is labeled in `src/main.cpp`. The servo library may need to be re-installed with the following command: `pio lib install "Servo"` in the PlatformIO command line. Upload the compiled code to its respective PCB. 

## PCB
The PCB's were designed with [KiCad 5](https://kicad-pcb.org/download/) and manufactured by [JLCPCB](https://jlcpcb.com).
