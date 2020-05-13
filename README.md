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

## Building OpenCV 4 on Ubuntu 18.04 
### Install Odroid C2 Operating System (OS)
Download the most up to date OS for the Odroid C2. It can be found [here](https://odroid.in/ubuntu_18.04lts/C2/). I selected `ubuntu-18.04.3-3.16-minimal-odroid-c2-20190814.img.xz`. The difference between minimal and mate is that the minimal version has no GUI and has slightly more usable ram. 

Use a 16 Gb or larger (reccomended 32 Gb) micro SD card capable of 98Mb/s or greater and flash the OS using [Balena Etcher](https://www.balena.io/etcher/). Once complete insert the SD card into the Odroid C2 and apply power. Once the Odroid has power and the OS has initialized a blue flashing light will begin. 

Once the system is ready, scan for the IP address of the Odroid using [Angry IP Scanner](https://angryip.org). On my MacOS the address is `root@192.168.2.3` and the default password is `odroid`.

### Prepare OpenCV Install
Run the following commands one at a time and read the outputs and act accordingly. 
```
$ sudo apt-get update
$ sudo apt-get upgrade

$ sudo apt-get install build-essential cmake unzip pkg-config

$ sudo apt-get install libjpeg-dev libpng-dev libtiff-dev
$ sudo apt-get install libavcodec-dev libavformat-dev libswscale-dev libv4l-dev
$ sudo apt-get install libxvidcore-dev libx264-dev

$ sudo apt-get install libgtk-3-dev

$ sudo apt-get install libatlas-base-dev gfortran

$ sudo apt-get install python3-dev
```

The latest release of OpenCV as of April 2020 is `4.3.0`. In the future this release can be changed, just update the numbers accordingly. 
```
$ cd ~
$ wget -O opencv.zip https://github.com/opencv/opencv/archive/4.3.0.zip
$ wget -O opencv_contrib.zip https://github.com/opencv/opencv_contrib/archive/4.3.0.zip

$ unzip opencv.zip
$ unzip opencv_contrib.zip

$ mv opencv-4.3.0 opencv
$ mv opencv_contrib-4.3.0 opencv_contrib
```

Install virtual environent requirments and source it. You can double check the commands worked by running `nano source ~/.bashrc` and checking the file contents. 
```
$ wget https://bootstrap.pypa.io/get-pip.py
$ sudo python3 get-pip.py

$ sudo pip install virtualenv virtualenvwrapper
$ sudo rm -rf ~/get-pip.py ~/.cache/pip

$ echo -e "\n# virtualenv and virtualenvwrapper" >> ~/.bashrc
$ echo "export WORKON_HOME=$HOME/.virtualenvs" >> ~/.bashrc
$ echo "export VIRTUALENVWRAPPER_PYTHON=/usr/bin/python3" >> ~/.bashrc
$ echo "source /usr/local/bin/virtualenvwrapper.sh" >> ~/.bashrc

$ source ~/.bashrc
```

Create a virutal environment and install numpy. 
```
$ mkvirtualenv cv -p python3

$ workon cv

$ pip install numpy
```

### Install OpenCV
Navigate the directories and prepare the cmake. 
```
$ cd ~/opencv
$ mkdir build
$ cd build

$ cmake -D CMAKE_BUILD_TYPE=RELEASE \
	-D CMAKE_INSTALL_PREFIX=/usr/local \
	-D INSTALL_PYTHON_EXAMPLES=ON \
	-D INSTALL_C_EXAMPLES=OFF \
	-D OPENCV_ENABLE_NONFREE=ON \
	-D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
	-D PYTHON_EXECUTABLE=~/.virtualenvs/cv/bin/python \
	-D BUILD_EXAMPLES=ON ..
```

The Odroid has a finite amount of ram and the install can hang and crash. To mititage this increase the swap storage, this should only be done temporarily as it can burn out the SD card. 

```
$ sudo apt-get install zram-config
````

Enter the command `sudo nano /usr/bin/init-zram-swapping` and change the line

```
mem=$(((totalmem / 2 / ${NRDEVICES}) * 1024))
```
To
```
mem=$(((totalmem / ${NRDEVICES}) * 1024))
```

OpenCV is now ready to be compiled. This make take forever so sit back and relax.

```
$ make -j4    
```

*Note: If the compile keeps failing try making with only one core: `make -j1`*

### Configure OpenCV
Once OpenCV has compiled to 100% it must be linked. Run the following commands and find the current python version installed in the virutal environment. 

```
$ sudo make install
$ sudo ldconfig

$ workon cv
$ python --version
$ deactivate
```

Make sure we are no longer in the virtual environment. The current version of python at the time of writing is `Python 3.6.9`. Using that information run the following (use tab completion to be sure):

```
$ cd /usr/local/lib/python3.6/site-packages/cv2/python-3.6
$ sudo mv cv2.cpython-36m-x86_64-linux-gnu.so cv2.so

$ cd ~/.virtualenvs/cv/lib/python3.6/site-packages
```

From there we need to create a sym-link to the compiled file. 

```
ln -s /usr/local/lib/python3.6/site-packages/cv2/python-3.6/cv2.so cv2.so
```

Make sure change back the swap memory `sudo nano /usr/bin/init-zram-swapping` and change the line:

```
mem=$(((totalmem / ${NRDEVICES}) * 1024))
```
Back to 
```
mem=$(((totalmem / 2 / ${NRDEVICES}) * 1024))
```

Reboot the device with `sudo reboot -h now` and everything should be configured properly. Enter the commands as seen below to verify.
```
$ workon cv
$ python
>>> import cv2
>>> cv2.__version__
'4.3.0'
```

If you run into any errors consider some of these resources: 
* https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/
* https://pysource.com/2019/08/26/install-opencv-4-1-on-nvidia-jetson-nano/
* https://www.pyimagesearch.com/2018/08/15/how-to-install-opencv-4-on-ubuntu/
* https://www.pyimagesearch.com/2019/09/16/install-opencv-4-on-raspberry-pi-4-and-raspbian-buster/

### Other
Install Tmux with `sudo apt install tmux`

Enter `tmux` to creat a session. 
Enter `control+b` then `d` to exit the session 
Enter `tmux attach` to rejoin a session 

To see memory and storage use `free -m` and `df -h` respectively. 

## To Do 
* pip3 install mavsdk
* Clean up documentation 
* pip3 install mavsdk