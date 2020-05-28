# Instructions for starting a Gazebo Software in the Loop (SITL) Simulation
Note this is just a rough guide. Lots of errors had to be followed and proper packages installed to get a working system. That being said some steps could be obmitted in the future. Read the outputs of everything otherwise!


## Linux Ubuntu Install
This was all tested with Ubuntu 20.04 LTS. I dual booted my desktop running Windows 10, however the process for MacOS is basically the same. 
1. Download [Ubuntu desktop](https://ubuntu.com/download).
2. Burn the image to a USB drive using [balenaEtcher](https://www.balena.io/etcher/).
3. Partion drive on main computer for space to be allocated to Ubuntu. 
4. Restart computer with the USB as the boot drive (hold `shift` on Windows while restarting or `option` on Mac to make the selection. May need to change BIOS boot drive if the first option doesent work).
5. Try Ubuntu to make sure everything is working as expected (run a YouTube video). Once satisified install Ubuntu from the shortcut on the desktop. 
6. Select standard keyboard layout(s).
7. Perform a `normal installation`. I selected to download updates and install third-party software. 
8. For the installation type select `something else`.
9. Find the previously partioned space and select it as the install location. The mount point for the installation should be `\`. Ext4journaling file system is probably the deafult and will be fine. 
10. Install and follow the instructions. 

There are a few other steps such as connecting to Wifi and creating an account. More detailed instructions can be found [here](https://itsfoss.com/install-ubuntu-1404-dual-boot-mode-windows-8-81-uefi/), [here](https://www.hellotech.com/guide/for/how-to-install-linux-on-windows-10), and [here](https://www.hellotech.com/guide/for/how-to-install-linux-on-mac).

I had an error where I could not login even though my password was correct. It was an infitite loop (there is an error that pops up if the password is incorrect). This was fixed as per [StackExchange](https://askubuntu.com/a/1231515).


## Initial Set Up 
Install some of the basics:

### Google Chrome 
```
$ cd
$ wget https://dl.google.com/linux/direct/google-chrome-stable_current_amd64.deb
$ sudo apt install ./google-chrome-stable_current_amd64.deb
```

### Visual Studio Code
Download the `.deb` from [here](https://code.visualstudio.com/Download). Just like Google Chrome navigate to the file and install it with `$ sudo apt install ./<file>.deb`

### Pip3
Python3 (version 3.8 ish) should come with the install but pip3 will need to be installed. Run `$ sudo apt install python3-pip`.

### Git
```
$ sudo apt install git
$ git config --global user.name "John Doe"
$ git config --global user.email johndoe@example.com
```

### Clean up
Double check everything is up to date
```
$ sudo apt-get update
$ sudo apt-get upgrade
$ sudo apt-get autoremove
```


## Gazebo and PX4
There is decent documentation for how to set up a correct development environment and there are many helpful bash scripts. The following just summarizes the ones that I used. I would reccomend reading through the full documentation starting [here](https://dev.px4.io/v1.9.0/en/setup/getting_started.html).

Run `$ sudo usermod -a -G dialout $USER` and then restart your system. 

Pixhawk/NuttX (and jMAVSim) install (acknowledge prompts as required and restart the system again upon completion):
```
$ cd
$ wget https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim_nuttx.sh
$ source ubuntu_sim_nuttx.sh
```

Gazebo Simulation (the script installs Gazebo9 as this is what is supported. Do **NOT** install anything e.g. - Gazebo11 on your own):
```
$ cd 
$ wget https://raw.githubusercontent.com/PX4/Devguide/v1.9.0/build_scripts/ubuntu_sim.sh
$ source ubuntu_sim.sh
```

You should be in the correct directory already. Double check with `$ cd ~/src/Firmware`. Now run `make px4_sitl gazebo` and we should get a simulation working. This is most likely where all the errors will show so install the packages as required and read carefully. 

A specific firmware can be tested, to see all the releases enter `git tag -l` and then switch the branch, e.g. `git checkout v1.7.4beta`. 

## Software in the Loop
TBD
