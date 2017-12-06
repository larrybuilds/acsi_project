# Advanced Control System Integration Class Project

Codebase for Advanced Control Systems Integration final project - Ball-in-a-cup on a quadrotor.

## Dependencies

This codebase was developed on Ubuntu 14.04. Much should be portable to 16.04, but nothing is built to do so.

Ubuntu 14.04 (Trusty Tahr)
ROS Indigo
Crazyflie 2.0
Crazyflie Client (programming firmware)
Optitrack System (motion capture system)

## Crazyflie Firmware Setup

### ARM Toolchain setup

#### For Ubuntu 14.04:

Download the GCC ARM toolchain needed to build the software, unpack and move into your home folder
```
wget https://launchpad.net/gcc-arm-embedded/4.7/4.7-2013-q1-update/+download/gcc-arm-none-eabi-4_7-2013q1-20130313-linux.tar.bz2
tar xjf gcc-arm-none-eabi-4_7-2013q1-20130313-linux.tar.bz2
mkdir ~/bin
mv gcc-arm-none-eabi-4_7-2013q1 ~/bin/gcc-arm-none-eabi
```
Add the location of the ARM toolchain to your PATH
```
echo -e "\nPATH=\$PATH:$HOME/bin/gcc-arm-none-eabi/bin" >> ~/.bashrc
source ~/.bashrc
```

#### For Ubuntu 16.04

```
sudo add-apt-repository ppa:team-gcc-arm-embedded/ppa
sudo apt-get update
sudo apt-get install libnewlib-arm-none-eabi
```

### Build the crazyflie firmware
```
cd crazyflie-firmware
make
```

## Optitrack System setup

### Connect to and configure Optitrack system

The optitrack system runs off a computer connected to the HDR-Network wifi. Connect your laptop to it using the following.
```
SSID: HDR-Netwwork
PASS: CMUHDR2015
```

Once connected, run ```ifconfig``` to get the IP address that was assigned to your computer. Open the MOTIVE software on the Optitrack computer and navigate to ``` View -> Data Streaming```. In the panel that comes up, configure the IP address the Optitrack system is streaming data to to your ip, ```Advanced Network Options -> Multicast Interface```.

### Add Crazyflie rigid body

Using 5 retro-reflective balls, mounted by the 4 rotors and 1 center, select them all in the MOTIVE software and right-click and select ```Rigid Body -> Create from Selected Markers```. This will create the body that the system tracks and reports position and attitude of to ROS.

### Reseting the Body-fixed Frame

Select the the rigid body and navigate to ```View -> Rigid Body Properties```. In the panel that comes up, go to the ```Orientation``` ribbon. Here, you can align the body fixed frame of the crazyflie so that frame is presented correctly in ROS. Algin the x-axis of the crazyflie pointing positive away from the lab door and click ```Reset to Current Orientation``` button to zero the roll,pitch,yaw

## Running Hover setpoint demo with Optitrack

Run
```
roslaunch launch/trajectory_tracker.launch
```
This will call the python script that will configure the onboard EKF and take in the optitrack data and command a hover setpoint 0.5m above the quad's current position.
