# acsi_project
Advanced Control System Integration class project repo

# Crazyflie firmware enviornment setup

## 1 ARM Toolchain setup

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

## 2 Download crazyflie firmware and initialize

```
git clone --recursive https://github.com/bitcraze/crazyflie-firmware.git
cd crazyflie-firmware
git submodule init
git submodule update
```

## 3 Build the crazyflie firmware
```
make
```
  # Setting up optitrack on Ubuntu 14.04

  ## 1 Install Optitrack ROS package
  Follow instructions on the following GIT repository:
  https://github.com/crigroup/optitrack
  
  
