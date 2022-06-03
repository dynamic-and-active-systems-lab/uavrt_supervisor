# Supervisor portion of the UAV-RT architecture 

This codebase acts as the middleware between the software running on the Ground Control Station (GCS) and the nodes/processes running on the Companion Computer. 

The development of this code was funded via National Science Foundation grant no. 2104570.

# Features

- Establishes and monitors a serial or UDP connection with the Pixhawk 4 or Gazebo SITL, respectively
- Recieves and publishes telemetry data from the Pixhawk 4 at a rate of 2Hz
  - Able to write this telemetry data to a .txt file for post-processing
- Provides a service request for determining the position of the UAV in space given a timestamp
- TBD 

# Basic operation

TBD 

# Documentation

The supporting documentation for this project can be found in the following directory: 

TBD

As well as on the following site: 

TBD 

# System requirements

### Platforms

- Windows is not supported at this time. 
- Linux - Ubuntu 20.04 (amd64 and arm64?) 
- macOS - Mojave (10.14) 

### Hardware

- Pixhawk 4
- Airspy HF+ Discovery 
- Computer computer
  - This project was developed and deployed on the [UDOO x86 II Ultra](https://shop.udoo.org/en/udoo-x86-ii-ultra.html)

### Dependencies 

Installation instructions will be supplied for Ubuntu 20.04 and macOS. 

- The standard Debian/Ubuntu package manager `apt` will be used for installation purposes on Linux
- [Homebrew](https://brew.sh/) will be used for installation purposes on macOS

**Note:** Check whether these dependencies are installed prior to running the installation commands below!

#### Python

Python 3.8 is the minimum requirement:

- Ubuntu 

```
sudo apt update && sudo apt upgrade
sudo apt install python3.9
python --version
```

- macOS

```
brew update
brew upgrade
brew install python@3.8
python --version
```

While a Python 3.9 enviroment is required for MATLAB ROS 2 support: 

- Ubuntu 

```
sudo apt update && sudo apt upgrade
sudo apt install python3.9
python --version
```
- macOS

```
brew update
brew upgrade
brew install python@3.9
python --version
```

#### CMake

Cmake 3.16.3+: 

- Ubuntu

```
sudo apt update && sudo apt upgrade 
sudo apt install cmake
cmake --version
```

- macOS

```
brew update
brew upgrade
brew install cmake
cmake --version
```

#### C++ Compilers

- Ubuntu — GNU Compiler Collection (GCC) 6.3+

```
sudo apt update && sudo apt upgrade
sudo apt install build-essential
sudo apt-get install manpages-dev
gcc --version
```

- macOS — Xcode 10+

To install Command Line Utilities only: 

```
xcode-select --install
gcc --version
```

#### Netcat 

Ubuntu 

```
sudo apt update && sudo apt upgrade
apt-get install netcat
dpkg -L netcat
```

macOS

```
brew update
brew upgrade
brew install netcat
netcat -h
```

#### airspyhf_rx

TBD. The instructions I wrote for this are in the [airspy_channelizer repo](https://github.com/dynamic-and-active-systems-lab/airspyhf_channelize#installing-airspyhf_rx). Do I point to that, copy or move them over here? I'd like to put them all in one place so I could edit them. Ultimately, the instructions for everything should be on the DASL site. 

#### ROS 2

This codebase supports the Foxy Fitzroy and Galactic Geochelone distributions of ROS 2: 

- [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html)
- [ROS 2 Galactic Geochelone](https://docs.ros.org/en/galactic/Releases/Release-Galactic-Geochelone.html)

#### MAVLink and Pymavlink

This codebase supports [MAVLink V2](https://mavlink.io/en/guide/mavlink_2.html). 

We use Pymavlink as an interface to the MAVLink protocol: 

- [Pymavlink](https://github.com/ArduPilot/pymavlink)

#### MATLAB

MATLAB 2022a+ is recommended but it is not required: 

- [MATLAB R2022a](https://www.mathworks.com/support/requirements/matlab-system-requirements.html)

# Installaton

For installing this package, it is required that you have a functional ROS 2 workspace. Below is a set of instructions to create a ROS2 workspace. These instructions will fail unless the previous dependencies have been met. 

- These instructions were adapted from [ROS 2 documentation](https://docs.ros.org/). 
- For more details on creating a workspace using [Foxy](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) or [Galactic](https://docs.ros.org/en/galactic/Tutorials.html). 

### uavrt_ws

**Note:** I will be supplying the instructions necessary to build a workspace within a ROS 2 Galactic enviroment. 

I need to 100% confirm that the final version of the codebase can run on Foxy. Else galactic will be the required ROS 2 version. 

#### Linux

Within a terminal window, run the following commands: 

```
source /opt/ros/galactic/setup.bash
# 'source ~/ros2_galactic/ros2-linux/setup.bash' run this command if the one above doesn't work
mkdir -p ~/uavrt_ws/
cd ~/uavrt_ws/
# Authentication is required for the following command
# Must be a member of the Dynamic and Active Systems Lab organization on Github
git clone https://github.com/dynamic-and-active-systems-lab/uavrt_supervise/
rosdep install -i --from-path uavrt_supervise --rosdistro galactic -y
# Should return "All required rosdeps installed successfully"
colcon build
# "build  install  log  uavrt_supervise" directories should exist in the workspace root (~/uavrt_ws) 
source /opt/ros/galactic/setup.bash
# 'source ~/ros2_galactic/ros2-linux/setup.bash' run this command if the one above doesn't work
. install/local_setup.bash
```

If these commands didn't fail, then should you be able to run the `supervisor` package with the following command: 

```
ros2 run supervisor supervisor_node
```

#### macOS

TBD. Instructions are listed in the ROS 2 documentation but I have not gone through them. I'm leaving this section blank until I go through this installation process on a Mac running Mohave 10.4. 

# License 

This codebase is released under the GNU Lesser General Public License v3 or later.
