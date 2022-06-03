# Supervisor portion of the UAV-RT architecture 

This codebase acts as the middleware between the software running on Ground Control Station (GCS) and the nodes running on the Companion Computer. 

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

- Linux — GNU Compiler Collection (GCC) 6.3+

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

- For more details on creating a workspace using [Foxy](https://docs.ros.org/en/foxy/Tutorials/Workspace/Creating-A-Workspace.html) or [Galactic](https://docs.ros.org/en/galactic/Tutorials.html). 

### uavrt_ws

#### Linux

- Open up a terminal from within your Home directory. 
- Run the following commands: 
```
source /opt/ros/foxy/setup.bash
mkdir -p ~/uavrt_ws/src
cd ~/uavrt_ws/src
git clone https://github.com/ros/ros_tutorials.git -b foxy-devel
```

#### macOS

TBD

# License 

This codebase is released under the GNU Lesser General Public License v3 or later.
