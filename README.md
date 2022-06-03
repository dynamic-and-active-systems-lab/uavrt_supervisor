# Supervisor portion of the UAV-RT architecture 

This codebase acts as the middleware between the software running on Ground Control Station (GCS) and the nodes running on the Companion Computer. 

# Features

- Establishes and monitors connection with the Pixhawk 4 or Gazebo SITL
- Recieves and publishes telemetry data from the Pixhawk 4 at a rate of 2Hz
    - Able to write this telemetry data to a .txt file for post-processing
- Provides a service request for determining a position in space given a timestamp
- TBD 

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
- Homebrew will be used for installation purposes on macOS

#### Python

Python 3.8 is the minimum requirement:

While a Python 3.9 enviroment is required for MATLAB ROS 2 support: 

#### CMake

Cmake 3.16.3+: 

Ubuntu

```
sudo apt update && sudo apt upgrade
sudo apt install cmake
cmake --version
```

macOS

#### C++ Compilers

Linux — GNU Compiler Collection (GCC) 6.3+

```
sudo apt update && sudo apt upgrade
sudo apt install build-essential
sudo apt-get install manpages-dev
gcc --version
```

macOS — Xcode 10+

#### ROS 2

This codebase supports the Foxy Fitzroy and Galactic Geochelone distributions of ROS 2: 

- [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html)
- [ROS 2 Galactic Geochelone](https://docs.ros.org/en/foxy/Releases/Release-Galactic-Geochelone.html)

#### MAVLink and Pymavlink

This codebase supports [MAVLink V2](https://mavlink.io/en/guide/mavlink_2.html). 

We use Pymavlink to use the MAVLink protocol. 

- [Pymavlink](https://github.com/ArduPilot/pymavlink)

#### MATLAB

MATLAB 2022a+ is recommended but it is not required: 

- [MATLAB R2022a](https://www.mathworks.com/support/requirements/matlab-system-requirements.html)

# Installaton

### Linux

# License 
