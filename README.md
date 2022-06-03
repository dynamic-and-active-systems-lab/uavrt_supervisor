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

- Python

Python 3.8 is the minimum requirement, while a Python 3.9 enviroment is required for MATLAB ROS 2 support. 

- CMake

Cmake 3.16.3 is the minimum requirement. 

```
sudo apt update
sudo apt install cmake
```

C++ Compilers

- Linux — GNU Compiler Collection (GCC) 6.3+

```
sudo apt update
sudo apt install build-essential
sudo apt-get install manpages-dev
gcc --version
```

- macOS — Xcode 10+

This codebase supports the Foxy Fitzroy and Galactic Geochelone distributions of ROS 2: 

- [ROS 2 Foxy Fitzroy](https://docs.ros.org/en/foxy/Releases/Release-Foxy-Fitzroy.html)
- [ROS 2 Galactic Geochelone](https://docs.ros.org/en/foxy/Releases/Release-Galactic-Geochelone.html)

MAVLink messaging: 

- [Pymavlink](https://github.com/ArduPilot/pymavlink)

The following software is recommended but it is not required: 

- [MATLAB R2022a](https://www.mathworks.com/support/requirements/matlab-system-requirements.html)

# Installaton

### Linux

# License 
