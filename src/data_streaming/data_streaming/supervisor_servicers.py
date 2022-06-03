#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
'''

from data_streaming.supervisor import *
from data_streaming.supervisor_publishers import *

# Using the custome GetPose service in order to fufill request from client.
# https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html
from custom_interfaces.srv import *

from functools import partial
from geometry_msgs.msg import *

# Used to find the Pose object that corresponds to the timestamp passed into
# the getPose service.
# https://docs.scipy.org/doc/scipy/tutorial/general.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
# Note: Requires installing scipy - pip3 install scipy
from scipy.interpolate import *


def createGetPoseServicer(supervisorNode):
    logger = supervisorNode.get_logger()

    supervisorNode.srv = supervisorNode.create_service(
        GetPose,
        '/getPose',
        partial(searchTelemetryArrays, supervisorNode))

    logger.info("/getPose service now running.")


def searchTelemetryArrays(supervisorNode, timestamp, pose):
    logger = supervisorNode.get_logger()

    logger.info("/getPose service has been requested.")

    position = Point()
    orientation = Quaternion()

    currentTime = supervisorNode.arrayCurrentTime
    positionX = supervisorNode.arrayPositionX
    positionY = supervisorNode.arrayPositionY
    positionZ = supervisorNode.arrayPositionZ

    quaternionX = supervisorNode.arrayQuaternionX
    quaternionY = supervisorNode.arrayQuaternionY
    quaternionZ = supervisorNode.arrayQuaternionZ
    quaternionW = supervisorNode.arrayQuaternionW

    print(currentTime.size)
    print(positionX.size)
    print(positionY.size)
    print(positionZ.size)
    print(quaternionX.size)
    print(quaternionY.size)
    print(quaternionZ.size)
    print(quaternionW.size)

    timestampSeconds = timestamp.sec + (timestamp.nanosec / NANOSECOND)

    funcPoseX = interp1d(currentTime,
                         positionX)
    funcPoseY = interp1d(currentTime,
                         positionY)
    funcPoseZ = interp1d(currentTime,
                         positionZ)

    funcQuatX = interp1d(currentTime,
                         quaternionX)
    funcQuatY = interp1d(currentTime,
                         quaternionY)
    funcQuatZ = interp1d(currentTime,
                         quaternionZ)
    funcQuatW = interp1d(currentTime,
                         quaternionW)

    position.x = funcPoseX(timestampSeconds)

    logger.info("/getPose service request has been fufilled.")

    return pose
