#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
'''

from supervisor.supervisor import *
from supervisor.supervisor_publishers import *

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

    position = Point()
    orientation = Quaternion()

    arrayCurrentTime = supervisorNode.arrayCurrentTime
    positionArray = supervisorNode.positionArray
    orientationArray = supervisorNode.orientationArray

    timestampSeconds = timestamp.sec + (timestamp.nanosec / NANOSECOND)

    positonFunction = interp1d(supervisorNode.arrayCurrentTime,
                               supervisorNode.positionArray)

    orientationFunction = interp1d(supervisorNode.arrayCurrentTime,
                                   supervisorNode.orientationArray)

    interpolatedPositions = positonFunction(timestampSeconds)

    position.x = interpolatedPositions[0]
    position.y = interpolatedPositions[1]
    position.z = interpolatedPositions[2]

    interpolatedOrientations = orientationFunction(timestampSeconds)

    orientation.x = interpolatedOrientations[0]
    orientation.y = interpolatedOrientations[1]
    orientation.z = interpolatedOrientations[2]
    orientation.w = interpolatedOrientations[3]

    pose.position = position
    pose.orientation = orientation

    logger.info("/getPose service request has been fufilled.")

    return pose
