#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
'''

# Used to find the Pose object that corresponds to the timestamp passed into
# the getPose service.
# https://docs.scipy.org/doc/scipy/tutorial/general.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
# Note: Requires installing scipy - pip3 install scipy
from scipy.interpolate import *


def searchTelemetryArrays(supervisorNode, timestamp, pose):
    position = Point()
    orientation = Quaternion()

    currentTimeArray = supervisorNode.currentTimeArray
    positionArray = supervisorNode.positionArray
    orientationArray = supervisorNode.orientationArray

    timestampSeconds = timestamp.sec + (timestamp.nanosec / NANOSECOND)

    positonFunction = interp1d(supervisorNode.currentTimeArray,
                               supervisorNode.positionArray)

    orientationFunction = interp1d(supervisorNode.currentTimeArray,
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

    supervisorNode.get_logger().info("/getPose service request has been fufilled.")

    return pose
