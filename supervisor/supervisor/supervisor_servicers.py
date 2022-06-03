#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials/Actions/Writing-a-Py-Action-Server-Client.html
https://docs.ros.org/en/rolling/Concepts/About-ROS-Interfaces.html
'''

# Used to find the Pose object that corresponds to the timestamp passed into
# the getPose service.
# https://docs.scipy.org/doc/scipy/tutorial/general.html
# https://docs.scipy.org/doc/scipy/reference/generated/scipy.interpolate.interp1d.html
# Note: Requires installing scipy - pip3 install scipy
from scipy.interpolate import *

# DELETE
from builtin_interfaces.msg import *
from geometry_msgs.msg import *

# One billionth (10^-9) of a second
# For converting nanoseconds
# https://stackoverflow.com/a/15650033
NANOSECOND = 1000000000


def searchTelemetryArrays(supervisorNode, request, response):

    timestamp = Time()
    pose = Pose()
    position = Point()
    orientation = Quaternion()

    currentTimeArray = supervisorNode.currentTimeArray
    positionArray = supervisorNode.positionArray
    orientationArray = supervisorNode.orientationArray

    timestamp = request.timestamp.sec + (request.timestamp.nanosec / NANOSECOND)

    positonFunction = interp1d(supervisorNode.currentTimeArray,
                               supervisorNode.positionArray)

    orientationFunction = interp1d(supervisorNode.currentTimeArray,
                                   supervisorNode.orientationArray)

    interpolatedPositions = positonFunction(timestamp)

    position.x = interpolatedPositions[0]
    position.y = interpolatedPositions[1]
    position.z = interpolatedPositions[2]

    interpolatedOrientations = orientationFunction(timestamp)

    orientation.x = interpolatedOrientations[0]
    orientation.y = interpolatedOrientations[1]
    orientation.z = interpolatedOrientations[2]
    orientation.w = interpolatedOrientations[3]

    response.pose.position = position
    response.pose.orientation = orientation

    supervisorNode.get_logger().info("/getPose service request has been fufilled.")

    return response
