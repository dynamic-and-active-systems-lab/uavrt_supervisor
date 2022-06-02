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

def createGetPoseServicer(supervisorNode):
    logger = supervisorNode.get_logger()

    supervisorNode.srv = supervisorNode.create_service(
        GetPose,
        '/getPose',
        partial(searchTelemetryArrays, supervisorNode))

    logger.info("/getPose service now running.")


def searchTelemetryArrays(supervisorNode, timestamp, pose):
    logger = supervisorNode.get_logger()

    timestampSeconds = timestamp.seconds + (timestamp.nanoseconds / NANOSECOND)

    logger.info("/getPose service has been requested.")

    # TODO
    interpolatedFunction = interpolate.interp1d(supervisorNode.telemetryTime, )

    logger.info("/getPose service request has been fufilled.")

    return pose
