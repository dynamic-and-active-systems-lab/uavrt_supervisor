#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
'''

from data_streaming.mavlink_helper import *

# Import Parameter class so we can create Paramaters.
# Not using this functioanlity at the moment since it doesn't seem to add much
# utility to the uavrt_ws.
# https://docs.ros.org/en/humble/Tutorials/Using-Parameters-In-A-Class-Python.html
# https://docs.ros2.org/latest/api/rclpy/api/parameters.html
# https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
# from rclpy.parameter import Parameter

# A set of packages which contain common interface files (.msg and .srv) for ROS2:
# https://github.com/ros2/common_interfaces/tree/master
#
# Import the built-in Geometry message type that the node uses to structure
# the telemetry data that it publishes on the /atennapPose topic.
# Allows for /getPose service call as well.
# https://github.com/ros2/common_interfaces/tree/master/geometry_msgs/msg
from geometry_msgs.msg import *
# Import the built-in Diagnostic messages that the node uses to timestamp and
# structure heartbeat status messages that it publishes on the /heartbeatStatus
# topic.
# https://github.com/ros2/common_interfaces/tree/master/diagnostic_msgs
# http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer#Generating_Diagnostics_Input
from diagnostic_msgs.msg import *

# Heartbeat unique string for heartbeatStatus messages.
# Hex(heartbeat)
HEARTBEAT_ID = '776f7264'
# Rate at which heartbeat status messages will be checked and published.
# Seconds
PUBLISH_HEARTBEAT_STATUS_TIME_PERIOD = 1
# Allow number of missed heartbeats
MISSED_HEARTBEAT_LIMIT = 5

# Rate at which telemetry data will be published and written to memory.
PUBLISH_TELEMETRY_DATA_TIME_PERIOD = .5


def heartbeatMonitor(supervisorNode):
    connection = supervisorNode.connection
    logger = supervisorNode.get_logger()
    heartbeatWatchdog = supervisorNode.heartbeatWatchdog

    statusArray = DiagnosticArray()
    status = DiagnosticStatus()
    value = KeyValue()

    currentStatus = checkHeartbeat(connection, logger)

    statusArray.header.frame_id = "heartbeatStatus"
    statusArray.header.stamp = supervisorNode.get_clock().now().to_msg()

    status.name = 'Heartbeat'
    status.hardware_id = HEARTBEAT_ID

    value.key = 'Number of missed heartbeats'

    if currentStatus != None:
        # Call by reference and call by value issue here
        # Updating the Node's heartbeatWatchdog and not simply the value
        # in this function
        supervisorNode.heartbeatWatchdog = MISSED_HEARTBEAT_LIMIT
        status.level = b'0'
        status.message = 'Alive'
    elif currentStatus == None and heartbeatWatchdog > 0:
        supervisorNode.heartbeatWatchdog -= 1
        status.level = b'1'
        status.message = 'Unknown'
    elif currentStatus == None and heartbeatWatchdog == 0:
        status.level = b'2'
        status.message = 'Dead'

    value.value = ''.format(MISSED_HEARTBEAT_LIMIT - heartbeatWatchdog)

    logger.info("Current heartbeat status: {}".format(status.message))

    status.values.append(value)
    statusArray.status.append(status)

    supervisorNode.heartbeatPublisher.publish(statusArray)

    if status.message == 'Dead':
        logger.info("Attempting to reestablish connection.")
        supervisorNode.connection = establishMavlinkConnection(logger)


def telemetryMonitor(supervisorNode):
    connection = supervisorNode.connection
    logger = supervisorNode.get_logger()

    poseStamped = PoseStamped()
    pose = Pose()
    position = Point()
    orientation = Quaternion()

    if connection != None and checkGPS(connection, logger) != None:
        poseStamped.header.frame_id = "telemetryData"
        poseStamped.header.stamp = supervisorNode.get_clock().now().to_msg()

        position.x = float(getLongitude(connection, logger))
        position.y = float(getLatitude(connection, logger))
        position.z = float(getAltitude(connection, logger))

        orientation.x = float(getLongitude(connection, logger))
        orientation.y = float(getLatitude(connection, logger))
        orientation.z = float(getAltitude(connection, logger))
        orientation.w = float(getAltitude(connection, logger))

        pose.position = position
        pose.orientation = orientation

        poseStamped.pose = pose

        supervisorNode.telemetryDataPublisher.publish(poseStamped)

        logger.info("Telemetry data has been successfully published.")
    else:
        logger.warn("Unable to publish telemetry data!")
        logger.warn("Connection or GPS lock was not established!")
