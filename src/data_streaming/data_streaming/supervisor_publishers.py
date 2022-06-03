#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
'''

from data_streaming.mavlink_helper import *
from data_streaming.supervisor_bags import *
from data_streaming.supervisor_servicers import *

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
# Standard messages for ROS2.
# Necessary for Header type messages.
# https://github.com/ros2/common_interfaces/tree/master/std_msgs/msg
from std_msgs.msg import *
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

# I made 3 files: supervisor_publishers, supervisor_subscribers,
# and supervisor_servicers.
# These files house ROS2 runctionality of the Supervisor Node.
# It's not advertised on ROS2's site/wiki that you can split the
# publisher/subscriber callbacks from the node, but it's not taboo either.
# I feel it makes the codebase much easier to follow when the functions are in
# separate files.
#
# In order to do this, you need to either use partial() or lambda for the
# callbacks.
# https://discourse.ros.org/t/callback-args-in-ros2/4727/2
# https://docs.python.org/3/library/functools.html
from functools import partial

# For array management; storing header information from PoseStamped objects
# as well as the Pose portion of the objects.
# https://numpy.org/doc/stable/user/quickstart.html
# Note: Requires installing scipy - pip3 install numpy
import numpy as np



# Queue size is a required QoS (quality of service) setting that limits the
# amount of queued messages if a subscriber is not receiving them fast enough.
QUEUE_SIZE = 10

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


def createHeatbeatPublisher(supervisorNode):
    logger = supervisorNode.get_logger()

    # Format: Msg type, topic, queue size
    supervisorNode.heartbeatPublisher = supervisorNode.create_publisher(
        DiagnosticArray,
        '/heartbeatStatus',
        QUEUE_SIZE)
    supervisorNode.timer = supervisorNode.create_timer(
        PUBLISH_HEARTBEAT_STATUS_TIME_PERIOD,
        partial(heartbeatMonitor, supervisorNode))
    logger.info("Heartbeat monitor is now publishing.")


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

    # Note: This could be made into a subscriber to reinforce abstraction but
    # it seems like overkill.
    if status.message == 'Dead':
        logger.info("Attempting to reestablish connection.")
        supervisorNode.connection = establishMavlinkConnection(logger)


def createTelemetryPublisher(supervisorNode):
    logger = supervisorNode.get_logger()

    # Format: Msg type, topic, queue size
    supervisorNode.telemetryDataPublisher = supervisorNode.create_publisher(
        PoseStamped,
        '/antennaPose',
        QUEUE_SIZE)
    supervisorNode.timer = supervisorNode.create_timer(
        PUBLISH_TELEMETRY_DATA_TIME_PERIOD,
        partial(telemetryMonitor, supervisorNode))
    logger.info("Telemetry monitor is now publishing.")


def telemetryMonitor(supervisorNode):
    connection = supervisorNode.connection
    logger = supervisorNode.get_logger()

    if connection != None and checkGPS(connection, logger) != None:
        header = Header()
        poseStamped = PoseStamped()
        pose = Pose()
        position = Point()
        orientation = Quaternion()

        # We get the current time seperately since the value that is stored
        # into the Node's time array needs to be in nanoseconds for
        # interpolation
        currentTime = supervisorNode.get_clock().now()

        header.frame_id = "telemetryData"
        header.stamp = currentTime.to_msg()

        position.x = float(getLongitude(connection, logger))
        position.y = float(getLatitude(connection, logger))
        position.z = float(getAltitude(connection, logger))

        orientation.x = float(getLongitude(connection, logger))
        orientation.y = float(getLatitude(connection, logger))
        orientation.z = float(getAltitude(connection, logger))
        orientation.w = float(getAltitude(connection, logger))

        pose.position = position
        pose.orientation = orientation

        poseStamped.header = header
        poseStamped.pose = pose

        supervisorNode.telemetryDataPublisher.publish(poseStamped)

        logger.info("Telemetry data has been successfully published.")

        # TODO: This shouldn't go here. I'm using this place for testing
        # purposes. It doesn't have a home atm.
        # recordTelemetryData(supervisorNode, poseStamped)

        supervisorNode.arrayCurrentTime = np.append(
            supervisorNode.arrayCurrentTime, currentTime.nanoseconds)

        supervisorNode.arrayPositionX = np.append(
            supervisorNode.arrayPositionX, position.x)
        supervisorNode.arrayPositionY = np.append(
            supervisorNode.arrayPositionY, position.y)
        supervisorNode.arrayPositionZ = np.append(
            supervisorNode.arrayPositionZ, position.z)

        supervisorNode.arrayQuaternionX = np.append(
            supervisorNode.arrayQuaternionX, orientation.x)
        supervisorNode.arrayQuaternionY = np.append(
            supervisorNode.arrayQuaternionX, orientation.y)
        supervisorNode.arrayQuaternionZ = np.append(
            supervisorNode.arrayQuaternionX, orientation.z)
        supervisorNode.arrayQuaternionW = np.append(
            supervisorNode.arrayQuaternionX, orientation.w)

        if (supervisorNode.arrayCurrentTime.size) > 3:
            currentTime = supervisorNode.get_clock().now().to_msg()

            pose = Pose()

            pose = searchTelemetryArrays(supervisorNode, currentTime, pose)
    else:
        logger.warn("Unable to publish telemetry data!")
        logger.warn("Connection or GPS lock was not established!")
