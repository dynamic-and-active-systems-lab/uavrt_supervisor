#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials.html
'''

# When importing Python files that reside in the same workspace/src/package,
# you will need to specify the name of the package as well as the file.
#
# This Stackoverflow question/answer helped me figured out this issue:
# https://stackoverflow.com/a/58504978
from supervisor.supervisor_publishers import *
from supervisor.supervisor_subscribers import *
from supervisor.supervisor_servicers import *

# Using the custome GetPose service in order to fufill request from client.
# https://docs.ros.org/en/humble/Tutorials/Custom-ROS2-Interfaces.html
from custom_interfaces.srv import *

# Import rclpy so its classes can be used.
# https://docs.ros2.org/latest/api/rclpy/index.html
# ROS2 testing library that provides insight into the functionlity of
# the different ROS2 classes.
# https://github.com/ros2/rclpy/tree/master/rclpy/test
import rclpy

# Import Node class so that we can create Nodes.
from rclpy.node import Node

# Logging functionality currently supported in ROS 2.
# Log messages can be formatted with native Python .format functioanlity!
# https://docs.ros.org/en/humble/Concepts/About-Logging.html
# https://docs.ros.org/en/humble/Tutorials/Logging-and-logger-configuration.html
# https://github.com/ros2/rclpy/blob/master/rclpy/test/test_logging.py
# https://github.com/ros2/launch/blob/master/launch/launch/logging/__init__.py
#
# Information on Python logging and a logging cookbook:
# https://docs.python.org/3/library/logging.html
# https://docs.python.org/3/howto/logging-cookbook.html#logging-cookbook
#
# We did not go with Python approach to logging as ROS supplies this
# functionality natively. This allows to more fully utilize the ROS framework.
# However, these links explore the functioality behidn ROS logger.
from rclpy.logging import LoggingSeverity

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

# Import Parameter class so we can create Paramaters.
# Not using this functioanlity at the moment since it doesn't seem to add much
# utility to the uavrt_ws.
# https://docs.ros.org/en/humble/Tutorials/Using-Parameters-In-A-Class-Python.html
# https://docs.ros2.org/latest/api/rclpy/api/parameters.html
# https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
# from rclpy.parameter import Parameter

# For array management; storing header information from PoseStamped objects
# as well as the Pose portion of the objects.
# https://numpy.org/doc/stable/user/quickstart.html
# Note: Requires installing scipy - pip3 install numpy
import numpy as np

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

# Python doesn’t have built-in constant types.
# By convention, Python uses a variable whose name contains all capital letters
# to indicate that a variable is a constant.
#
# Queue size is a required QoS (quality of service) setting that limits the
# amount of queued messages if a subscriber is not receiving them fast enough.
QUEUE_SIZE = 10
# Rate at which heartbeat status messages will be checked and published.
# Seconds
PUBLISH_HEARTBEAT_STATUS_TIME_PERIOD = 1
# Rate at which telemetry data will be published and written to memory.
PUBLISH_TELEMETRY_DATA_TIME_PERIOD = .5

# TODO: Create launch logging configuration file
# UNIQUE_LOG_BASE_PATH = '~/uavrt_ws/src/supervisor/log'
# TODO: Need to add unit testing eventually.
# TODO: Not sure how make it so import/from statments are all in one file/
# organized/centralized - email Don
# TODO: Organize everything into a callback_group to ensure the proper order
# of callbacks and log messages.
# TODO: Figure out serial reset
# "command that would reset the serial port if I unplugged and then replugged in usb"


class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisorNode')

        logger = self.get_logger()

        logger.info("Supervisor node has been created.")

        logger.set_level(LoggingSeverity.INFO)
        logger.info("Logging severity has been set to info.")

        self.connection = None
        self.heartbeatWatchdog = 0

        # Make a 1 by 1 array filled with 0's of type int
        # [[0.]]
        self.currentTimeArray = np.zeros((1, 1), dtype=int)

        # Make a 3 by 1 array filled with 0's of type float64
        # [[0.]
        #  [0.]
        #  [0.]]
        self.positionArray = np.zeros((3, 1), dtype=np.float64)

        # Make a 4 by 1 array filled with 0's of type float64
        # [[0.]
        #  [0.]
        #  [0.]
        #  [0.]]
        self.orientationArray = np.zeros((4, 1), dtype=np.float64)

        # Heartbeat status monitor
        self.createHeartbeatPublisher()

        # Telemetry monitor
        self.createTelemetryPublisher()

        # getPose service
        self.createGetPoseServicer()

    def createHeartbeatPublisher(self):
        # Format: Msg type, topic, queue size
        self.heartbeatPublisher = self.create_publisher(
            DiagnosticArray,
            '/heartbeatStatus',
            QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISH_HEARTBEAT_STATUS_TIME_PERIOD,
            partial(heartbeatMonitor, self))
        self.get_logger().info("Heartbeat monitor is now publishing.")

    def createTelemetryPublisher(self):
        self.telemetryPublisher = self.create_publisher(
            PoseStamped,
            '/antennaPose',
            QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISH_TELEMETRY_DATA_TIME_PERIOD,
            partial(telemetryMonitor, self))
        self.get_logger().info("Telemetry monitor is now publishing.")

    def createGetPoseServicer(self):
        self.srv = self.create_service(
            GetPose,
            '/getPose',
            partial(searchTelemetryArrays, self))
        self.get_logger().info("/getPose service now running.")


def main(args=None):
    rclpy.init(args=args)

    supervisorNode = Supervisor()
    # logger is a rclpy.impl.rcutils_logger.RcutilsLogger object
    logger = supervisorNode.get_logger()

    try:
        rclpy.spin(supervisorNode)
        logger.info("Supervisor node has been started.")
    except KeyboardInterrupt as instance:
        logger.info("Keyboard interrupt recieved.")
        logger.info("Type: {}".format(type(instance)))
    finally:
        logger.info("Supervisor node shutting down.")
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        supervisorNode.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
