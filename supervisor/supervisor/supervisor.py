#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials.html
'''

# When importing Python files that reside in the same ws/src/package, you will
# need to specify the name of the package as well as the file.
#
# This Stackoverflow question/answer helped me figured out this issue:
# https://stackoverflow.com/a/58504978
from supervisor.supervisor_publishers import *
from supervisor.supervisor_subscribers import *
from supervisor.supervisor_servicers import *
from supervisor.supervisor_bags import *

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


# TODO: Create launch logging configuration file
# UNIQUE_LOG_BASE_PATH = '~/uavrt_ws/src/supervisor/log'
# TODO: Need to add unit testing eventually.
# TODO: Not sure how make it so import/from statments are all in one file/
# organized/centralized - email Don
# TODO: Organize everything into a callback_group to ensure the proper order
# of callbacks and log messages.

import time


class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisorNode')

        logger = self.get_logger()

        logger.set_level(LoggingSeverity.INFO)
        logger.info("Supervisor node has been created.")
        logger.info("Logging severity has been set to info.")

        self.heartbeatWatchdog = 0
        self.connection = None

        self.telemetryDirectoryName = None
        self.telemetryFileName = None

        self.arrayCurrentTime = np.zeros(0, dtype=int)

        self.arrayPositionX = np.zeros(0, dtype=np.float64)
        self.arrayPositionY = np.zeros(0, dtype=np.float64)
        self.arrayPositionZ = np.zeros(0, dtype=np.float64)

        self.arrayQuaternionX = np.zeros(0, dtype=np.float64)
        self.arrayQuaterniony = np.zeros(0, dtype=np.float64)
        self.arrayQuaternionZ = np.zeros(0, dtype=np.float64)
        self.arrayQuaternionW = np.zeros(0, dtype=np.float64)

        # Heartbeat status monitor
        createHeatbeatPublisher(self)

        # Telemetry monitor
        createTelemetryPublisher(self)

        # Telemetry bag writer
        # createTelemetryBagWriter(self)

        # Telemetry bag reader
        # createTelemetryBagReader(self)

        # /getPose service
        createGetPoseServicer(self)


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
