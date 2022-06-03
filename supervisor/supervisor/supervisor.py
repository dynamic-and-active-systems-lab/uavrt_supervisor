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
        self.createHeartbeatPublisher(logger)

        # Telemetry monitor
        self.createTelemetryPublisher(logger)

        # /getPose service
        createGetPoseServicer(self)

    def createHeartbeatPublisher(self, logger):
        # Format: Msg type, topic, queue size
        self.heartbeatPublisher = self.create_publisher(
            DiagnosticArray,
            '/heartbeatStatus',
            QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISH_HEARTBEAT_STATUS_TIME_PERIOD,
            partial(heartbeatMonitor,
                    self,
                    self.heartbeatPublisher,
                    self.get_logger(),
                    self.connection,
                    self.heartbeatWatchdog))
        logger.info("Heartbeat monitor is now publishing.")

    def createTelemetryPublisher(self, logger):
        self.telemetryPublisher = self.create_publisher(
            PoseStamped,
            '/antennaPose',
            QUEUE_SIZE)
        self.timer = self.create_timer(
            PUBLISH_TELEMETRY_DATA_TIME_PERIOD,
            partial(telemetryMonitor,
                    self,
                    self.telemetryPublisher,
                    self.get_logger(),
                    self.connection,
                    self.currentTimeArray,
                    self.positionArray,
                    self.orientationArray))
        logger.info("Telemetry monitor is now publishing.")


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
