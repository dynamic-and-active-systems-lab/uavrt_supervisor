#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials.html
'''

# ROS2 testing library that provides insight into the functionlity of
# the different ROS2 classes.
# https://github.com/ros2/rclpy/tree/master/rclpy/test

# When importing Python files that reside in the same ws/src/package, you will
# need to specify the name of the package as well as the file.
#
# This Stackoverflow question/answer helped me figured out this issue:
# https://stackoverflow.com/a/58504978
from data_streaming.mavlink_helper import *

# Import rclpy so its classes can be used.
import rclpy

# Import Node class so that we can create Nodes.
from rclpy.node import Node

# Import Parameter class so we can create Paramaters.
# https://docs.ros.org/en/humble/Tutorials/Using-Parameters-In-A-Class-Python.html
# https://docs.ros2.org/latest/api/rclpy/api/parameters.html
# https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
from rclpy.parameter import Parameter

# Logging functionality currently supported in ROS 2.
# Log messages can be formatted with native Python .format functioanlity!
# https://docs.ros.org/en/humble/Concepts/About-Logging.html
# https://docs.ros.org/en/humble/Tutorials/Logging-and-logger-configuration.html
# https://github.com/ros2/rclpy/blob/master/rclpy/test/test_logging.py
# https://github.com/ros2/launch/blob/master/launch/launch/logging/__init__.py
from rclpy.logging import LoggingSeverity

# A set of packages which contain common interface files (.msg and .srv) for ROS2:
# https://github.com/ros2/common_interfaces/tree/master
#
# Import the built-in PoseStamped message type that the node uses to structure
# the telemetry data that it passes on to the /atennapPose topic.
from geometry_msgs.msg import PoseStamped

# TODO: Create launch logging configuration file
# UNIQUE_LOG_BASE_PATH = '~/uavrt_ws/src/data_streaming/log'


class Supervisor(Node):

    def __init__(self):
        super().__init__('supervisorNode')

        logger = self.get_logger()

        logger.set_level(LoggingSeverity.INFO)
        logger.info("Supervisor node has been created.")
        logger.info("Logging severity has been set to info.")

        self.connection = establishMavlinkConnection(logger)

        if self.connection != None and checkGPS(self.connection, logger) != False:
                x = float(getLongitude(self.connection, logger))
                y = float(getLatitude(self.connection, logger))
                z = float(getAltitude(self.connection, logger))


def main(args = None):
    rclpy.init(args = args)

    supervisorNode=Supervisor()
    # logger is a rclpy.impl.rcutils_logger.RcutilsLogger object
    logger=supervisorNode.get_logger()

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

# TODO: Need to add unit testing eventually.
