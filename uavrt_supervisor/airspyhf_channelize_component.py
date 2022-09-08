#!/usr/bin/env python3
#
# Codebase for the Supervisor package used within the UAV-RT architecture.
# Copyright (C) 2022 Dynamic and Active Systems Lab
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of  MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.

# https://docs.python.org/3/library/subprocess.html
from subprocess import Popen
from subprocess import run
from subprocess import CalledProcessError

# https://docs.ros2.org/galactic/api/rclpy/api/node.html
from rclpy.node import Node
# https://docs.ros2.org/galactic/api/rclpy/api/logging.html
# Log messages can be formatted with native Python .format functioanlity!
from rclpy.logging import LoggingSeverity
# https://docs.ros2.org/galactic/api/rclpy/api/topics.html#module-rclpy.publisher
from rclpy.publisher import Publisher
# https://docs.ros2.org/galactic/api/rclpy/api/topics.html#module-rclpy.subscription
from rclpy.subscription import Subscription
# https://docs.ros2.org/galactic/api/rclpy/api/timers.html
from rclpy.timer import Timer
from rclpy.timer import Rate

# A set of packages which contain common interface files (.msg and .srv) for ROS2:
# https://github.com/ros2/common_interfaces/tree/master
#
# Import the built-in Diagnostic messages that the node uses to timestamp and
# structure heartbeat status messages that it publishes on the /heartbeatStatus
# topic.
# https://github.com/ros2/common_interfaces/tree/master/diagnostic_msgs
# http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer#Generating_Diagnostics_Input
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Enum values to describe the indice that is being accessed
from uavrt_supervisor.enum_members_values import DiagnosticStatusIndiceControl
from uavrt_supervisor.enum_members_values import KeyValueIndicesControl
from uavrt_supervisor.enum_members_values import NetcatAirspyhfSubprocessDictionary


class AirspyfhChannelizeComponent(Node):
    def __init__(self):
        super().__init__('AirspyfhChannelizeComponent')

        # Queue size is a required QoS (quality of service) setting that limits the
        # amount of queued messages if a subscriber is not receiving them fast enough.
        self._queue_size_ = 10
        # Rate at which status timer messages will be checked and published in seconds.
        self._status_timer_message_publish_rate = .5
        # This value can be increased but the number of netcat_airspyhf
        # subprocesses needs to increase as well.
        self.airspyhf_channelize_subprocess_limit = 1
        # Ensure that this counter is <= to the limit.
        self.airspyhf_channelize_subprocess_counter = 0
        # Dictionary for storing netcat/airspyhf subprocess objects.
        self.airspyhf_channelize_subprocess_dictionary = {}
        #
        self.airspyhf_channelize_subprocess_sampling_rate = 192000
        #
        self.airspyhf_channelize_subprocess_decimation_rate = 48

        # Note: This needs to be swapped out with a logging configuration
        # that goes with a launch file.
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Logging severity has been set to info.")

        self.get_logger().info("Airspy Channelize Component has been created.")

        # Control subscriber
        self._initialize_control_subscriber()

        # Status publisher
        self._initialize_status_publisher()

        # Status timer
        self._initialize_status_timer()

    def _initialize_control_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._control_subscriber = self.create_subscription(
            DiagnosticArray,
            'control_airspyhf_channelize_subprocess',
            self._control_callback_subscriber,
            self._queue_size_)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_airspyhf_channelize_subprocess' topic.")

    def _initialize_status_publisher(self):
        # Format: Msg type, topic, queue size
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            'status_airspyhf_channelize_subprocess',
            self._queue_size_)
        self.get_logger().info(
            "Status publisher is now publishing to the 'status_airspyhf_channelize_subprocess' topic.")

    def _initialize_status_timer(self):
        self._status_timer = self.create_timer(
            self._status_timer_message_publish_rate,
            self._status_timer_callback)
        self.get_logger().info(
            "Status timer is now publishing to the 'status_airspyhf_channelize_subprocess' topic.")

    def _control_callback_subscriber(self, message):
        pass

    def _status_timer_callback(self):
        pass
