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
from subprocess import PIPE

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


class NetcatAirspyhfComponent(Node):

    def __init__(self):
        super().__init__('NetcatAirspyhfComponent')

        # Queue size is a required QoS (quality of service) setting that limits the
        # amount of queued messages if a subscriber is not receiving them fast enough.
        self._queue_size_ = 10
        # Rate at which status messages will be checked and published in seconds.
        self._status_message_publish_rate = .5
        # This value can be increased but the number of airspyhf_channelize
        # subprocesses needs to increase as well.
        self.netcat_airspyhf_subprocess_limit = 1
        # Ensure that this counter is <= to the limit.
        self.netcat_airspyhf_subprocess_counter = 0
        # Number of times that a subprocess will be restarted untill it timesout.
        self.netcat_airspyhf_subprocess_attempt_limit = 5

        # Note: This needs to be swapped out with a logging configuration
        # that goes with a launch file.
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Logging severity has been set to info.")

        self.get_logger().info("Netcat-Airspyhf Component has been created.")

        # Control subscriber
        self._initialize_control_subscriber()

        # Status publisher
        self._initialize_status_publisher()

        # Status timer
        self._initialize_status_timer()

    def _initialize_control_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self.subscription = self.create_subscription(
            DiagnosticArray,
            'control_netcat_airspyhf',
            self.control_callback_subscriber,
            self._queue_size_)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_netcat_airspyhf' topic.")

    def control_callback_subscriber(self, message):
        message_type = message.header.frame_id
        message_time = message.header.stamp

        message_status_array = message.status[
            DiagnosticStatusIndiceControl.DIAGNOSTIC_STATUS.value]

        message_level = message_status_array.level
        message_name = message_status_array.name
        message_message = message_status_array.message
        message_hardware_id = message_status_array.hardware_id

        message_center_frequency_array = message_status_array.values[
            KeyValueIndicesControl.CENTER_FREQUENCY.value]
        message_center_frequency = message_center_frequency_array.value

        message_sample_rate_array = message_status_array.values[
            KeyValueIndicesControl.SAMPLE_RATE.value]
        message_sample_rate = message_sample_rate_array.value

        # Standard arguments string for starting a netcat_airspyhf_subprocess
        # Read the Security Considerations section before using shell=True
        # elsewhere. It is used here since user input is a factor here.
        # https://docs.python.org/3/library/subprocess.html#security-considerations
        netcat_airspyhf_standard_arguments_string = \
            "/usr/local/bin/airspyhf_rx -f " + message_center_frequency + " -m on " \
            "-a " + message_sample_rate + " -r stdout -g on -l high -t 0 | netcat " \
            "-u localhost 10000"

        if message_message == "start":
            if self.netcat_airspyhf_subprocess_counter < self.netcat_airspyhf_subprocess_limit:
                self.netcat_airspyhf_subprocess_counter += 1

                netcat_airspyhf_subprocess = Popen(
                    netcat_airspyhf_standard_arguments_string, shell=True)

                # Add subprocess to collection
                # Publish status message using original message
                # Log

            elif self.netcat_airspyhf_subprocess_counter >= self.netcat_airspyhf_subprocess_limit:
                # Publish status message with ERROR level
                # Log
                pass
        elif message_message == "stop":
            # Kill process in collection
            # Remove subprocess from the collection
            # Publish status message with STALE level
            # Log
            pass

    def _initialize_status_publisher(self):
        # Format: Msg type, topic, queue size
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            'status_netcat_airspyhf',
            self._queue_size_)
        self.get_logger().info(
            "Status publisher is now publishing to the 'status_netcat_airspyhf' topic.")

    def _initialize_status_timer(self):
        self.timer = self.create_timer(
            self._status_message_publish_rate,
            self._status_timer_callback)
        self.get_logger().info(
            "Status time is now publishing to the 'status_netcat_airspyhf' topic.")

    def _status_timer_callback(self):
        _status_array = DiagnosticArray()
        _status = DiagnosticStatus()
        _value = KeyValue()

        # Iterate through subprocess collection, polling each subprocess
        # Create new Diagnostic message. Publish as status
        # If subprocess is dead, remove subprocess object from collection
        # and publish status message and log message. Use ERROR level.
        # If subprocess attempts is greater than netcat_airspyhf_subprocess_attempt_limit,
        # cease attempts and notify user that a STOP is necessary.
        # Also check stderr for errors. 
        #
        # Note: Keep subprocess limit in the status message values array.
