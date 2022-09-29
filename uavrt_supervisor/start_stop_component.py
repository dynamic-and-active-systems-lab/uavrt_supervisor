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

# https://docs.python.org/3/library/random.html
from random import randint

# https://docs.python.org/3/library/pathlib.html
from pathlib import Path

# https://docs.ros2.org/galactic/api/rclpy/api/node.html
from rclpy.node import Node
# https://docs.ros2.org/galactic/api/rclpy/api/logging.html
# Log messages can be formatted with native Python .format functioanlity!
from rclpy.logging import LoggingSeverity
# https://docs.ros2.org/galactic/api/rclpy/api/topics.html#module-rclpy.publisher
from rclpy.publisher import Publisher
# https://docs.ros2.org/galactic/api/rclpy/api/topics.html#module-rclpy.subscription
from rclpy.subscription import Subscription

# A set of packages which contain common interface files (.msg and .srv) for ROS2:
# https://github.com/ros2/common_interfaces/tree/master
#
# https://github.com/ros2/common_interfaces/tree/master/diagnostic_msgs
# http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer#Generating_Diagnostics_Input
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Custom uavrt message types
from uavrt_interfaces.msg import TagDef

# Enum values to describe the indice that is being accessed
from uavrt_supervisor.enum_members_values import DiagnosticStatusIndicesControl
from uavrt_supervisor.enum_members_values import KeyValueIndicesControl


class StartStopComponent(Node):
    def __init__(self):
        super().__init__('StartStopComponent')

        # Queue size is a required QoS (quality of service) setting that limits the
        # amount of queued messages if a subscriber is not receiving them fast enough.
        self._queue_size_ = 10

        # Note: This needs to be swapped out with a logging configuration
        # that goes with a launch file.
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Logging severity has been set to info.")

        self.get_logger().info("Start/Stop Component has been created.")

        # Control Start subscriber
        self._initialize_control_start_subscriber()

        # Control Stop subscriber
        self._initialize_control_stop_subscriber()

        # Control netcat airspyhf publisher
        self._initialize_control_netcat_airspyhf_publisher()

        # Control airspyhf channelizer publisher
        self._initialize_control_airspyhf_channelizer_publisher()

        # Control detector publisher
        self._initialize_control_detector_publisher()

    def _initialize_control_start_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._control_start_subscriber = self.create_subscription(
            TagDef,
            'control_start_subprocess',
            self._control_start_callback,
            self._queue_size_)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_start_subprocess' topic.")

    def _initialize_control_stop_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._control_stop_subscriber = self.create_subscription(
            DiagnosticArray,
            'control_stop_subprocess',
            self._control_stop_callback,
            self._queue_size_)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_stop_subprocess' topic.")

    def _initialize_control_netcat_airspyhf_publisher(self):
        # Format: Msg type, topic, queue size
        self._control_netcat_airspyhf_publisher = self.create_publisher(
            DiagnosticArray,
            'control_netcat_airspyhf_subprocess',
            self._queue_size_)
        self.get_logger().info(
            "Control publisher is now publishing to the 'control_netcat_airspyhf_subprocess' topic.")

    def _initialize_control_airspyhf_channelizer_publisher(self):
        # Format: Msg type, topic, queue size
        self._control_airspyhf_channelizer_publisher = self.create_publisher(
            DiagnosticArray,
            'control_airspyhf_channelize_subprocess',
            self._queue_size_)
        self.get_logger().info(
            "Control publisher is now publishing to the 'control_airspyhf_channelize_subprocess' topic.")

    def _initialize_control_detector_publisher(self):
        # Format: Msg type, topic, queue size
        self._control_detector_publisher = self.create_publisher(
            DiagnosticArray,
            'control_detector_subprocess',
            self._queue_size_)
        self.get_logger().info(
            "Control publisher is now publishing to the 'control_detector_subprocess' topic.")

    def _control_start_callback(self, message):
        status_array = DiagnosticArray()
        status = DiagnosticStatus()
        center_frequency_value = KeyValue()

        status_array.header.frame_id = "control"
        status_array.header.stamp = self.get_clock().now().to_msg()

        status.level = b'0'
        status.name = "X_component"
        status.message = "start"
        # The hardware_id corresponds to a random int value between 1 and 100000
        # There could be repeated hardware_ids but the chance is slim.
        # This should be fixed later.
        status.hardware_id = str(randint(1, 100000))

        center_frequency_value.key = "center_frequency"
        center_frequency_value.value = str(message.frequency)
        print("center_frequency_value.value: {}\n".format(center_frequency_value.value))

        status.values.append(center_frequency_value)

        status_array.status.append(status)

        # Note: This starting procedure must start airspyhf_channelizer
        # subprocesses before the netcat_airspyhf subproesses!
        # Else netcat_airspyhf will not start and stay running.
        # This is a limitation/constraint of the processes.
        self._control_airspyhf_channelizer_publisher.publish(status_array)
        self._control_netcat_airspyhf_publisher.publish(status_array)
        self._control_detector_publisher.publish(status_array)

    def _control_stop_callback(self, message):
        message_status_array = message.status[
            DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value]

        message_hardware_id = message_status_array.hardware_id

        if message_hardware_id == "stop all":
            # Note: This stopping procedure must stop the netcat_airspyhf
            # subprocesses before the airspyhf_channelizer subprocesses!
            # Else airspyhf_rx will not shut down properly.
            # This is a limitation/contraint of the processes.
            self._control_netcat_airspyhf_publisher(message)
            self._control_airspyhf_channelizer_publisher(message)
            self._control_detector_publisher(message)

            self.get_logger().info(
                "Successfully issued 'stop all' command to all subprocesses.")

    def _digest_tags(self):
        pass

    def _create_config_files(self):
        pass

    def _create_logs(self):
        pass
