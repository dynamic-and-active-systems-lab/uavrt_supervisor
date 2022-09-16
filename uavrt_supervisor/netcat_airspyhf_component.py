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
from subprocess import DEVNULL

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
# https://github.com/ros2/common_interfaces/tree/master/diagnostic_msgs
# http://wiki.ros.org/diagnostics/Tutorials/Creating%20a%20Diagnostic%20Analyzer#Generating_Diagnostics_Input
from diagnostic_msgs.msg import DiagnosticArray
from diagnostic_msgs.msg import DiagnosticStatus
from diagnostic_msgs.msg import KeyValue

# Enum values to describe the indice that is being accessed
from uavrt_supervisor.enum_members_values import DiagnosticStatusIndiceControl
from uavrt_supervisor.enum_members_values import KeyValueIndicesControl
from uavrt_supervisor.enum_members_values import NetcatAirspyhfSubprocessDictionary
from uavrt_supervisor.enum_members_values import SystemControl


class NetcatAirspyhfComponent(Node):
    def __init__(self):
        super().__init__('NetcatAirspyhfComponent')

        # Queue size is a required QoS (quality of service) setting that limits the
        # amount of queued messages if a subscriber is not receiving them fast enough.
        self._queue_size_ = 10
        # Rate at which status timer messages will be checked and published in seconds.
        self._status_timer_message_publish_rate = .5
        # This value can be increased but the number of airspyhf_channelize
        # subprocesses needs to increase as well.
        self._netcat_airspyhf_subprocess_limit = 1
        # Ensure that this counter is <= to the limit.
        self._netcat_airspyhf_subprocess_counter = 0
        # Dictionary for storing netcat/airspyhf subprocess objects.
        self._netcat_airspyhf_subprocess_dictionary = {}

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
        self._control_subscriber = self.create_subscription(
            DiagnosticArray,
            'control_netcat_airspyhf_subprocess',
            self._control_callback_subscriber,
            self._queue_size_)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_netcat_airspyhf_subprocess' topic.")

    def _initialize_status_publisher(self):
        # Format: Msg type, topic, queue size
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            'status_netcat_airspyhf_subprocess',
            self._queue_size_)
        self.get_logger().info(
            "Status publisher is now publishing to the 'status_netcat_airspyhf_subprocess' topic.")

    def _initialize_status_timer(self):
        self._status_timer = self.create_timer(
            self._status_timer_message_publish_rate,
            self._status_timer_callback)
        self.get_logger().info(
            "Status timer is now publishing to the 'status_netcat_airspyhf_subprocess' topic.")

    def _control_callback_subscriber(self, message):
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
        # Note: Using the -w argument within the Netcat call doesn't always work
        # for Macos. The UAVRT system doesn't support Macos at the moment, so
        # it is not a current concern.
        # The timeout value has to be greater than 0 else the airspyhf_rx tool
        # process will not stay alive.
        netcat_airspyhf_standard_arguments_string = \
            "/usr/local/bin/airspyhf_rx -f " + message_center_frequency + " -m on " \
            "-a " + message_sample_rate + " -r stdout -g on -l high -t 0 | netcat " \
            "-w 1 -u localhost 10000"

        # Note: A match switch-case statement would work here as well.
        # It would follow the format: match message_message: ... case "start":
        # ... etc. However, the match keyword was introduced in Python version
        # 3.10, which isn't being used for this iteration of the UAVRT system.
        # https://docs.python.org/3.10/whatsnew/3.10.html#pep-634-structural-pattern-matching
        if message_message == "start":
            try:
                if self._netcat_airspyhf_subprocess_counter >= self._netcat_airspyhf_subprocess_limit:
                    raise Exception(
                        "The limit of netcat/airspyhf subprocesses been reached.")
                # Test if the the airspyhf radio is plugged in/available.
                # If check is true, and the process exits with a non-zero exit code,
                # a CalledProcessError exception will be raised.
                test_airtspyhf_subprocess = run(
                    "/usr/local/bin/airspyhf_info",
                    check=True,
                    stdout=DEVNULL)
                # Start the new subprocess
                # Use Popen() instead of run() because processes started with
                # subprocess.run() do not have access to the poll functionality.
                netcat_airspyhf_subprocess = Popen(
                    netcat_airspyhf_standard_arguments_string,
                    stdout=DEVNULL,
                    shell=True)
                # Add subprocess to collection
                # The message_hardware_id will correspond to the hardware_id
                # of the airspyhf_channelize subprocess that was started
                # prior to this netcat_airspy_subprocess.
                # We save the frquency and sample rate as well since we will
                # need them later in order to restart the subprocess if need be.
                self._netcat_airspyhf_subprocess_dictionary[message_hardware_id] = \
                    [message_center_frequency, message_sample_rate,
                     netcat_airspyhf_subprocess]
                # Increment counter
                self._netcat_airspyhf_subprocess_counter += 1
                # Log
                self.get_logger().info("A new netcat/airspyhf subprocesses has been started.")
            except (CalledProcessError, Exception) as instance:
                # Publish status message with ERROR level
                message.status[DiagnosticStatusIndiceControl.DIAGNOSTIC_STATUS.value].level = b'2'
                self.get_logger().error("Type: {}".format(type(instance)))
                self.get_logger().error("Message: {}".format(instance))
            finally:
                # Publish status message using original message
                self._status_publisher.publish(message)

        elif message_message == "stop" and message_hardware_id == str(SystemControl.STOP_ALL_SUBPROCESS.value):
            try:
                if self._netcat_airspyhf_subprocess_counter <= 0:
                    raise Exception(
                        "The number of netcat/airspyhf subprocesses is already 0.")
                # Iterate through subprocess dictionary and kill processes
                for subprocess_hardware_id in self._netcat_airspyhf_subprocess_dictionary.keys():
                    # Kill process in collection
                    self._netcat_airspyhf_subprocess_dictionary[subprocess_hardware_id][
                        NetcatAirspyhfSubprocessDictionary.NETCAT_AIRSPYHF_SUBPROCESS.value].kill
                    # Remove subprocess from the collection
                    self._netcat_airspyhf_subprocess_dictionary.pop(
                        message_hardware_id)
                    # Decrement counter
                    self._netcat_airspyhf_subprocess_counter -= 1
                    # Log
                    self.get_logger().info("A netcat/airspyhf subprocesses has been stopped.")
            except Exception as instance:
                # Publish status message with ERROR level
                message.status[DiagnosticStatusIndiceControl.DIAGNOSTIC_STATUS.value].level = b'2'
                self.get_logger().error("Type: {}".format(type(instance)))
                self.get_logger().error("Message: {}".format(instance))
            finally:
                # Publish status message using original message
                self._status_publisher.publish(message)

    def _status_timer_callback(self):
        _status_array = DiagnosticArray()
        _status = DiagnosticStatus()
        _center_frequency_value = KeyValue()
        _sample_rate_value = KeyValue()

        # Iterate through subprocess collection, polling each subprocess
        # Create new Diagnostic message. Publish as status
        try:
            for subprocess_hardware_id in self._netcat_airspyhf_subprocess_dictionary.keys():
                _status_array.header.frame_id = "status"
                _status_array.header.stamp = self.get_clock().now().to_msg()

                _status.name = "netcat_airspyhf_component"
                _status.hardware_id = subprocess_hardware_id

                _center_frequency_value.key = "center_frequency"
                _center_frequency_value.value = self._netcat_airspyhf_subprocess_dictionary[
                    subprocess_hardware_id][NetcatAirspyhfSubprocessDictionary.CENTER_FREQUENCY.value]

                _sample_rate_value.key = "sample_rate"
                _sample_rate_value.value = self._netcat_airspyhf_subprocess_dictionary[
                    subprocess_hardware_id][NetcatAirspyhfSubprocessDictionary.SAMPLE_RATE.value]

                _status.values.append(_center_frequency_value)
                _status.values.append(_sample_rate_value)

                # A None value indicates that the process hasn’t terminated yet.
                if self._netcat_airspyhf_subprocess_dictionary[
                        subprocess_hardware_id][NetcatAirspyhfSubprocessDictionary.NETCAT_AIRSPYHF_SUBPROCESS.value].poll() == None:
                    _status.level = b'0'
                    _status.message = "alive"
                # Any value other than None means that the processes has terminated.
                # If subprocess is dead, remove subprocess object from collection
                # and publish status message and log message. Use ERROR level.
                elif self._netcat_airspyhf_subprocess_dictionary[
                        subprocess_hardware_id][NetcatAirspyhfSubprocessDictionary.NETCAT_AIRSPYHF_SUBPROCESS.value].poll() != None:
                    _status.level = b'2'
                    _status.message = "dead"

                    # Kill process in collection
                    self._netcat_airspyhf_subprocess_dictionary[subprocess_hardware_id][2].kill
                    # Remove subprocess from the collection
                    self._netcat_airspyhf_subprocess_dictionary.pop(
                        subprocess_hardware_id)
                    # Decrement counter
                    self._netcat_airspyhf_subprocess_counter -= 1

                    self.get_logger().info(
                        "A netcat/airspyhf subprocesses has been stopped since it was dead.")

                _status_array.status.append(_status)

                # Publish status message
                self._status_publisher.publish(_status_array)
        # RuntimeError: Dictionary changed size during iteration.
        # It seems impossible to get away from this error. W/e collection
        # type we use, it must be modified during Runtime.
        except RuntimeError as instance:
            self.get_logger().error("Type: {}".format(type(instance)))
            self.get_logger().error("Message: {}".format(instance))
