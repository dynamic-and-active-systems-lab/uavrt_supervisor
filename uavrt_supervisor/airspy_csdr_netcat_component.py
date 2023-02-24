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

# Using subprocess.Popen.kill() will not work when attempting to kill a
# subproces that is started with the Shell=True argument. The SO link below
# describes the correct process.
# https://stackoverflow.com/a/4791612
from os import killpg
from os import setsid
from os import getpgid
from signal import SIGTERM

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
from uavrt_supervisor.enum_members_values import SubprocessConstants
from uavrt_supervisor.enum_members_values import DiagnosticStatusIndicesControl
from uavrt_supervisor.enum_members_values import KeyValueIndicesControl
from uavrt_supervisor.enum_members_values import AirspyCSDRNetcatComponentSubprocessDictionary


class AirspyCSDRNetcatComponent(Node):
    def __init__(self):
        super().__init__('AirspyCSDRNetcatComponent')

        # This value can be increased but the number of channelizer
        # subprocesses needs to increase as well.
        self._airspy_csdr_netcat_subprocess_limit = 1
        # Ensure that this counter is <= to the limit.
        self._airspy_csdr_netcat_subprocess_counter = 0
        # Dictionary for storing airspy_csdr_netcat subprocess objects.
        self._airspy_csdr_netcat_subprocess_dictionary = {}

        # Note: This needs to be swapped out with a logging configuration
        # that goes with a launch file.
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Logging severity has been set to info.")

        self.get_logger().info("Airspy_CSDR_Netcat_Component has been created.")

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
            'control_airspy_csdr_netcat_subprocess',
            self._control_callback_subscriber,
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_airspy_csdr_netcat_subprocess' topic.")

    def _initialize_status_publisher(self):
        # Format: Msg type, topic, queue size
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            'status_airspy_csdr_netcat_subprocess',
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Status publisher is now publishing to the 'control_airspy_csdr_netcat_subprocess' topic.")

    def _initialize_status_timer(self):
        self._status_timer = self.create_timer(
            SubprocessConstants.STATUS_TIMER_MESSAGE_PUBLISH_RATE.value,
            self._status_timer_callback)
        self.get_logger().info(
            "Status timer has been started for the 'status_airspy_csdr_netcat_subprocess' topic.")

    def _control_callback_subscriber(self, message):
        message_type = message.header.frame_id
        message_time = message.header.stamp

        message_status_array = message.status[
            DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value]

        message_level = message_status_array.level
        message_name = message_status_array.name
        message_message = message_status_array.message
        message_hardware_id = message_status_array.hardware_id

        # Note: A match switch-case statement would work here as well.
        # It would follow the format: match message_message: ... case "start":
        # ... etc. However, the match keyword was introduced in Python version
        # 3.10, which isn't being used for this iteration of the UAVRT system.
        # https://docs.python.org/3.10/whatsnew/3.10.html#pep-634-structural-pattern-matching
        if message_message == "start":
            # Control messages such as "stop" do not include a KeyValue array.
            # Instead of creating a control callback for each control, we can
            # set the center frequency within the "start" branch.
            message_center_frequency_array = message_status_array.values[
                KeyValueIndicesControl.CENTER_FREQUENCY.value]
            message_center_frequency = message_center_frequency_array.value

            # Standard arguments string for starting a airspy_csdr_netcat subprocess.
            # Read the Security Considerations section before using shell=True
            # elsewhere. It is used here since user input is a factor here.
            # https://docs.python.org/3/library/subprocess.html#security-considerations
            # Note: Using the -w argument within the Netcat call doesn't always work
            # for Macos. The UAVRT system doesn't support Macos at the moment, so
            # it is not a current concern.
            # The timeout value has to be greater than 0 else the airspy_rx tool
            # process will not stay alive.
            airspy_csdr_netcat_standard_arguments_string = \
                "/usr/local/bin/airspy_rx -f " + message_center_frequency + " -r - " \
                "-p 0 -a " + str(SubprocessConstants.RADIO_SAMPLING_RATE.value) + " -t 0 -d " \
                " | csdr fir_decimate_cc 8 0.05 HAMMING | netcat -w 1 -u localhost 10000"
            try:
                if (self._airspy_csdr_netcat_subprocess_counter >=
                        self._airspy_csdr_netcat_subprocess_limit):
                    raise Exception(
                        "The limit of airspy_csdr_netcat subprocesses been reached.")
                # Test if the the airspy radio is plugged in/available.
                # If check is true, and the process exits with a non-zero exit code,
                # a CalledProcessError exception will be raised.
                test_airspy_subprocess = run(
                    "/usr/local/bin/airspy_info",
                    check=True,
                    stdout=DEVNULL)
                # Start the new subprocess
                # Use Popen() instead of run() because processes started with
                # subprocess.run() do not have access to the poll functionality.
                airspy_csdr_netcat_subprocess = Popen(
                    airspy_csdr_netcat_standard_arguments_string,
                    stdout=DEVNULL,
                    stderr=DEVNULL,
                    preexec_fn=setsid,
                    shell=True)
                # Add subprocess to collection
                # We save the center frequency as well since we will
                # need it later in order to restart the subprocess if need be.
                self._airspy_csdr_netcat_subprocess_dictionary[message_hardware_id] = \
                    [message_center_frequency, airspy_csdr_netcat_subprocess]
                # Increment counter
                self._airspy_csdr_netcat_subprocess_counter += 1
                # Log
                self.get_logger().info("A new airspy_csdr_netcat subprocess has been started.")
            except (CalledProcessError, Exception) as instance:
                # Publish status message with ERROR level
                message.status[DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value].level = b'2'
                self.get_logger().error("Type: {}".format(type(instance)))
                self.get_logger().error("Message: {}".format(instance))
            finally:
                # Publish status message using original message
                self._status_publisher.publish(message)

        elif message_message == "stop" and message_hardware_id == "stop all":
            try:
                if self._airspy_csdr_netcat_subprocess_counter <= 0:
                    raise Exception(
                        "The number of airspy_csdr_netcat subprocesses is already 0.")
                # Iterate through subprocess dictionary and kill processes
                for subprocess_hardware_id in self._airspy_csdr_netcat_subprocess_dictionary.keys():
                    # Kill process in collection
                    killpg(getpgid(
                        self._airspy_csdr_netcat_subprocess_dictionary[subprocess_hardware_id][
                            AirspyCSDRNetcatComponentSubprocessDictionary.AIRSPY_CSDR_NETCAT_SUBPROCESS.value].pid),
                           SIGTERM)
                    # Remove subprocess from the collection
                    self._airspy_csdr_netcat_subprocess_dictionary.pop(
                        subprocess_hardware_id)
                    # Decrement counter
                    self._airspy_csdr_netcat_subprocess_counter -= 1
                    # Log
                    self.get_logger().info("A airspy_csdr_netcat subprocesses has been stopped.")
            except Exception as instance:
                # Publish status message with ERROR level
                message.status[DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value].level = b'2'
                self.get_logger().error("Type: {}".format(type(instance)))
                self.get_logger().error("Message: {}".format(instance))
            except RuntimeError as instance:
                # Whenever the dictionary size changes during runtime, this
                # error will be thrown. It is not fatal, so we catch, report,
                # and move on.
                self.get_logger().info("Type: {}".format(type(instance)))
                self.get_logger().info("Message: {}".format(instance))
            finally:
                # Publish status message using original message
                self._status_publisher.publish(message)

    def _status_timer_callback(self):
        status_array = DiagnosticArray()
        status = DiagnosticStatus()
        center_frequency_value = KeyValue()
        sample_rate_value = KeyValue()

        # Iterate through subprocess collection, polling each subprocess
        # Create new Diagnostic message, publish as status
        try:
            for subprocess_hardware_id in self._airspy_csdr_netcat_subprocess_dictionary.keys():
                status_array.header.frame_id = "status"
                status_array.header.stamp = self.get_clock().now().to_msg()

                status.name = "airspy_csdr_netcat_component"
                status.hardware_id = subprocess_hardware_id

                center_frequency_value.key = "center_frequency"
                center_frequency_value.value = self._airspy_csdr_netcat_subprocess_dictionary[
                    subprocess_hardware_id][AirspyCSDRNetcatComponentSubprocessDictionary.CENTER_FREQUENCY.value]

                status.values.append(center_frequency_value)

                # A None value indicates that the process hasn’t terminated yet.
                if self._airspy_csdr_netcat_subprocess_dictionary[
                        subprocess_hardware_id][AirspyCSDRNetcatComponentSubprocessDictionary.AIRSPY_CSDR_NETCAT_SUBPROCESS.value].poll() == None:
                    status.level = b'0'
                    status.message = "alive"
                # Any value other than None means that the processes has terminated.
                # If subprocess is dead, remove subprocess object from collection
                # and publish status message and log message. Use ERROR level.
                elif self._airspy_csdr_netcat_subprocess_dictionary[
                        subprocess_hardware_id][AirspyCSDRNetcatComponentSubprocessDictionary.AIRSPY_CSDR_NETCAT_SUBPROCESS.value].poll() != None:
                    status.level = b'2'
                    status.message = "dead"
                    # Kill process in collection
                    killpg(getpgid(
                        self._airspy_csdr_netcat_subprocess_dictionary[subprocess_hardware_id][
                            AirspyCSDRNetcatComponentSubprocessDictionary.AIRSPY_CSDR_NETCAT_SUBPROCESS.value].pid),
                           SIGTERM)
                    # Remove subprocess from the collection
                    self._airspy_csdr_netcat_subprocess_dictionary.pop(
                        subprocess_hardware_id)
                    # Decrement counter
                    self._airspy_csdr_netcat_subprocess_counter -= 1
                    self.get_logger().info(
                        "A airspy_csdr_netcat subprocess has been stopped since it was dead.")

                status_array.status.append(status)

        # ProcessLookupError: [Errno 3] No such process
        # killpg(getpgid())
        except ProcessLookupError as instance:
        # Publish status message with ERROR level
            message.status[DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value].level = b'2'
            self.get_logger().error("Type: {}".format(type(instance)))
            self.get_logger().error("Message: {}".format(instance))
        # RuntimeError: Dictionary changed size during iteration.
        # It seems impossible to get away from this error. W/e collection
        # type we use, it must be modified during Runtime.
        except RuntimeError as instance:
            self.get_logger().error("Type: {}".format(type(instance)))
            self.get_logger().error("Message: {}".format(instance))
        finally:
            # Publish status message using original message
            self._status_publisher.publish(status_array)
