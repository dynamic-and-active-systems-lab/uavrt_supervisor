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
# Note: Some behavior may be platform dependent.
from subprocess import Popen
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

# https://docs.python.org/3/library/pathlib.html
from pathlib import Path

# https://docs.python.org/3/library/socket.html
# Note: Some behavior may be platform dependent, since calls are made to the
# operating system socket APIs.
from socket import socket
from socket import AF_INET
from socket import SOCK_DGRAM

# https://docs.python.org/3/library/time.html
# Note: Use sparingly! Sleeping the main process will cause deadlock on that
# thread.
from time import sleep

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
from uavrt_supervisor.enum_members_values import DiagnosticStatusIndicesControl
from uavrt_supervisor.enum_members_values import KeyValueIndicesControl
from uavrt_supervisor.enum_members_values import DetectorSubprocessDictionary


class DetectorComponent(Node):
    def __init__(self):
        super().__init__('DetectorComponent')

        # Queue size is a required QoS (quality of service) setting that limits the
        # amount of queued messages if a subscriber is not receiving them fast enough.
        self._queue_size_ = 10
        # Rate at which status timer messages will be checked and published in seconds.
        self._status_timer_message_publish_rate = .5
        # This value can be increased/decreased
        self._detector_subprocess_limit = 5
        # Ensure that this counter is <= to the limit.
        self._detector_subprocess_counter = 0
        # Dictionary for storing detector subprocess objects.
        self._detector_subprocess_dictionary = {}
        # Directory where airspyhf_channelize is installed
        self._airspyhf_channelize_installation_directory = \
            Path("./uavrt_source/portairspyhf_channelize").resolve()
        # Directory where ros 2 galactic local_setup.bash script is installed
        self._uavrt_workspace_installation_directory = \
            Path("./install/local_setup.bash").resolve()
        # Receive port for function control commands
        # TEMP: REMOVE THIS
        self._detector_command_port = ("127.0.0.1", 30000)

        # Note: This needs to be swapped out with a logging configuration
        # that goes with a launch file.
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Logging severity has been set to info.")

        self.get_logger().info("Detector Component has been created.")

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
            'control_detector_subprocess',
            self._control_callback_subscriber,
            self._queue_size_)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_detector_subprocess' topic.")

    def _initialize_status_publisher(self):
        # Format: Msg type, topic, queue size
        self._status_publisher = self.create_publisher(
            DiagnosticArray,
            'status_detector_subprocess',
            self._queue_size_)
        self.get_logger().info(
            "Status publisher is now publishing to the 'status_detector_subprocess' topic.")

    def _initialize_status_timer(self):
        self._status_timer = self.create_timer(
            self._status_timer_message_publish_rate,
            self._status_timer_callback)
        self.get_logger().info(
            "Status timer is now publishing to the 'status_detector_subprocess' topic.")

    def _control_callback_subscriber(self, message):
        message_type = message.header.frame_id
        message_time = message.header.stamp

        message_status_array = message.status[
            DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value]

        message_level = message_status_array.level
        message_name = message_status_array.name
        message_message = message_status_array.message
        message_hardware_id = message_status_array.hardware_id

        # Required export command to use the detectors
        # Points to the directory that contains the airspyhf_channelize exe
        # That directory should also contain the .so objects that were used to
        # compile the executable.
        # https://stackoverflow.com/a/64391542
        airspyhf_channelize_export_string = {'LD_LIBRARY_PATH': str(
            self._airspyhf_channelize_installation_directory), 'HOME': "/home/dasl"}

        # Standard arguments string for starting a detector_subprocess
        # This string assumes that the detector config files are located in
        # ~/uavrt_workspace/uavrt_source/generated_tags
        # Split separate commands with newline chars: https://stackoverflow.com/a/38187706
        #
        # Note: The source command used depends on the ROS 2 install.
        # The following code assumes the suggested Ubuntu install by ROS 2
        # developers. This makes it a requirement.
        # ". /opt/ros/galactic/setup.bash\n" + \
        detector_standard_arguments_string = \
            "source ~/ros2_galactic/install/local_setup.bash\n" + \
            ". install/setup.bash\n" + \
            "cd ~/uavrt_workspace/uavrt_source/log/detector_log/config_example\n" + \
            "ros2 run uavrt_detection uavrt_detection"

        # Required to switch detector process to "Run" state
        # https://docs.python.org/3/library/stdtypes.html#int.to_bytes
        detector_run_byte = (1).to_bytes(length=1,
                                         byteorder='little')
        # https://pythontic.com/modules/socket/udp-client-server-example
        udp_send_command_socket = socket(family=AF_INET,
                                         type=SOCK_DGRAM)

        if message_message == "start":
            # Control messages such as "stop" do no include a KeyValue array.
            # Instead of creating a control callback for each control, we can
            # set the center frequency within the "start" branch.
            message_center_frequency_array = message_status_array.values[
                KeyValueIndicesControl.CENTER_FREQUENCY.value]
            message_center_frequency = message_center_frequency_array.value

            try:
                if self._detector_subprocess_counter >= self._detector_subprocess_limit:
                    raise Exception(
                        "The limit of detector subprocesses been reached.")
                # Start the new subprocess
                detector_subprocess = Popen(
                    detector_standard_arguments_string,
                    executable='/bin/bash',
                    # stdout=DEVNULL,
                    preexec_fn=setsid,
                    shell=True,
                    # CHANGE THIS CHANGE THIS CHANGE THIS
                    # cwd=str(
                    #    "/home/dasl/uavrt_workspace/uavrt_source/log/detector_log/config_example"),
                    env=airspyhf_channelize_export_string)
                # We need to sleep the parent thread for a second to allow
                # the detector process time to start up.
                # It's low complexity but dirty.
                sleep(5)
                # Issue run command via UDP socket
                # CHANGE THIS CHANGE THIS CHANGE THIS - SHOULD NOT BE A CONSTANT PORT
                udp_send_command_socket.sendto(detector_run_byte,
                                               self._detector_command_port)
                # Add subprocess to collection
                # The hardware_id corresponds to a random int value between 1 and 100000
                # There could be repeated hardware_ids but the chance is slim.
                # This should be fixed later in the event you have multiple detectors.
                # We save the center frquency as well since we will
                # need it later in order to restart the subprocess if need be.
                self._detector_subprocess_dictionary[message_hardware_id] = \
                    [message_center_frequency, detector_subprocess]
                # Increment counter
                self._detector_subprocess_counter += 1
                # Log
                self.get_logger().info("A new detector subprocesses has been started.")
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
                if self._detector_subprocess_counter <= 0:
                    raise Exception(
                        "The number of detector subprocesses is already 0.")
                # Iterate through subprocess dictionary and kill processes
                for subprocess_hardware_id in self._detector_subprocess_dictionary.keys():
                    # Kill process in collection
                    killpg(getpgid(
                        self._detector_subprocess_dictionary[subprocess_hardware_id][
                            DetectorSubprocessDictionary.DETECTOR_SUBPROCESS.value].pid),
                           SIGTERM)
                    # Remove subprocess from the collection
                    self._detector_subprocess_dictionary.pop(
                        subprocess_hardware_id)
                    # Decrement counter
                    self._detector_subprocess_counter -= 1
                    # Log
                    self.get_logger().info("A detector subprocesses has been stopped.")
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
        pass
