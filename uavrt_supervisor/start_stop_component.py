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

# https://docs.python.org/3/library/os.html#process-parameters
from os import getcwd

# https://docs.python.org/3/library/time.html
from time import time
from time import gmtime
from time import strftime

# Necessary for converting
# https://numpy.org/doc/stable/reference/generated/numpy.array.html
from numpy import array

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
# https://github.com/ros2/common_interfaces/blob/master/std_msgs/msg/Bool.msg
from std_msgs.msg import Bool

# Custom uavrt message types
from uavrt_interfaces.msg import TagDef

# Radio tuner functionality to find new center frequencies for tags
from uavrt_supervisor.tuner import tuner

# Enum values to describe the constants used with the uavrt_supervisor package
from uavrt_supervisor.enum_members_values import SubprocessConstants
from uavrt_supervisor.enum_members_values import TunerOutputConstants
from uavrt_supervisor.enum_members_values import DiagnosticStatusIndicesControl
from uavrt_supervisor.enum_members_values import KeyValueIndicesControl


class StartStopComponent(Node):
    def __init__(self):
        super().__init__('StartStopComponent')

        # List that contains the tag objects for one "flight"
        self._tag_object_list = []
        # List that contains the tag frequencies for one "flight"
        self._tag_frequency_list = []
        # Radio center frequency that is found using radio_tuner.py
        self._radio_center_frequency = 0
        # Number of tags sent over for processing
        self._number_of_tags = 0
        # List that contains the config paths for each of the detectors
        # Note: This is cleared after each start up call.
        self._config_path_list = []

        # Note: This needs to be swapped out with a logging configuration
        # that goes with a launch file.
        self.get_logger().set_level(LoggingSeverity.INFO)
        self.get_logger().info("Logging severity has been set to info.")

        self.get_logger().info("Start/Stop Component has been created.")

        # Store tag information subscriber
        self._initialize_store_tag_information_subscriber()

        # Release tag information subscriber
        self._initialize_release_tag_information_subscriber()

        # Control Start subscriber
        self._initialize_control_start_subscriber()

        # Control Stop subscriber
        self._initialize_control_stop_subscriber()

        # Control netcat airspyhf publisher
        self._initialize_control_airspy_csdr_netcat_publisher()

        # Control airspyhf channelizer publisher
        self._initialize_control_channelizer_publisher()

        # Control detector publisher
        self._initialize_control_detector_publisher()

    def _initialize_store_tag_information_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._store_tag_information_subscriber = self.create_subscription(
            TagDef,
            'store_tag_information',
            self._store_tag_information_callback,
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Subscriber is now subscribing to the 'store_tag_information' topic.")

    def _initialize_release_tag_information_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._control_start_subscriber = self.create_subscription(
            Bool,
            'release_tag_information',
            self._release_tag_information_callback,
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Subscriber is now subscribing to the 'release_tag_information' topic.")

    def _initialize_control_start_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._control_start_subscriber = self.create_subscription(
            DiagnosticArray,
            'control_start_subprocess',
            self._control_start_callback,
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_start_subprocess' topic.")

    def _initialize_control_stop_subscriber(self):
        # Format: Msg type, topic, callback, queue size
        self._control_stop_subscriber = self.create_subscription(
            DiagnosticArray,
            'control_stop_subprocess',
            self._control_stop_callback,
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Control subscriber is now subscribing to the 'control_stop_subprocess' topic.")

    def _initialize_control_airspy_csdr_netcat_publisher(self):
        # Format: Msg type, topic, queue size
        self._control_airspy_csdr_netcat_publisher = self.create_publisher(
            DiagnosticArray,
            'control_airspy_csdr_netcat_subprocess',
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Control publisher is now publishing to the 'control_netcat_airspyhf_subprocess' topic.")

    def _initialize_control_channelizer_publisher(self):
        # Format: Msg type, topic, queue size
        self._control_channelizer_publisher = self.create_publisher(
            DiagnosticArray,
            'control_channelizer_subprocess',
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Control publisher is now publishing to the 'control_channelizer_subprocess' topic.")

    def _initialize_control_detector_publisher(self):
        # Format: Msg type, topic, queue size
        self._control_detector_publisher = self.create_publisher(
            DiagnosticArray,
            'control_detector_subprocess',
            SubprocessConstants.QUEUE_SIZE.value)
        self.get_logger().info(
            "Control publisher is now publishing to the 'control_detector_subprocess' topic.")

    def _store_tag_information_callback(self, message):
        self._tag_object_list.append(message)

        # Need to convert NNNNNN to NNN.NNN
        self._tag_frequency_list.append(message.frequency / 1000)

        # Increment the number of tags
        self._number_of_tags = + 1

        self.get_logger().info(
            "Successfully stored an additional tag.")

    def _release_tag_information_callback(self, message):
        if message.data == True:
            radio_tuner_output = tuner(SubprocessConstants.CHANNELIZER_SAMPLING_RATE.value,
                                       SubprocessConstants.CHANNELIZER_DECIMATION_RATE.value,
                                       array(self._tag_frequency_list))

            # Empty tag frequency list after finding the correct frequencies
            self._tag_frequency_list.clear()

            # Optimal radio center frequency in MHz
            self._radio_center_frequency = radio_tuner_output[
                TunerOutputConstants.RADIO_CENTER_FREQENCY.value]

            # The center frequencies of each channel(ascending) in MHz
            center_channel_frequency_list = radio_tuner_output[
                TunerOutputConstants.CENTER_CHANNEL_FREQUENCIES.value]

            # The index of the channel that each tag should be present in
            tag_channel_number_list = radio_tuner_output[
                TunerOutputConstants.TAG_CHANNEL_NUMBER.value]

            self._create_log_directories(center_channel_frequency_list,
                                         tag_channel_number_list)

            # Empty tag object list after creating directories and config files
            self._tag_object_list.clear()

            self.get_logger().info(
                "Successfully released all tags.")

    def _control_start_callback(self, message):
        status_array = DiagnosticArray()
        status = DiagnosticStatus()
        center_frequency_value = KeyValue()
        config_path_value = KeyValue()

        status_array.header.frame_id = "control"
        status_array.header.stamp = self.get_clock().now().to_msg()

        status.level = b'0'
        status.name = "NA"
        status.message = "start"
        status.hardware_id = "start all"

        center_frequency_value.key = "center_frequency"
        center_frequency_value.value = str(self._radio_center_frequency)

        status.values.append(center_frequency_value)

        status_array.status.append(status)

        # Note: This starting procedure must start airspyhf_channelizer
        # subprocesses before the netcat_airspyhf subproesses!
        # Else netcat_airspyhf will stop prematurely.
        # This is a limitation/constraint of the processes.
        self._control_channelizer_publisher.publish(status_array)
        self._control_airspy_csdr_netcat_publisher.publish(status_array)

        # We need to pass the config paths down to each of the detectors.
        # In order to do so, I am removing the center_frequency_value that was
        # previously appended. Otherwise, it would be appended again.
        # Note: In hindsight, I think this issue could have been circumnavigated
        # if I used a custom message type.
        # status.values.remove(center_frequency_value)

        for tag in range(self._number_of_tags):
            # Create a key value pair to send down the config path
            config_path_value.key = "config_path"
            config_path_value.value = str(self._config_path_list[tag])

            status.values.append(config_path_value)

            status_array.status.append(status)

            self._control_detector_publisher.publish(status_array)

            # Once the key value pair is published, we need to remove to add
            # a new config path with each iteration of the for loop
            status.values.remove(config_path_value)

        self.get_logger().info(
            "Successfully issued 'start all' command to all subprocesses.")

        # Clear the number of tags
        self._number_of_tags = 0

        # Clear the config path list
        self._config_path_list.clear()

    def _control_stop_callback(self, message):
        message_status_array = message.status[
            DiagnosticStatusIndicesControl.DIAGNOSTIC_STATUS.value]

        message_hardware_id = message_status_array.hardware_id

        if message_hardware_id == "stop all":
            # Note: This stopping procedure must stop the netcat_airspyhf
            # subprocesses before the airspyhf_channelizer subprocesses!
            # Else airspyhf_rx will not shut down properly.
            # This is a limitation/contraint of the processes.
            self._control_airspy_csdr_netcat_publisher.publish(message)
            self._control_channelizer_publisher.publish(message)
            self._control_detector_publisher.publish(message)

            self.get_logger().info(
                "Successfully issued 'stop all' command to all subprocesses.")

    def _create_log_directories(self,
                                center_channel_frequency_list,
                                tag_channel_number_list):
        source_directory_path = Path("./uavrt_source/log").resolve()

        # Converting POSIX time to UTC in the format of year, month, day, hour,
        # minute, second
        converted_time = strftime(
            '%Y-%m-%d_%H:%M:%S', gmtime(time()))

        # New flight directory name
        flight_log_directory_name = "flight_log_" + converted_time

        # Make the directory for the new flight
        flight_log_directory_path = Path(source_directory_path,
                                         flight_log_directory_name)

        flight_log_directory_path.mkdir()

        # Make the directory for the netcat_airspyhf radio logs
        netcat_airspyhf_log_directory_path = Path(flight_log_directory_path,
                                                  "netcat_airspyhf_log")

        netcat_airspyhf_log_directory_path.mkdir()

        # Make the directory for the airspyhf_channelizer radio logs
        airspyhf_channelizer_log_directory_path = Path(flight_log_directory_path,
                                                       "airspyhf_channelizer_log")

        airspyhf_channelizer_log_directory_path.mkdir()

        # Make the directory for the detector logs
        detector_logging_directory_path = Path(flight_log_directory_path,
                                               "detector_log")

        detector_logging_directory_path.mkdir()

        # Make individual detector directories
        for tag_object in range(len(self._tag_object_list)):

            detector_specific_directory_path = Path(detector_logging_directory_path,
                                                    "tag_id_{}".format(self._tag_object_list[tag_object].tag_id))

            detector_specific_directory_path.mkdir()

            # Make directory for output files
            output_directory_path = Path(detector_specific_directory_path,
                                         "output")

            output_directory_path.mkdir()

            # Make directory for config file
            config_directory_path = Path(detector_specific_directory_path,
                                         "config")

            config_directory_path.mkdir()

            # Add config paths to list so that they can be sent to the detectors
            # on start up
            self._config_path_list.append(detector_specific_directory_path)

            self._create_config_file(
                self._tag_object_list[tag_object],
                config_directory_path,
                output_directory_path,
                center_channel_frequency_list,
                tag_channel_number_list)

    def _create_config_file(self,
                            tag_object,
                            config_directory_path,
                            output_directory_path,
                            center_channel_frequency_list,
                            tag_channel_number_list):
        # The name of the config files does not differ
        config_file_path = Path(config_directory_path, "detectorConfig.txt")

        # Make bin file path for data record
        data_bin_path = Path(output_directory_path, "data_record.bin")

        # Note: Minus 1 since value of tag ids start at 1
        tag_index_value = int(tag_channel_number_list[tag_object.tag_id - 1])
        # Center channel frequency for associated tag
        center_frequency = float(
            center_channel_frequency_list[tag_index_value])

        # Port number for incoming data
        port_number_data = str(20000 + tag_index_value)
        # Port number for incoming control commands
        port_number_control = str(30000 + tag_index_value)

        # Sampling rate for each of the detectors
        detector_sampling_rate = str(SubprocessConstants.CHANNELIZER_SAMPLING_RATE.value /
                                     SubprocessConstants.CHANNELIZER_DECIMATION_RATE.value)

        # TODO: Edit out k and falseAlarmProb hardcoded values.
        file = open(config_file_path, "w")
        file.write("##################################################" + "\n" +
                   "ID:" + "\t" + str(tag_object.tag_id) + "\n" +
                   "channelCenterFreqMHz:" + "\t" + str(center_frequency) + "\n" +
                   "timeStamp:" + "\t" + str(time()) + "\n" +
                   "ipData:" + "\t" + "0.0.0.0" + "\n" +
                   "portData:" + "\t" + port_number_data + "\n" +
                   "ipCntrl:" + "\t" + "127.0.0.1" + "\n" +
                   "portCntrl:" + "\t" + port_number_control + "\n" +
                   "Fs:" + "\t" + detector_sampling_rate + "\n" +
                   # Converting miliseconds to seconds
                   "tagFreqMHz:" + "\t" + str(tag_object.frequency / 1000) + "\n" +
                   "tp:" + "\t" + str(tag_object.pulse_duration / 1000) + "\n" +
                   "tip:" + "\t" + str(tag_object.interpulse_time_1 / 1000) + "\n" +
                   "tipu:" + "\t" + str(tag_object.interpulse_time_uncert / 1000) + "\n" +
                   "tipj:" + "\t" + str(tag_object.interpulse_time_jitter / 1000) + "\n" +
                   "K:" + "\t" + "3" + "\n" +
                   "opMode:" + "\t" + "freqSearchHardLock" + "\n" +
                   "excldFreqs:" + "\t" + "[Inf, -Inf]" + "\n" +
                   "falseAlarmProb:" + "\t" + "0.00080" + "\n" +
                   "dataRecordPath:" + "\t" + str(data_bin_path) + "\n" +
                   "processedOuputPath:" + "\t" + str(output_directory_path) + "\n" +
                   "ros2enable:" + "\t" + "true" + "\n" +
                   "startInRunState:" + "\t" + "true" + "\n")
        file.close()
