#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-A-Bag-From-Your-Own-Node-Python.html
https://github.com/ros2/rosbag2/tree/master/rosbag2_py/test
'''

# https://docs.ros.org/en/humble/Tutorials/Ros2bag/Recording-A-Bag-From-Your-Own-Node-Python.html
# https://github.com/ros2/rosbag2/blob/master/rosbag2_py/test/test_sequential_writer.py
import rosbag2_py

# We need to serialize/deserialize the data before writing/reading it to/from
# the Telemetry bag
from rclpy.serialization import serialize_message
from rclpy.serialization import deserialize_message

# For converting ROS_TIME to datetime
# https://docs.python.org/3/library/datetime.html
# http://wiki.ros.org/rospy/Overview/Time
# https://design.ros2.org/articles/clock_and_time.html#ros-time-source
#
# https://answers.ros.org/question/286602/convience-methods-to-convert-rospytime-to-datetimedatetime/?answer=365276#post-id-365276
# rospy.Time to datetime.datetime: datetime.utcfromtimestamp(T.to_sec())
# datetime.datetime to rospy.Time: rospy.Time.from_sec(T.replace(tzinfo=datetime.timezone.utc).timestamp())
from datetime import datetime as dt

# One billionth (10^-9) of a second
# For converting nanoseconds
# https://stackoverflow.com/a/15650033
NANOSECOND = 1000000000


def createTelemetryBagWriter(supervisorNode):
    logger = supervisorNode.get_logger()

    supervisorNode.telemetryBagWriter = rosbag2_py.SequentialWriter()

    # TODO: A ROS2 parameter could be supplied to a launch file and used here
    # as a file name/directory for dynamic naming and saving of files.
    # This really needs to be changed, as the directory name is equal to the
    # file name. When opening up a reader, you need to pass in the directory
    # name, append the directory name onto that, and then append '_0.db3'
    supervisorNode.telemetryDirectoryName = '{}'.format(
        dt.utcfromtimestamp(supervisorNode.get_clock().now().nanoseconds / NANOSECOND))

    # The data is stored in a SQLite database file
    # Note: There are different storage options
    storage_options = rosbag2_py._storage.StorageOptions(
        uri=supervisorNode.telemetryDirectoryName,
        storage_id='sqlite3')
    # The default conversion options are used, which will perform no
    # conversion and store the messages in the serialization format they
    # are received in.
    converter_options = rosbag2_py._storage.ConverterOptions('', '')

    try:
        supervisorNode.telemetryBagWriter.open(
            storage_options, converter_options)
    # Happens if the directory already exists
    except RuntimeError as instance:
        logger.warn("Unable to create telemetry bag reader!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

    # TopicMetadata object registered to 'telemetry_bag' writer
    # This object specifies the topic name, topic data type, and
    # serialization format used.
    # The messages' data is stored/serialized in binary format (CDR)
    # Note: There are different serialization formats
    topic_info = rosbag2_py._storage.TopicMetadata(
        name='/antennaPose',
        type='geometry_msgs/msg/PoseStamped',
        serialization_format='cdr')
    supervisorNode.telemetryBagWriter.create_topic(topic_info)

    logger.info("Telemetry bag writer has been successfully created.")


def recordTelemetryData(supervisorNode, msg):
    # Topic name, serialize_message(msg), timestamp
    # Expected romat: str, PoseStamped() object, int
    supervisorNode.telemetryBagWriter.write(
        '/antennaPose',
        serialize_message(msg),
        supervisorNode.get_clock().now().nanoseconds)


def createTelemetryBagReader(supervisorNode):
    logger = supervisorNode.get_logger()

    # Note: This reader will only read from supervisorNode.filename!
    # If you need to read from another telemetry file, you either need to
    # make a new function or have a bag that can read from multiple files.
    supervisorNode.telemetryBagReader = rosbag2_py.SequentialReader()

    # Note: AGAIN this works, but it needs to be changed
    finalName = "{}/{}_0.db3".format(supervisorNode.telemetryDirectoryName,
                                     supervisorNode.telemetryDirectoryName)

    storage_options = rosbag2_py._storage.StorageOptions(
        uri=finalName,
        storage_id='sqlite3')
    converter_options = rosbag2_py._storage.ConverterOptions('', '')

    try:
        supervisorNode.telemetryBagReader.open(
            storage_options, converter_options)
    # Happens if the file could not be opened
    except RuntimeError as instance:
        logger.warn("Unable to create telemetry bag reader!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

    logger.info("Telemetry bag reader has been successfully created.")

    # We expect only PoseStamped in this file, so it is not necessary to filter.
    # However, if need be, this is how you would do it:
    # https://github.com/ros2/rosbag2/blob/d97e2911e074ec6ab5fd23a546b0ad04950b6750/rosbag2_py/test/test_sequential_reader.py#L44


# TODO: Finish
def telemetryBagToFile(supervisorNode):
    logger = supervisorNode.get_logger()

    reader = supervisorNode.telemetryBagReader

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        msg_type = get_message(topic)
        msg = deserialize_message(data, msg_type)

# TODO: Finish
def searchTelemetryBag(supervisorNode):
    logger = supervisorNode.get_logger()

    reader = supervisorNode.telemetryBagReader

    (topic, data, t) = reader.seek()
    msg_type = get_message(topic)
    msg = deserialize_message(data, msg_type)
