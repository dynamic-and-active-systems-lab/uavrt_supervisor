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


class TestHarness(Node):

    def __init__(self):
        super().__init__('TestHarness')

        # Queue size is a required QoS (quality of service) setting that limits the
        # amount of queued messages if a subscriber is not receiving them fast enough.
        self._queue_size_ = 10

        # Status publisher
        self._initialize_test_harness_netcat_airspyhf_component_control_publisher()

    def _initialize_test_harness_netcat_airspyhf_component_control_publisher(self):
        # Format: Msg type, topic, queue size
        self._test_harness_netcat_airspyhf_component_control_publisher = self.create_publisher(
            DiagnosticArray,
            'control_netcat_airspyhf',
            self._queue_size_)
        self.get_logger().info(
            "Test harness is now publishing to the 'control_netcat_airspyhf' topic.")

        _status_array = DiagnosticArray()
        _status = DiagnosticStatus()
        _center_frequency_value = KeyValue()
        _sample_rate_value = KeyValue()

        _status_array.header.frame_id = "control"
        _status_array.header.stamp = self.get_clock().now().to_msg()

        _status.level = b'0'
        _status.name = "netcat_airspyhf_component"
        _status.message = "start"
        _status.hardware_id = "1"

        _center_frequency_value.key = "center_frequency"
        _center_frequency_value.value = "91.7"

        _sample_rate_value.key = "sample_rate"
        _sample_rate_value.value = "912000"

        _status.values.append(_center_frequency_value)
        _status.values.append(_sample_rate_value)

        _status_array.status.append(_status)

        self._test_harness_netcat_airspyhf_component_control_publisher.publish(_status_array)
