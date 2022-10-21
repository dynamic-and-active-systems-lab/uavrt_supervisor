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

# Import individual rclpy classes.
# https://docs.ros2.org/latest/api/rclpy/index.html
# ROS 2 testing library that provides insight into the functionlity of
# the different ROS 2 classes.
# https://github.com/ros2/rclpy/tree/master/rclpy/test
#
# https://docs.ros2.org/galactic/api/rclpy/api/init_shutdown.html
from rclpy import init
#
# https://docs.ros2.org/galactic/api/rclpy/api/execution_and_callbacks.html
from rclpy.executors import SingleThreadedExecutor
from rclpy.executors import ShutdownException
# https://docs.ros2.org/galactic/api/rclpy/api/logging.html
from rclpy.logging import get_logger

# When importing Python files that reside in the same workspace/src/package,
# you will need to specify the name of the package as well as the file.
#
# This Stackoverflow question/answer helped me figured out this issue:
# https://stackoverflow.com/a/58504978
from uavrt_supervisor.start_stop_component import StartStopComponent
from uavrt_supervisor.netcat_airspyhf_component import NetcatAirspyhfComponent
from uavrt_supervisor.airspyhf_channelize_component import AirspyfhChannelizeComponent
from uavrt_supervisor.detector_component import DetectorComponent

# NOTE: Only for debug purposes. Delete after.
from uavrt_supervisor.test_harness import TestHarness


def main(args=None):
    init(args=args)

    executor = SingleThreadedExecutor()

    start_stop_component = StartStopComponent()
    netcat_airspyhf_component = NetcatAirspyhfComponent()
    airspyhf_channelize_component = AirspyfhChannelizeComponent()
    detector_component = DetectorComponent()

    executor.add_node(start_stop_component)
    executor.add_node(netcat_airspyhf_component)
    executor.add_node(airspyhf_channelize_component)
    executor.add_node(detector_component)

    try:
        # Using "rclpy.logging.get_logger("[node_name]").[severity]("[msg]")"
        # This is a generic rclpy logger that does not need to be tied to any
        # individual node/component. It is useful within main.
        # logger is a rclpy.impl.rcutils_logger.RcutilsLogger object
        get_logger("main").info(
            "uavrt_supervisor package has been started.")
        get_logger("main").info("Press Ctrl+C to stop the program.")
        executor.spin()
    except KeyboardInterrupt as instance:
        get_logger("main").info("Keyboard interrupt recieved.")
        get_logger("main").info("Type: {}".format(type(instance)))
    except ShutdownException as instance:
        get_logger("main").info("Signal that executor was shut down recieved.")
        get_logger("main").info("Type: {}".format(type(instance)))
    finally:
        get_logger("main").info("uavrt_supervisor package is shutting down.")
        executor.shutdown()


if __name__ == '__main__':
    main()
