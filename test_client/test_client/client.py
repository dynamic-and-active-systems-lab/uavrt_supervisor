#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/galactic/Tutorials/Writing-A-Simple-Py-Service-And-Client.html

This code is purely for testing purposes.
'''

import sys

from custom_interfaces.srv import *

import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import *


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(GetPose, '/getPose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPose.Request()

    def send_request(self):
        self.req.timestamp.sec = int(sys.argv[1])
        self.req.timestamp.nanosec = int(sys.argv[2])
        self.future = self.cli.call_async(self.req)


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                minimal_client.get_logger().info("Resulting interpolation: {}".format(
                    response.pose.position, response.pose.orientation))
                minimal_client.get_logger().info("Success")
            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
