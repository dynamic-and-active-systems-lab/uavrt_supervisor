#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html
'''

from supervisor.mavlink_utilities import *

# DELETE
from supervisor.supervisor_servicers import *

from std_msgs.msg import *
from geometry_msgs.msg import *
from diagnostic_msgs.msg import *

import numpy as np

# Heartbeat unique string for heartbeatStatus messages.
# Hex(heartbeat)
HEARTBEAT_ID = '776f7264'
# Allow number of missed heartbeats
MISSED_HEARTBEAT_LIMIT = 5


def heartbeatMonitor(supervisorNode):
    logger = supervisorNode.get_logger()

    statusArray = DiagnosticArray()
    status = DiagnosticStatus()
    value = KeyValue()

    currentStatus = checkHeartbeat(supervisorNode.connection, logger)

    statusArray.header.frame_id = "heartbeatStatus"
    statusArray.header.stamp = supervisorNode.get_clock().now().to_msg()

    status.name = 'Heartbeat'
    status.hardware_id = HEARTBEAT_ID

    value.key = 'Number of missed heartbeats'

    if currentStatus != None:
        supervisorNode.heartbeatWatchdog = MISSED_HEARTBEAT_LIMIT
        status.level = b'0'
        status.message = 'Alive'
    elif currentStatus == None and supervisorNode.heartbeatWatchdog > 0:
        supervisorNode.heartbeatWatchdog -= 1
        status.level = b'1'
        status.message = 'Unknown'
    elif currentStatus == None and supervisorNode.heartbeatWatchdog == 0:
        status.level = b'2'
        status.message = 'Dead'

    value.value = ''.format(MISSED_HEARTBEAT_LIMIT -
                            supervisorNode.heartbeatWatchdog)

    logger.info("Current heartbeat status: {}".format(status.message))

    status.values.append(value)
    statusArray.status.append(status)

    supervisorNode.heartbeatPublisher.publish(statusArray)

    if status.message == 'Dead':
        logger.info("Attempting to reestablish connection.")
        supervisorNode.connection = establishMavlinkConnection(supervisorNode)


def telemetryMonitor(supervisorNode):
    logger = supervisorNode.get_logger()

    if supervisorNode.connection != None and checkGPS(supervisorNode) != None:
        header = Header()
        poseStamped = PoseStamped()
        pose = Pose()
        position = Point()
        orientation = Quaternion()

        # We get the current time seperately since the value that is stored
        # into the Node's currentTimeArray array needs to be in nanoseconds for
        # interpolation
        currentTime = supervisorNode.get_clock().now()

        header.frame_id = "telemetryData"
        header.stamp = currentTime.to_msg()

        position.x = float(getLongitude(supervisorNode))
        position.y = float(getLatitude(supervisorNode))
        position.z = float(getAltitude(supervisorNode))

        orientation.x = float(getLongitude(supervisorNode))
        orientation.y = float(getLatitude(supervisorNode))
        orientation.z = float(getAltitude(supervisorNode))
        orientation.w = float(getAltitude(supervisorNode))

        pose.position = position
        pose.orientation = orientation

        poseStamped.header = header
        poseStamped.pose = pose

        supervisorNode.telemetryPublisher.publish(poseStamped)

        logger.info("Telemetry data has been successfully published.")

        supervisorNode.currentTimeArray = np.append(
            supervisorNode.currentTimeArray, currentTime.nanoseconds)

        supervisorNode.positionArray = np.append(supervisorNode.positionArray,
                                                 [[position.x], [position.y],
                                                     [position.z]],
                                                 axis=1)

        supervisorNode.orientationArray = np.append(supervisorNode.orientationArray,
                                                    [[orientation.x], [orientation.y], [
                                                        orientation.z], [orientation.w]],
                                                    axis=1)

        logger.info("Telemetry data has been appended to telemetry arrays.")

        # DELETE 
        # searchTelemetryArrays(supervisorNode, currentTime.to_msg(), pose)

    else:
        logger.warn("Unable to publish telemetry data!")
        logger.warn("Connection or GPS lock was not established!")
