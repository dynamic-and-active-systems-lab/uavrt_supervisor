#!/usr/bin/env python3

'''
Based on the example code from these sites:
https://mavlink.io/en/mavgen_python/
https://www.ardusub.com/developers/pymavlink.html
https://github.com/ArduPilot/pymavlink/tree/master/examples
'''

# Information on mavutil.py:
# https://github.com/ArduPilot/pymavlink/blob/master/mavutil.py
from pymavlink import mavutil

# Information on pathlib module and Path:
# https://docs.python.org/3/library/pathlib.html
from pathlib import Path

# Necessary for catching SerialException
# https://github.com/pyserial/pyserial/blob/master/serial/serialutil.py
from serial.serialutil import SerialException

# Python doesn’t have built-in constant types.
# By convention, Python uses a variable whose name contains all capital letters
# to indicate that a variable is a constant.
SERIAL_PATH = '/dev/ttyACM0'
UDP_IP_PORT = 'udp:0.0.0.0:14540'

# MAVLink API listening for SITL connection via UDP:
# (using 0.0.0.0 for when IP is unknown)
# 	udp:0.0.0.0:14540
# Linux computer connected to the vehicle via USB:
#	/dev/ttyACM0
# For info on these connections and udp/tcp with pymavlink:
# https://mavlink.io/en/mavgen_python/#setting_up_connection
# NOTE: Must install pyserial to use a serial connection
# 	python3 -m pip install pyserial
#
# First I check if the serial connection is alive.
# If not, I check if the UDP connection is alive.
# I do it in this order since it is more difficult to scan for an open
# UDP port than it is to check for a valid serial file descriptor.


def establishMavlinkConnection(logger):

    # Attempt to establish a serial connection with a Pixhawk
    # This sets the system and component ID of remote system for the serial link
    #
    # Information on handling exceptions:
    # https://docs.python.org/3/tutorial/errors.html#handling-exceptions
    #
    # Information on Path.resovle(string=T/F) functionality:
    # https://docs.python.org/3/library/pathlib.html#pathlib.Path.resolve
    try:
        Path(SERIAL_PATH).resolve(strict=True)

        # The serial connection should be OK but we check for a heartbeat to be safe
        connection = mavutil.mavlink_connection(SERIAL_PATH)
        if checkHeartbeat(connection, logger) != None:
            logger.info("Pixhawk is connected via serial!")
            logger.info("Target system: {}".format(connection.target_system))
            logger.info("Target component: {}".format(
                connection.target_component))
            return connection
        else:
            logger.warn(
                "Pixhawk is connected via serial but is NOT recieving heartbeat messages!")
            return None
    except FileNotFoundError as instance:
        logger.warn("Pixhawk is not connected via serial!")
        # the exception instance
        # __str__ allows args to be printed directly, but may be overridden in
        # exception subclasses
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
    except SerialException as instance:
        logger.warn("Pixhawk has been disconnected or reconnected!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

    # Attempt to establish an UDP connection with a Pix4 SITL (e.g. Gazebo)
    # This sets the system and component ID of remote system for the UDP link
    connection = mavutil.mavlink_connection(UDP_IP_PORT)
    if checkHeartbeat(connection, logger) != None:
        logger.info("Pixhawk is connected via UDP!")
        return connection
    else:
        logger.warn("Pixhawk is not connected via UDP!")

    # If we get here, then both connections have failed
    logger.warn("Pixhawk is not connected via serial nor UDP!")
    return None


def checkHeartbeat(connection, logger):
    # Check for a heartbeat, it should be every 1 Hz
    # This function is blocking
    # Timeout is in seconds
    try:
        heartbeat = connection.recv_match(
            type='HEARTBEAT', blocking=True, timeout=1)

        if heartbeat != None:
            logger.info("Heartbeat was recieved!")
            return heartbeat
        else:
            logger.warn("Heartbeat not recieved!")
            return None
    except AttributeError as instance:
        logger.warn("Connection does not exist!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None
    except SerialException as instance:
        logger.warn("Pixhawk has been disconnected or reconnected!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None


def checkGPS(connection, logger):
    # Check if there is GLOBAL_POSITION_INT present in the MAVLink data
    try:
        gps = connection.messages['GLOBAL_POSITION_INT']
        logger.info("GPS satellite(s) locked.")
        return gps
    except KeyError as instance:
        logger.warn("No GPS satellite(s) found!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None


# If you just want to synchronously access the last message of a
# particular type that was received (and when it was received)
# you can do so using the connection's mavutil.messages dictionary.
# https://mavlink.io/en/mavgen_python/#receiving-messages
#
# MAVLink message types are defined here:
# https://mavlink.io/en/messages/common.html
# We use GLOBAL_POSITION_INT ( #33 ) for lon, lat, alt
# and ATTITUDE_QUATERNION ( #31 ) for quaternion data (x, y, z, w)
#
# TODO: How do I catch BAD_DATA? Should I make a seperate except clause for it?
def getLongitude(connection, logger):
    try:
        longitude = connection.messages['GLOBAL_POSITION_INT'].lon
        return longitude
    except KeyError as instance:
        logger.warn("Unable to retrieve longitude!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None


def getLatitude(connection, logger):
    try:
        latitude = connection.messages['GLOBAL_POSITION_INT'].lat
        return latitude
    except KeyError as instance:
        logger.warn("Unable to retrieve latitude!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None


def getAltitude(connection, logger):
    try:
        altitude = connection.messages['GLOBAL_POSITION_INT'].alt
        return altitude
    except KeyError as instance:
        logger.warn("Unable to retrieve altitude!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

def getQuaternionX(connection, logger):
    try:
        quaternionX = connection.messages['ATTITUDE_QUATERNION'].q2
        return quaternionX
    except KeyError as instance:
        logger.warn("Unable to retrieve quaternion X!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

def getQuaternionY(connection, logger):
    try:
        quaternionY = connection.messages['ATTITUDE_QUATERNION'].q3
        return quaternionY
    except KeyError as instance:
        logger.warn("Unable to retrieve quaternion Y!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

def getQuaternionZ(connection, logger):
    try:
        quaternionZ = connection.messages['ATTITUDE_QUATERNION'].q4
        return quaternionZ
    except KeyError as instance:
        logger.warn("Unable to retrieve quaternion Z!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None

def getQuaternionW(connection, logger):
    try:
        quaternionW = connection.messages['ATTITUDE_QUATERNION'].q1
        return quaternionW
    except KeyError as instance:
        logger.warn("Unable to retrieve quaternion W!")
        logger.warn("Type: {}".format(type(instance)))
        logger.warn("Instance: {}".format(instance))
        return None
