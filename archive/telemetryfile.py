#!/usr/bin/env python3

# MAVSDK-Python: https://mavsdk.mavlink.io/main/en/python/
from mavsdk import System

import time
import platform

# Information regarding asyncio in Python:
# https://docs.python.org/3/library/asyncio.html
import asyncio

# Needed for signal handlers: SIGINT and SIGTERM
# https://docs.python.org/3/library/asyncio-eventloop.html#set-signal-handlers-for-sigint-and-sigterm
import functools
import os
import signal

# Needed for creating a string buffer:
# https://docs.python.org/3.9/library/stdtypes.html#common-sequence-operations
from io import StringIO

"""
The TelemetryFile class is used for collecting data
from the PixHawk and writing the data to a text document.
"""
class TelemetryFile(object):
    def __init__(self):
        self.platformOS = None # OS connection string
        self.drone = None # Drone object
        
        self.fileName = None # Telemetry file name
        self.buffer = StringIO() # String buffer
        
        self.time = None # POSIX time
        self.heading = None # Heading in degrees (range: 0 to +360)
        self.roll = None # Roll angular velocity (AVB)
        self.pitch = None # Pitch angular velocity (AVB)
        self.yaw = None # Yaw angular velocity (AVB)
        self.latitude = None # Latitude in degrees (range: -90 to +90) (Position)
        self.longitude = None # Longitude in degrees (range: -180 to +180) (Position)
        self.altitude = None # Altitude AMSL (above mean sea level) in metres (Position)

    async def connectToPixhawk(self):
        # Initialize the drone
        drone = System("0.0.0.0", "50051")
        await drone.connect(system_address="udp://14540") # TODO: change back
        print("Waiting for drone to connect...")
        
        async for state in drone.core.connection_state():
            if state.is_connected:
                print(f"Drone discovered!")
            break
            
        self.drone = drone
        
    async def setTime(self):
        self.time = time.time();

    async def setHeading(self):
        async for heading in self.drone.telemetry.heading():
            self.heading = heading.heading_deg
            break
            
    async def attitudeAVB(self):
        async for attitude in self.drone.telemetry.attitude_angular_velocity_body():
            self.roll = attitude.pitch_rad_s
            self.pitch = attitude.roll_rad_s
            self.yaw = attitude.yaw_rad_s
            break
    
    async def setPosition(self):
        async for position in self.drone.telemetry.position():
            self.latitude = position.latitude_deg
            self.longitude = position.longitude_deg
            self.altitude = position.absolute_altitude_m
            break
            
    async def createFile(self):
        # Converting POSIX time to UTC in the format of year, month, day, hour,
        # minute, second
        convertedTime = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(self.time))
        
        self.fileName = "Telemetry_File_" + convertedTime + ".txt"
        file = open(self.fileName, "w")
        file.write("Time_POSIX" + "\t""Heading_deg" + "\t" + "Pitch_rad" + "\t" +
                "Roll_rad" + "\t" + "Yaw_rad" + "\t" + "Lat" + "\t" +
                "Long" + "\t" + "Alt_m_AGL" + "\n")
        file.close()
        
    # Note: this not the fastest way of writing to a buffer nor the slowest.
    # However, it is easy to read and understand what is happening.
    async def writeToBuffer(self):
        # TODO: line below is temp
        convertedTime = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(self.time))
    
        (self.buffer).write(convertedTime + "\t" + str(self.heading) + "\t" +
            str(self.pitch) + "\t" + str(self.roll) + "\t" +
            str(self.latitude) + "\t" + str(self.longitude) + "\t" +
            str(self.altitude) + "\n")
            
    async def writeToFile(self):
        # TODO: line below is temp
        convertedTime = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(self.time))
        print(convertedTime)
        
        file = open(self.fileName, "a")
        file.write((self.buffer).getvalue())
        file.close()
        
        # Clear the buffer
        (self.buffer).seek(0)
        (self.buffer).truncate(0)
            
    """
    Check the OS to see what connection string is used in order to talk to
    the Pixhawk.

    Windows is currently not supported.
    """
    def setPlatform(self):
        hostPlatform = platform.system()
        
        if hostPlatform == "Darwin":
            # MacOS
            connectionString = "serial:///dev/tty.usbmodem01"
        else:
            # Linux
            connectionString = "serial:///dev/ttyACM0"
        
    def askExit(self, signame, loop):
        print("Caught signal %s, now exiting" % signame)
        loop.stop()
        
async def main():
    loop = asyncio.get_running_loop()
    
    telemetryFileObject = TelemetryFile()
    await telemetryFileObject.connectToPixhawk()
    await telemetryFileObject.createFile()
    
    for signame in {'SIGINT', 'SIGTERM'}:
        loop.add_signal_handler(
            getattr(signal, signame),
            functools.partial(telemetryFileObject.askExit, signame, loop))
            
    print("Event loop running, press Ctrl+C to interrupt.")
    print(f"pid {os.getpid()}: send SIGINT or SIGTERM to exit.")
    
    currentIteration = 0
    
    while True:
        await asyncio.gather(
                telemetryFileObject.setTime(),
                telemetryFileObject.setHeading(),
                telemetryFileObject.attitudeAVB(),
                telemetryFileObject.setPosition(),
        )

        await telemetryFileObject.writeToBuffer()
        
        if (currentIteration == 60):
            await telemetryFileObject.writeToFile()
            currentIteration = 0
        
        await asyncio.sleep(.5)
        
        currentIteration += 1 
        
if __name__ == "__main__":
    # asyncio debug mode: https://docs.python.org/dev/library/asyncio-dev.html#
    asyncio.run(main(), debug=True)


# TODO: Comments.
# TODO: Upload to Github.
# TODO: Handle shutting down process gracefully from Supervisor.
# TODO: Add tests/exception handlers/error codes.
# TODO: Most functions in MAVSDK-Python can raise exceptions that your code should handle with try... except
# TODO: Add exception to ensure all values inside the file.write statement are valid/expected.
# TODO: Pass exception error codes up pipeline to be processed by Supervisor.
# TODO: Change connection string back.
#
# NOTE: Telemetry file is currently being written current directory. Will need to be changed later.
# NOTE: Can set .gather function call to a result as a list. This list could be used to identify error codes.
