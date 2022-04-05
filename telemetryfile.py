#!/usr/bin/env python3

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
        self.roll = None # Roll angular velocity
        self.pitch = None # Pitch angular velocity
        self.yaw = None # Yaw angular velocity
        self.latitude = None # Latitude in degrees (range: -90 to +90)
        self.longitude = None # Longitude in degrees (range: -180 to +180)
        self.altitude = None # Altitude AMSL (above mean sea level) in metres

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
            
    async def setRoll(self):
        async for roll in self.drone.telemetry.attitude_angular_velocity_body():
            self.roll = roll.pitch_rad_s
            break
            
    async def setPitch(self):
        async for pitch in self.drone.telemetry.attitude_angular_velocity_body():
            self.pitch = pitch.roll_rad_s
            break
            
    async def setYaw(self):
        async for yaw in self.drone.telemetry.attitude_angular_velocity_body():
            self.yaw = yaw.yaw_rad_s
            break
    
    async def setLatitude(self):
        async for latitude in self.drone.telemetry.position():
            self.latitude = latitude.latitude_deg
            break
            
    async def setLongitude(self):
        async for longitude in self.drone.telemetry.position():
            self.longitude = longitude.longitude_deg
            break
            
    async def setAltitude(self):
        async for altitude in self.drone.telemetry.position():
            self.longitude = altitude.absolute_altitude_m
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
        convertedTime = time.strftime('%Y-%m-%d %H:%M:%S', time.gmtime(self.time))
    
        (self.buffer).write(convertedTime + "\t" + str(self.heading) + "\t" +
            str(self.pitch) + "\t" + str(self.roll) + "\t" +
            str(self.latitude) + "\t" + str(self.longitude) + "\t" +
            str(self.altitude) + "\n")
            
    async def writeToFile(self):
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
        
        return
        
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
        await telemetryFileObject.setTime()
        await telemetryFileObject.setHeading()
        await telemetryFileObject.setRoll()
        await telemetryFileObject.setPitch()
        await telemetryFileObject.setYaw()
        await telemetryFileObject.setLatitude()
        await telemetryFileObject.setLongitude()
        await telemetryFileObject.setAltitude()
        
        await telemetryFileObject.writeToBuffer()
        
        if (currentIteration == 60):
            await telemetryFileObject.writeToFile()
            currentIteration = 0
        
        await asyncio.sleep(.5)
        
        currentIteration += 1 
        
if __name__ == "__main__":
    asyncio.run(main())


# TODO: Change timing to be scheduled.
# loop.call_later(.5, lambda : telemetryFileObject.writeToBuffer())
# loop.call_later(30, lambda : telemetryFileObject.writeToFile())
#
# TODO: Comments.
# TODO: Upload to Github.
# TODO: Handle shutting down process gracefully.
# TODO: Add tests/exception handlers/error codes.
# TODO: Add exception to ensure all values inside the file.write statement are valid/expected.
# TODO: Change connection string back.

# NOTE: Telemetry file is currently being written current directory. Will need to be changed later.
