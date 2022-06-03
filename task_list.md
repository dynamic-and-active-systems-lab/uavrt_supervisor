# Task List for the Summer '22 Term 

### Phase 1:

#### 1. Build telemetry system on supervisor node

- [x] Receives telemetry data from Pixhawk at 2Hz
- [x] Publishes /vehiclePose topic to ros2 network as geometry_msg/PoseStamped message type
- [ ] Records received telemetry to file
- [ ] Provides /getPoseStamped service that provides the pose of the vehicle at the time sent as part of the request

Expected time to implement: 2 weeks
Actual time to implement: 3 weeks 

#### 2. Build transmission and receiving system on supervisor node

- [ ] Subscribes to the /pulsePose topic
  - [ ] Repackages /pulsePose messages as Mavlink debug messages and transmits to Pixhawk 

- [ ] Tramsmitting Pixhawk heartbeat messages to the Pixhawk

- [ ] Recieve debug messages via Mavlink 
  - [ ] Recieve commands via Mavlink 
    - [ ] Process commands 
  - [ ] Receives tag priori via Mavlink

- [ ] Builds file structure for archive
  - [ ] Writes detector config files

- [ ] Calculates radio settings (f_cent, n_channels)

Expected time to implement: 1-2 weeks 
Actual time to implement: 

Note: This is contingent on whether we have decided on Pymavlink or MAVSDK. 

#### 3. Build radio control system on supervisor node

- [ ] Start subprocesses using the [Python subprocess library](https://docs.python.org/3/library/subprocess.html) 
  - [ ] Start subprocesses from different paths for configuration files 
  - [ ] Start subprocesses over the local network using SSH 

- [ ] Start subprocesses 
  - [ ] Start channelizer process
  - [ ] Start airspy_rx -> netcat process 
  - [ ] Start detectors processes using configuration files

- [ ] Send Start/Stop/Pause/Unpause commands to subprocesses via UDP
  - [ ] Send commands to channelizer 
  - [ ] Send commands to detector 

Note: This is contingent on we decide to connect to remote systems.  

Expected time to implement: 1-2 weeks 
Actual time to implement: 

#### 4. Full demo of supervisor node (minimum number of processes and tags)

- [ ] Launch supervisor_node 
- [ ] Start up publishers/service(s)
  - [ ] Establish connection with Pixhawk/SITL
- [ ] Waits for command via Mavlink (Start/Stop/Pause/Unpause)
- [ ] On start
  - [ ] Builds file structure for archive
  - [ ] Receives tag priori via Mavlink
  - [ ] Calculates radio settings (f_cent, n_channels)
  - [ ] Writes detector config files
  - [ ] Idle start channelizer
  - [ ] Idle start all detectors
  - [ ] Start airspy_rx -> netcat process (channelizer must be running)
  - [ ] Send run command to all detectors 
  - [ ] Send run command to channelizer
  - [ ] Publishing radio system heartbeat messages via Mavlink
- [ ] On pause
  - [ ] Send pause command to channelizer
  - [ ] Send pause command to detector(s)
- [ ] On unpause
  - [ ] Send run command to channelizer
  - [ ] Send run command to detector(s)
- [ ] On stop
  - [ ] Send kill commands to all detectors and channelizer (order doesn’t matter)

Expected time to implement: 1 week
Actual time to implement: 

Note: Please suggest more requirements to satisfy for a complete test. 

5. Uncategorized

- [ ] Spend time optimizing and cleaning up current codebase prior to Phase 2

### Phase 2

TBD. I'd like to get more done with Phase 1 before laying out the tasks for Phase 2. 

# Meeting notes 

### June 3rd 2022 

- Demo 
- Are we using Pymavlink or MAVSDK? 
- Should I move to subprocesses or debug message receiving/transmitting? 
- [Python subprocess library](https://docs.python.org/3/library/subprocess.html) 
- [Python subprocess remote start/login](https://programmer.group/experience-sharing-the-best-practice-of-remote-login-server-with-python.html)
  - The subprocess library supports opening up ssh connections to other machines on the network in order to start processes on those machines 
  - This would only work if the machine that is being ssh'ed into has the required dependices installed, but this is a reasonable requirement 
  - Output from the remote process can be sent back to the local parent process
  - Note: I could not find better documentation of this feature than the link above;
    - I'm certain that it will work, but there is limited documentation on the functionality despite being supported by an official python library 
  - Biggest concern: We could never have a distributed system unless we have an established network. How else would you start a remote subprocess? 
 - Meeting with Don? 
 - Paul: Can we meet for 5 minutes after today's meeting? 
