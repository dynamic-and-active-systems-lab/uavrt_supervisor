# Task List for the Summer '22 Term 

### Phase 1:

1. Build telemetry system on supervisor node

- [x] Receives telemetry data from Pixhawk at 2Hz
- [x] Publishes /vehiclePose topic to ros2 network as geometry_msg/PoseStamped message type
- [ ] Records received telemetry to file
- [ ] Provides /getPoseStamped service that provides the pose of the vehicle at the time sent as part of the request

Expected time to implement: 2 weeks
Actual time to implement: 3 weeks 

2. Build transmission system on supervisor node

- [ ] Subscribes to the /pulsePose topic
- [ ] Repackages /pulsePose messages as Mavlink debug messages and transmits to Pixhawk 

- [ ] Recieve debug messages from 

- [ ] Spend time optimizing and cleaning up current codebase 

Expected time to implement: 1 week 
Actual time to implement: 

Note: This is purely contingent on whether we have decided on pymavlink or mavsdk. 

3. Build radio system on supervisor node

- [ ] Start subprocesses using the [Python subprocess library](https://docs.python.org/3/library/subprocess.html) 
  - [ ] Start subprocesses from different paths for configuration files 
  - [ ] Start subprocesses over the local network using SSH 

- [ ] Start subprocesses 
  - [ ] Start channelizer process
  - [ ] Start airspy_rx -> netcat process (channelizer must be running)

- [ ] Send Start/Stop/Pause/Unpause commands to subprocesses via UDP
  - [ ] Send commands to channelizer 
  - [ ] Send commands to detector 

- [ ] R

- [ ] Waits for command via ML ()
- [ ] On start
  - [ ] Builds file structure for archive
  - [ ] Receives tag priori via ML
  - [ ] Calculates radio settings (f_cent, n_channels)
  - [ ] Writes detector config files
  - [ ] Idle start channelizer
  - [ ] Idle start all detectors

  - [ ] Send run command to all detectors 
  - [ ] Send run command to channelizer
  - [ ] Publishing radio system heartbeat messages via ML
- [ ] On pause
  - [ ] Send pause command to channelizer
- [ ] On unpause
  - [ ] Send run command to channelizer
- [ ] On stop
  - [ ]  Send kill commands to all detectors and channelizer (order doesn’t matter)

Expected time to implement: ? weeks 
Actual time to implement: 

### Phase 2

TBD. I'd like to get more done with Phase 1 before laying out the tasks for Phase 2. 

## Revisions 

- The mavlink2ros and ros2mavlink nodes were merged into the supervisor node
- I think the second step of Phase 1 should be figuring out the debug messages 

# Meeting notes 

### June 3rd 2022 

- Demo 
- Are we using pymavlink or mavsdk? 
- Should I move to subprocesses or debug message receiving/transmitting? 
- [Python subprocess library](https://docs.python.org/3/library/subprocess.html) 
- [Python subprocess remote start/login](https://programmer.group/experience-sharing-the-best-practice-of-remote-login-server-with-python.html)
  - The subprocess library supports opening up ssh connections to other machines in order to start processes on those machines 
  - This would only work if the machine that is being ssh'ed into has the required dependices installed, but this is a reasonable requirement 
  - Output from the remote process can be sent back to the local parent process
  - Note: I could not find better documentation of this feature than the link above;
    - I'm certain that it will work, but there is limited documentation on the functionality despite being supported by an official python library 
 - Meeting with Don? 
 - Paul: Can I talk to you for 5 minutes after the meeting? It's in regard to my CS 685 course and paper topic for my thesis. 
