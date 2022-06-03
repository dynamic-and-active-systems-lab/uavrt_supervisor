# Task List for the Summer '22 Term 

### Phase 1:

1. Build telemetry system on supervisor node

- [x] Receives telemetry data from Pixhawk at 2Hz
- [x] Publishes /vehiclePose topic to ros2 network as geometry_msg/PoseStamped message type
- [ ] Records received telemetry to file
- [ ] Provides /getPoseStamped service that provides the pose of the vehicle at the time sent as part of the request

2. Build radio system on supervisor node

- [ ] Waits for command via ML (Start/Stop/Pause/Unpause)
- [ ] On start
  - [ ] Builds file structure for archive
  - [ ] Receives tag priori via ML
  - [ ] Calculates radio settings (f_cent, n_channels)
  - [ ] Writes detector config files
  - [ ] Idle start channelizer
  - [ ] Idle start all detectors
  - [ ] Start airspy_rx -> netcat process (channelizer must be running)
  - [ ] Send run command to all detectors 
  - [ ] Send run command to channelizer
  - [ ] Publishing radio system heartbeat messages via ML
- [ ] On pause
  - [ ] Send pause command to channelizer
- [ ] On unpause
  - [ ] Send run command to channelizer
- [ ] On stop
  - [ ]  Send kill commands to all detectors and channelizer (order doesn’t matter)

3. Build transmission node

- [ ] Subscribes to the /pulsePose topic
- [ ] Repackages /pulsePose messages as ML debug messages and transmits to Pixhawk 

### Phase 2



## Revisions 

- The mavlink2ros and ros2mavlink nodes were merged into the supervisor node

# Meeting notes 

### June 3rd 2022 

- Are we using pymavlink or mavsdk? 
- [Python subprocess library](https://docs.python.org/3/library/subprocess.html) 
- [Python subprocess remote start/login](https://programmer.group/experience-sharing-the-best-practice-of-remote-login-server-with-python.html)
  - The subprocess library supports opening up ssh connections to other machines in order to start processes on those machines 
  - This would only work if the machine that is being ssh'ed into has the required dependices installed, but this is a reasonable requirement 
  - Output from the remote process can be sent back to the local parent process
  - Note: I could not find better documentation of this feature than the link above;
    - I'm certain that it will work, but there is limited documentation on the functionality despite being supported by an official python library 
 - Meeting with Don? 
