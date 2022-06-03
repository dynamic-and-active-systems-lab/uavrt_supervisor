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

3. Build ros2mavlink node

- [ ] Subscribes to the /pulsePose topic
- [ ] Repackages /pulsePose messages as ML debug messages and transmits to Pixhawk 

## Revisions 

- The mavlink2ros and ros2mavlink nodes were merged into the supervisor node

# Meeting notes 

### June 3rd 2022 

- [Python subprocess library](https://docs.python.org/3/library/subprocess.html) supports 
