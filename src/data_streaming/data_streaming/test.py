

from pymavlink import mavutil


# create a mavlink serial instance
#master = mavutil.mavlink_connection('udp:0.0.0.0:14540')
master = mavutil.mavlink_connection('/dev/ttyACM0')

# wait for the heartbeat msg to find the system ID
print("Waiting for APM heartbeat")

heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
if heartbeat != None:
    print("Heartbeat from APM (system %u component %u)" %
          (master.target_system, master.target_component))

    for i in range(15):
        msg = master.recv_match(blocking=True)
        print(msg)

    longitude = master.messages['GLOBAL_POSITION_INT'].lat
else:
    print("No heartbeat")
