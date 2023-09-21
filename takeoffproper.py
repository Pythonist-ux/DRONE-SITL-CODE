from pymavlink import mavutil
import time
from time import sleep

# Start a connection listening on a TCP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
mode_id = the_connection.mode_mapping()['GUIDED']

# Arm the drone (correction)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Arm ACK: {ack_msg}")

# Set the mode to GUIDED (correction)
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Change Mode ACK: {ack_msg}")

# Takeoff to an altitude of 10 meters
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Takeoff ACK: {ack_msg}")



# Sleep for 8 seconds (correction)
time.sleep(8)

