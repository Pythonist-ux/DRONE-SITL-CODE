from pymavlink import mavutil
import time

# Number of drones
num_drones = 3
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
the_connection2 = mavutil.mavlink_connection('tcp:localhost:5772')
the_connection3 = mavutil.mavlink_connection('tcp:localhost:5782')
# Create a list to store connections to each drone
connections = [the_connection,the_connection2,the_connection3]
x=125
y=75
# Create connections for each drone and store them in the 'connections' list


# Arm and takeoff each drone
for connection in connections:
    # Wait for the first heartbeat
    connection.wait_heartbeat()
    print(f"Heartbeat from system  (system {connection.target_system} component {connection.target_component})")

    mode_id = connection.mode_mapping()['GUIDED']

    # Arm the drone
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Arm ACK for drone : {ack_msg}")

    # Set the mode to GUIDED
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                     mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
    ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Change Mode ACK for drone : {ack_msg}")

# Takeoff all drones to an altitude of 10 meters
for connection in connections:
    connection.mav.command_long_send(connection.target_system, connection.target_component,
                                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    ack_msg = connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
    print(f"Takeoff ACK for drone : {ack_msg}")
    

# Sleep for 8 seconds
time.sleep(8)