from pymavlink import mavutil
import time

# Number of drones
num_drones = 3
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')
the_connection2 = mavutil.mavlink_connection('tcp:localhost:5772')
the_connection3 = mavutil.mavlink_connection('tcp:localhost:5782')
# Create a list to store connections to each drone
connections = [the_connection, the_connection2, the_connection3]

# Define waypoints for each drone
waypoints = [
    [(125, 75), (225, 95), (325, 115)],
    [(50, 100), (100, 200), (150, 300)],
    [(100, 45), (500, 55), (1000, 65)]
]

# Arm and takeoff each drone
for connection in connections:
    # Wait for the it_heartbeat()
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

#Send waypoints to each drone
for i, connection in enumerate(connections):
    for x, y in waypoints[i]:
        connection.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(
            10, connection.target_system, connection.target_component, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            3576, x, y, -25, 0, 0, 0, 0, 0, 0, 0, 10))
        print(f"Waypoint sent to drone {i + 1}: x={x}, y={y}")

# Sleep for 8 seconds
time.sleep(8)
