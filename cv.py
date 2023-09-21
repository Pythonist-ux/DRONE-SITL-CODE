from pymavlink import mavutil
import time
import cv2
from mediapipe.python.solutions import hands
import math

# Initialize the camera
cap = cv2.VideoCapture(0)

# Start a connection listening on a TCP port
the_connection = mavutil.mavlink_connection('tcp:localhost:5762')

# Wait for the first heartbeat
the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))
mode_id = the_connection.mode_mapping()['GUIDED']

# Arm the drone
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Arm ACK: {ack_msg}")

# Set the mode to GUIDED
the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                                     0, mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id, 0, 0, 0, 0, 0)
ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print(f"Change Mode ACK: {ack_msg}")

# Initialize MediaPipe Hands
mp_hands = hands.Hands()

# Gesture recognition loop
thumbs_up_detected = False  # Initialize a flag for thumbs-up gesture detection

while True:
    ret, frame = cap.read()
    
    # Perform hand tracking using MediaPipe
    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = mp_hands.process(frame_rgb)

    if results.multi_hand_landmarks:
        # Get hand landmarks for the first detected hand (assuming one hand is present)
        landmarks = results.multi_hand_landmarks[0]
        
        # Get key landmarks
        thumb_tip = landmarks.landmark[hands.HandLandmark.THUMB_TIP]
        index_tip = landmarks.landmark[hands.HandLandmark.INDEX_FINGER_TIP]

        # Calculate the distance between thumb tip and index finger tip
        distance = math.sqrt((thumb_tip.x - index_tip.x)**2 + (thumb_tip.y - index_tip.y)**2)

        # Check if the distance is below a threshold (indicating thumbs-up)
        if distance < 0.08:  # Adjust the threshold as needed
            thumbs_up_detected = True
        else:
            thumbs_up_detected = False

    # Bring the drone down if thumbs-up gesture is detected
    if thumbs_up_detected:
        print("Thumbs-Up Gesture Detected (Descending)")
        
        # Descend the drone
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0, 0)
        ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Land ACK: {ack_msg}")

    cv2.imshow("Gesture Detection", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        # Disarm the drone when closing the detection window
        print("Disarming the drone")
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
        ack_msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
        print(f"Disarm ACK: {ack_msg}")
        break

cap.release()
cv2.destroyAllWindows()
