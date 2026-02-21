# ArduPilot flight of ArduCopter SITL simulation by watpoints in AUTO mode

from pymavlink import mavutil
import time
import math

# Define flight mode
def set_mode(mode):
    # Get mode ID
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )
    while True:
        ack = master.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            print(f"Mode changed to {mode}")
            break
        time.sleep(0.5)

# Arm vehicle
def arm():
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Vehicle armed")

# Takeoff
def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude
    )
    print(f"Takeoff command sent to {altitude} m")

    # Wait until vehicle reaches of target altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.1f} m  ", end="\r", flush=True)

        if current_alt >= altitude:
            print("")
            print("Reached takeoff altitude")
            return
        
        time.sleep(0.1)

# Land
def land():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("\nLanding...")

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.1f} m  ", end="\r", flush=True)

        if current_alt <= 1:
            print("Landed                ")
            return
        
        time.sleep(0.1)

# Waypoint file parser
def load_qgc_waypoints(filename):
    waypoints = []

    with open(filename, 'r') as f:
        lines = f.readlines()

    if not lines[0].startswith("QGC WPL"):
        raise ValueError("Invalid QGC waypoint file")

    for line in lines[1:]:
        parts = line.strip().split('\t')
        if len(parts) < 12:
            continue

        lat = float(parts[8])
        lon = float(parts[9])
        alt = float(parts[10])

        waypoints.append((lat, lon, alt))

    return waypoints

def send_mission(waypoints):
    # 1. Clear existing mission
    master.mav.mission_clear_all_send(master.target_system, master.target_component)
    
    # 2. Send the number of waypoints
    count = len(waypoints)
    master.mav.mission_count_send(master.target_system, master.target_component, count)
    print(f"Notifying Autopilot of {count} waypoints...")

    # 3. Handle the "Request-Response" loop
    for i in range(count):
        # Wait for the specific request for this waypoint index
        msg = master.recv_match(type=['MISSION_REQUEST', 'MISSION_REQUEST_INT'], blocking=True, timeout=5)
        if not msg:
            print("Upload failed: No request received from Autopilot")
            return
            
        lat, lon, alt = waypoints[msg.seq]
        
        # Send MISSION_ITEM_INT (more precise than MISSION_ITEM)
        master.mav.mission_item_int_send(
            master.target_system,
            master.target_component,
            msg.seq,                     # Use the sequence the AP requested
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, 0, 0,            # Params 1-4 (unused here)
            int(lat * 1e7),
            int(lon * 1e7),
            float(alt)
        )
        print(f"Sent waypoint {msg.seq}")

    # 4. Wait for final acknowledgement
    ack = master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
    if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
        print("Mission upload successful!")
    else:
        print(f"Mission upload failed with ACK type: {ack.type if ack else 'Timeout'}")

# Connect via MAVProxy UDP port (default 14550)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to MAVProxy/SITL")

# Load waypoints from a TXT file
waypoints = load_qgc_waypoints("mission_waypoints.txt")

# Takeoff in GUIDED mode
set_mode('GUIDED')
arm()
takeoff(10)

# Fly in AUTO mode
send_mission(waypoints) 
set_mode('AUTO')          # start following waypoints

# Wait until mission complete and print the progress
while True:
    msg = master.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    print(f"Reached waypoint {msg.seq}")
    if msg.seq == len(waypoints) - 1:
        print("All waypoints reached")
        break

land()
