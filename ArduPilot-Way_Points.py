# ArduPilot flight of ArduCopter SITL simulation by watpoints in GUIDED mode

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

# Way-points ------------------------------------------------------------------------------

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

# Fly to a single waypoint
def goto_waypoint(lat, lon, alt, tolerance=1.5):
    print(f"Going to waypoint: {lat}, {lon}, {alt} m")

    while True:
        master.mav.set_position_target_global_int_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        cur_lat = msg.lat / 1e7
        cur_lon = msg.lon / 1e7
        cur_alt = msg.relative_alt / 1000.0

        # Distance estimation (meters)
        dlat = (lat - cur_lat) * 111139
        dlon = (lon - cur_lon) * 111139
        dist = math.sqrt(dlat**2 + dlon**2)

        print(f"Distance to WP: {dist:.2f} m  ", end="\r", flush=True)

        if dist <= tolerance and abs(cur_alt - alt) <= 1.0:
            print("\nWaypoint reached")
            break

        time.sleep(0.2)

# Follow all waypoints
def follow_waypoints(waypoints):
    print(f"Following {len(waypoints)} waypoints")

    for i, (lat, lon, alt) in enumerate(waypoints):
        print(f"\nWaypoint {i+1}/{len(waypoints)}")
        goto_waypoint(lat, lon, alt)




# Connect via MAVProxy UDP port (default 14550)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to MAVProxy/SITL")

# Load waypoints from a TXT file
waypoints = load_qgc_waypoints("mission_waypoints")

set_mode('GUIDED')
arm()
takeoff(10)

# Follow waypoints
follow_waypoints(waypoints)

land()

