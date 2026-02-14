# ArduPilot flight of ArduCopter SITL simulation - Square Loitering

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

# Flight to dx [m], dy [m], duration [seconds]
def flight_to(dx, dy, duration):
    
    print("Starting the flight")

    # Get current position as circle center
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    center_lat = msg.lat / 1e7
    center_lon = msg.lon / 1e7
    alt = msg.relative_alt / 1000.0

    earth_radius = 6378137.0  # meters
    start_time = time.time()

    while time.time() - start_time < duration:

        # Direction relative to North
        msg = master.recv_match(type='VFR_HUD', blocking=True)
        heading = msg.heading  # degrees, 0–359
        print(f"Heading (from North): {heading}°  ", end="\r", flush=True)

        t = time.time() - start_time
       
        # Convert meters → lat/lon
        dlat = (dy / earth_radius) * (180 / math.pi)
        dlon = (dx / (earth_radius * math.cos(math.radians(center_lat)))) * (180 / math.pi)

        target_lat = center_lat + dlat
        target_lon = center_lon + dlon

        master.mav.set_position_target_global_int_send(
            0,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,  # position enabled
            int(target_lat * 1e7),
            int(target_lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

        time.sleep(0.1)

    print("")
    print("Flight to the point is completed")


# Land
def land():
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0, 0, 0, 0
    )
    print("Landing...")

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.1f} m  ", end="\r", flush=True)

        if current_alt <= 1:
            print("Landed                ")
            return
        
        time.sleep(0.1)

# Connect via MAVProxy UDP port (default 14550)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to MAVProxy/SITL")

set_mode('GUIDED')
arm()
takeoff(10)
flight_to(0,100,20) 
flight_to(100,0,20) 
flight_to(0,-100,20) 
flight_to(-100,0,20) 
land()
