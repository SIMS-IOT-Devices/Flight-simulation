# PyMavlink GCS flight managment of ArduCopter SITL simulation - Up and Down

from pymavlink import mavutil
import time

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

    # Wait until vehicle reaches 90% of target altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = msg.relative_alt / 1000.0
        print(f"Altitude: {current_alt:.1f} m")

        if current_alt >= altitude * 0.9:
            print("Reached takeoff altitude")
            return
        
        time.sleep(1)

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
        print(f"Altitude: {current_alt:.1f} m")

        if current_alt <= 1:
            print("Landed")
            return
        
        time.sleep(1)

# Connect via MAVProxy UDP port (default 14550)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to MAVProxy/SITL")

set_mode('GUIDED')
arm()
takeoff(10)
land()
