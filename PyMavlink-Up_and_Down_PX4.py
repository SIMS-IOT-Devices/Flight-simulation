# The pymavlink Ground Control Station (GCS) code for
# SITL jMAVSim simulation running a PX4 program on the flight computer
# The program designed to take the quadcopter UP and DOWN

import time
from pymavlink import mavutil

# 1. Connection Setup
master = mavutil.mavlink_connection('udp:127.0.0.1:14540')
master.wait_heartbeat()
print("Heartbeat received! Target System:", master.target_system)

# 2. Define Bitmasks
# To control velocity, we must IGNORE position and acceleration
# Bitmask: 1 = Ignore, 0 = Enable
VELOCITY_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

# To hover, we use a mask that enables Position
POSITION_MASK = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
)

def send_setpoint(mask, x=0, y=0, z=0, vx=0, vy=0, vz=0):
    """Sends MAVLink setpoints to the flight controller"""
    master.mav.set_position_target_local_ned_send(
        0, # boot_time
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        mask,
        x, y, z,     # Position
        vx, vy, vz,  # Velocity
        0, 0, 0,     # Acceleration
        0, 0         # Yaw
    )

# 3. Pre-flight initialization
# PX4 requires a stream of setpoints before it allows entering OFFBOARD mode
print("Sending initial stream of setpoints...")
for _ in range(50):
    send_setpoint(VELOCITY_MASK, vz=0)
    time.sleep(0.1)

# 4. Set Mode to OFFBOARD
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0,
    1, # Base Mode: Custom
    6, # Custom Mode: PX4 OFFBOARD
    0, 0, 0, 0, 0
)

# 5. Arm the Motors
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    1, 0, 0, 0, 0, 0, 0
)
print("Armed and Offboard Active.")

# 6. Execution: Velocity Takeoff
target_altitude = -5.0  # 5 meters up (NED is negative)
ascent_speed = -0.75    # 0.75 m/s upwards
current_alt = 0.0

print(f"Beginning ascent to {abs(target_altitude)}m...")

while current_alt > target_altitude:
    # Continuously send velocity command
    send_setpoint(VELOCITY_MASK, vz=ascent_speed)
    
    # Read actual altitude from the flight controller
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
    if msg:
        current_alt = msg.z
        print(f"Altitude: {abs(current_alt):.2f}m", end='\r')
    
    time.sleep(0.1)

# 7. Transition to Position Hold (Hover)
print(f"\nTarget reached. Hovering at {abs(target_altitude)}m.")
for _ in range(100): # Hover for 10 seconds
    send_setpoint(POSITION_MASK, x=0, y=0, z=target_altitude)
    time.sleep(0.1)

# 8. Landing (Simple velocity descent)
print("Landing...")
while current_alt < -0.2: # Land until 20cm above ground
    send_setpoint(VELOCITY_MASK, vz=0.5) # 0.5 m/s down
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=0.1)
    if msg:
        current_alt = msg.z
    time.sleep(0.1)

# Disarm
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
    0, 0, 0, 0, 0, 0, 0
)
print("Landed and Disarmed.")
