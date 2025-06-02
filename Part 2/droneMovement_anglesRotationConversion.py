"""
A python script to move a drone in AirSim using NED coordinates, with conversions from Unreal coordinates if needed.

NOTE: airsim does not work with the newest version of tornado (works with 5.1.1)
Creating virtual environment steps:
cd "C:(backslash)Users(backslash)parin(backslash)Desktop(backslash)Airsim Project(backslash)Part 2"
python -m venv airsim_env
airsim_env(backslash)Scripts(backslash)activate

Uninstall Existing Tornado and Dependencies & Reinstall:
pip uninstall tornado msgpack-rpc-python airsim -y
pip install tornado==4.5.3
pip install numpy (needed for installing airsim)
pip install airsim
pip install msgpack-rpc-python
pip list (to verify installation)
"""

import airsim
import math

print("Hello Drone")

# Connecting to AirSim
client = airsim.MultirotorClient()
client.confirmConnection()
client.enableApiControl(True)
client.armDisarm(True)

# Unreal Eiler angles (degrees, Z-up): yaw=90, pitch=10, roll=0
unreal_yaw_deg = 90 # Facing right (Y-axis in Unreal)
unreal_pitch_deg = 10 # Nose up
unreal_roll_deg = 0

# Convert to radians and adjust for NED (Z-down) -> for quaternion
ned_yaw = math.radians(unreal_yaw_deg) # Yaw aligns if Unreal X is North
ned_pitch = -math.radians(unreal_pitch_deg) # Flip pitch for Z-down
ned_roll = math.radians(unreal_roll_deg)

# Convert to quaternion for AirSim -> for pose orientation
quaternion = airsim.to_quaternion(ned_pitch, ned_roll, ned_yaw)

"""
When in doubt, convert Euler angles to quaternions to ensure compatibility with AirSim's pose APIs.
"""

# Set pose using quaternion

"""Previous version code:"""
# pose = airsim.Pose(
#     position=airsim.Vector3r(10, 5, -2), # NED:10m North, 5m East, 2m up
#     orientation=quaternion
# )

"""New version code:"""
pose = airsim.Pose(
    airsim.Vector3r(10, 5, -2),
    quaternion
)

client.simSetVehiclePose(pose, ignore_collision=True)

# Take off and move to another NED position
client.takeoffAsync().join()
client.moveToPositionAsync(
    x=10, y=5, z=-10, # NED coords (10m North, 5m East, 10m up)
    velocity=5, # m/s
    drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
    yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)
).join()

"""
yaw_mode: Controls the yaw (rotation around the vertical axis) of the vehicle during movement

is_rate: False, means that the vehicle should maintain a fixed yaw angle specified by yaw_or_rate.
If True, it means the vehicle should rotate at a specified rate

yaw_or_rate: When is_rate=False, this value represents the desired yaw angle in degrees.
when is_rate=True, it indicates the rate of rotation in degrees per second
"""

# # Move using Euler yaw (alternative approach)
# client.takeoffAsync().join()
# client.moveToPositionAsync(
#     x=20, y=10, z=-5,  # NED: 20m North, 10m East, 5m up
#     velocity=5,
#     yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=math.degrees(ned_yaw))  # Yaw in degrees
# ).join()


"""
The above commented approach is labeled “alternative” because it reuses the Euler yaw angle from the initial pose calculation (unreal_yaw_deg = 90)
to maintain the same orientation (facing East) during movement, rather than setting a new yaw angle (like 0° in the first code).
"""

pose2 = client.simGetVehiclePose()
print(f"Position: {pose.position.x_val}, {pose.position.y_val}, {pose.position.z_val}")

# Land
print("Landing...")
client.landAsync().join()
client.armDisarm(False) # When you arm the drone, you enable its motors and prepare it for flight
client.enableApiControl(False)


"""
NOTE:
Drone Behavior:

You can observe a sudden movement and the drone turning right. This is expected due to:

Sudden movement: client.simSetVehiclePose(pose, ignore_collision=True) teleports the drone instantly to (10, 5, -2) (10m North, 5m East, 2m up in NED).
Turning right: The unreal_yaw_deg = 90 converts to a 90-degree yaw in NED (facing East if Unreal's X-axis is North),
causing the drone to face right relative to its initial orientation.
"""









