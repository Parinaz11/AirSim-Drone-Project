import airsim
import time
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

"""
NOTE: Create a virual environment with:
- python -m venv airsim_env1
Install the libraries: (time is default in python)
- pip install numpy pandas matplotlib
- pip install mgspack-rpc-python
- pip install airsim
Then cd to:
- cd airsim_env1(backslash)Scripts
And run this:
- .(backslash)activate
"""

def interpolate_points(p1, p2, num_points=100):
    """Generate interpolated points between two waypoints."""
    points = []
    for t in np.linspace(0, 1, num_points):
        x = p1[0] + t * (p2[0] - p1[0])
        y = p1[1] + t * (p2[1] - p1[1])
        z = p1[2] + t * (p2[2] - p1[2])
        points.append(airsim.Vector3r(x, y, z))
    return points

def test_straight_line(client, vehicle_name="Drone1"):
    """Moving the drone along a straight-line path to test settings."""
    print(f"Executing straight-line path for {vehicle_name}...")
    
    waypoints = [
        [0, 0, -3],   # We start at (0, 0, -3)
        [20, 20, -3]  # Then move 20 meters along x-axis (forward) and 20 along y-axis (to the right)
    ]
    
    smooth_path = interpolate_points(waypoints[0], waypoints[1], num_points=50)
    
    print("Path points:")
    for i, point in enumerate(smooth_path):
        print(f"Point {i}: {point.x_val}, {point.y_val}, {point.z_val}")
    
    # Initializing the drone
    client.enableApiControl(True, vehicle_name)
    client.armDisarm(True, vehicle_name)
    client.takeoffAsync(vehicle_name=vehicle_name).join()
    
    # Moving along the path
    try:
        start_time = time.time()

        client.moveOnPathAsync(
            smooth_path,
            velocity=2,
            drivetrain=airsim.DrivetrainType.ForwardOnly,
            yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0),
            vehicle_name=vehicle_name
            ).join()

        elapsed_time = time.time() - start_time
        print(f"Straight-line path complete in {elapsed_time:.2f} seconds.")
    except Exception as e:
        print(f"Error: {e}")
    
    # Logging the final position
    pose = client.simGetVehiclePose(vehicle_name=vehicle_name)
    print(f"Final position: {pose.position}, Yaw: {pose.orientation}")
    
    # Disconnecting
    client.armDisarm(False, vehicle_name)
    client.enableApiControl(False, vehicle_name)
    print(f"Disconnected from {vehicle_name}.")

if __name__ == "__main__":
    # Connecting to AirSim
    client = airsim.MultirotorClient()
    client.confirmConnection()
    
    test_straight_line(client)