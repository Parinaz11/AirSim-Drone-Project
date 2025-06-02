import airsim
import math
import time
import numpy as np

def interpolate_points(start, end, num_points=100):
    """Interpolate num_points between start and end points for smooth movement."""
    points = []
    for i in range(num_points + 1):
        t = i / num_points
        x = (1 - t) * start[0] + t * end[0]
        y = (1 - t) * start[1] + t * end[1]
        z = (1 - t) * start[2] + t * end[2]
        points.append(airsim.Vector3r(x, y, z))
    return points

def generate_circle_points(center, radius, altitude, num_points=100):
    """Generate waypoints for a circular path."""
    points = []
    for i in range(num_points):
        theta = 2 * math.pi * i / num_points
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        z = altitude
        points.append(airsim.Vector3r(x, y, z))
    return points

def generate_half_circle_points(center, radius, altitude, num_points=50):
    """Generate waypoints for a half-circle path (180 degrees)."""
    points = []
    for i in range(num_points):
        theta = math.pi * i / (num_points - 1)  # 0 to pi radians
        x = center[0] + radius * math.cos(theta)
        y = center[1] + radius * math.sin(theta)
        z = altitude
        points.append(airsim.Vector3r(x, y, z))
    return points

def move_point_to_point(client):
    """Moving drone from one point to another."""
    print("Executing point-to-point movement...")
    start = [0, 0, -2] # The drone’s starting position, with coordinates (x, y, z) = (0, 0, -2) meters. -> goes up
    end = [10, 0, -2] #The drone’s destination, with coordinates (x, y, z) = (10, 0, -2) meters. -> goes north
    
    """
    NOTE: AirSim Coordinate System:
    x: Forward/backward direction (positive x is forward).
    y: Left/right direction (positive y is right).
    z: Up/down direction (negative z is upward, positive z is downward).
    """
    
    points = interpolate_points(start, end, num_points=100)
    client.moveOnPathAsync(points, velocity=5, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                           yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
    print("Point-to-point movement complete.")

def move_multi_point_path(client):
    """Move drone along a path with multiple waypoints (square)."""
    print("Executing multi-point path...")
    waypoints = [
        [0, 0, -2],
        [10, 0, -2],
        [10, 10, -2],
        [0, 10, -2],
        [0, 0, -2]
    ]

    # Interpolate 100 points between each pair of waypoints
    smooth_path = []
    for i in range(len(waypoints) - 1):
        smooth_path.extend(interpolate_points(waypoints[i], waypoints[i + 1], num_points=100))
    
    client.moveOnPathAsync(smooth_path, velocity=5, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                           yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)
                           ).join()
    print("Multi-point path complete.")

def move_circular_path(client):
    """Move drone along a circular path."""
    print("Executing circular path...")
    center = [5, 5, -2]  # Center at (5, 5, -2)
    radius = 5
    points = generate_circle_points(center, radius, altitude=-2, num_points=100)
    try:
        while True:
            client.moveOnPathAsync(points, velocity=5, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                                   yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
            print("Completed one circle, repeating...")
    except KeyboardInterrupt:
        print("Stopping circular path...")
    print("Circular path complete.")

def move_triangular_path(client):
    """Move drone along a triangular path."""
    print("Executing triangular path...")
    waypoints = [
        [0, 0, -2],           # Vertex 1
        [10, 0, -2],          # Vertex 2
        [5, 8.66, -2],        # Vertex 3 (equilateral triangle, height = 10 * sqrt(3)/2)
        [0, 0, -2]            # Back to start
    ]
    # Interpolating 100 points between each pair of waypoints
    smooth_path = []
    for i in range(len(waypoints) - 1):
        smooth_path.extend(interpolate_points(waypoints[i], waypoints[i + 1], num_points=100))
    client.moveOnPathAsync(smooth_path, velocity=5, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                           yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
    print("Triangular path complete.")

def move_half_circle_path(client):
    """Move drone along a half-circle path."""
    print("Executing half-circle path...")
    center = [5, 5, -2]
    radius = 5
    points = generate_half_circle_points(center, radius, altitude=-2, num_points=50)
    client.moveOnPathAsync(points, velocity=5, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                           yaw_mode=airsim.YawMode(is_rate=False, yaw_or_rate=0)).join()
    print("Half-circle path complete.")

def main():
    # Connecting to AirSim
    client = airsim.MultirotorClient()
    try:
        client.confirmConnection()
        client.enableApiControl(True)
        client.armDisarm(True)

        print("Taking off...")
        client.takeoffAsync().join()

        while True:
            print("\nSelect a path to execute:")
            print("1. Point-to-point movement")
            print("2. Multi-point path (square)")
            print("3. Circular path (continuous)")
            print("4. Triangular path")
            print("5. Half-circle path")
            print("6. Exit")
            choice = input("Enter choice (1-6): ")

            if choice == "1":
                move_point_to_point(client)
            elif choice == "2":
                move_multi_point_path(client)
            elif choice == "3":
                move_circular_path(client)
            elif choice == "4":
                move_triangular_path(client)
            elif choice == "5":
                move_half_circle_path(client)
            elif choice == "6":
                return
            else:
                print("Invalid choice, try again.")

        # Land
        print("Landing...")
        client.landAsync().join()

    except Exception as e:
        print(f"Error: {e}")
    finally:
        client.armDisarm(False)
        client.enableApiControl(False)
        print("Disconnected from AirSim.")

if __name__ == "__main__":
    main()