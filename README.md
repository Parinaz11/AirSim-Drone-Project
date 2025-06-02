# AirSim-Drone-Project
This repository contains a project for controlling a drone in the Microsoft AirSim simulator using the Python API. The project focuses on setting up AirSim, configuring drone simulations, and implementing various flight paths (straight line, square, triangle, circle, and half-circle) for a drone in a virtual environment. The goal is to explore AirSim's capabilities for drone control and data collection, laying the groundwork for future reinforcement learning applications.

## Project Overview

The project includes:

**Setup Guides:** Instructions for cloning and building AirSim, setting up Unreal Engine 4.27, and integrating the AirSim plugin.

**Configuration:** Customization of the settings.json file for drone simulation modes (e.g., Multirotor), vehicle properties, and environmental conditions like wind.

**Python Scripts:** Implementation of drone control scripts to execute predefined flight paths:

**Straight Line Path:** Moves the drone 10 meters along the x-axis at a constant altitude.

**Square Path:** Follows a 10x10 meter square in the xy-plane.

**Triangular Path:** Traces an equilateral triangle with 10-meter sides.

**Circular Path:** Follows a 5-meter radius circle centered at (5,5).

**Half-Circle Path:** Traces a 180-degree arc with a 5-meter radius.


**Data Logging**: Records drone position, orientation, and velocity during flight for analysis.

**Coordinate System**: Handles conversions between Unreal Engine’s coordinate system (Z-up, cm) and AirSim’s NED coordinate system (Z-down, meters).

After following the AirSim setup instructions:

## Configure settings.json:

Locate or create settings.json in Documents\AirSim\.
Example configuration for a single drone is in Part1\settings.json


## Key Scripts and Functionality

`test_straight_line.py`: Configures a single drone to follow a straight path using settings.json.

`Part2\droneMovement_anglesRotationConversion.py`: Handles coordinate and rotation conversions between Unreal Engine and AirSim’s NED system.
SplinePath.py: Implements drone movement functions:

`move_point_to_point`: Moves the drone in a straight line (e.g., from [0,0,-2] to [10,0,-2]).

`move_multi_point_path`: Follows a square path with four waypoints.

`move_triangular_path`: Follows an equilateral triangular path.

`move_circular_path`: Follows a circular path with a 5-meter radius.

`move_half_circle_path`: Follows a 180-degree arc.


**Data Logging:** Configured in settings.json to record position, orientation, and velocity to airsim_rec.txt for analysis (e.g., using matplotlib and pandas).

## Coordinate System

AirSim NED: X (North, forward), Y (East, right), Z (Down, positive downward), in meters.
Unreal Engine: X (forward), Y (right), Z (up), in centimeters.
Conversions: Handled in scripts (e.g., Part2_droneMovement_anglesRotationConversion.py) to map Unreal coordinates to AirSim’s NED system.

## Data Analysis

Recorded data (airsim_rec.txt) includes position, orientation (quaternions), velocity, and timestamps.
Use Python with matplotlib and pandas to visualize flight paths (example in the PDF).

## Notes

Ensure settings.json is saved in ASCII format to avoid issues.
The ClockSpeed setting can adjust simulation speed (e.g., 5.0 for 5x faster).
Wind and geographic origin can be set in settings.json or via client.simSetWind for testing path robustness.
Tested paths include straight, square, triangular, circular, and half-circle trajectories, all at a constant 2-meter altitude.
