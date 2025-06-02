# AirSim-Drone-Project
This repository contains a project for controlling a drone in the Microsoft AirSim simulator using the Python API. The project focuses on setting up AirSim, configuring drone simulations, and implementing various flight paths (straight line, square, triangle, circle, and half-circle) for a drone in a virtual environment. The goal is to explore AirSim's capabilities for drone control and data collection, laying the groundwork for future reinforcement learning applications.

Project Overview

The project includes:

Setup Guides: Instructions for cloning and building AirSim, setting up Unreal Engine 4.27, and integrating the AirSim plugin.

Configuration: Customization of the settings.json file for drone simulation modes (e.g., Multirotor), vehicle properties, and environmental conditions like wind.

Python Scripts: Implementation of drone control scripts to execute predefined flight paths:

Straight Line Path: Moves the drone 10 meters along the x-axis at a constant altitude.

Square Path: Follows a 10x10 meter square in the xy-plane.

Triangular Path: Traces an equilateral triangle with 10-meter sides.

Circular Path: Follows a 5-meter radius circle centered at (5,5).

Half-Circle Path: Traces a 180-degree arc with a 5-meter radius.



Data Logging: Records drone position, orientation, and velocity during flight for analysis.



Coordinate System: Handles conversions between Unreal Engine’s coordinate system (Z-up, cm) and AirSim’s NED coordinate system (Z-down, meters).
