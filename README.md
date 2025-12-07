# Robust-Navigation-Trajectory-Control-for-Autonomous-Mobile-RobotsRobust Navigation & Trajectory Control for Autonomous Mobile Robots

## 1. Project Overview

This repository implements a complete navigation stack for differential drive robots (TurtleBot3) operating in unstructured environments. The system bypasses traditional heavy global planners (like Nav2's costmap) in favor of a lightweight, hybrid control architecture.

## Core Capabilities:

Global Path Smoothing: Converts sparse waypoints into continuous B-Spline trajectories to ensure smooth kinematic profiles.

Trajectory Tracking: Utilizes a Pure Pursuit controller with sequential index search for robust path following.

Reactive Safety Layer: Implements Artificial Potential Fields (APF) using 2D LiDAR data to dynamically deviate from the global path and avoid obstacles.

## 2. Simulation Results

Gazebo Environment ("The Mess")

The robot navigates through a randomized field of obstacles (construction cones, barriers, dumpsters). The environment is designed to test the robot's ability to deviate from its optimal path to ensure safety.

(Place your Gazebo screenshot here. E.g., The robot facing the obstacles)

RViz Visualization

Real-time visualization of the B-Spline path (Green), LiDAR scans (Red), and the robot's odometry frame.

(Place your RViz screenshot here. E.g., The green path line and laser scan)

## 2. Setup & Installation

2.1 Prerequisites

OS: Ubuntu 22.04 LTS (Jammy Jellyfish)

Middleware: ROS 2 Humble Hawksbill

Simulation: Gazebo 11 (Classic)

Dependencies: python3-scipy, python3-numpy

## 2.2 Installation

Clone the repository into your ROS 2 workspace src directory:

cd ~/tb3_ws/src
git clone [https://github.com/YOUR_USERNAME/Robust-Navigation-Trajectory-Control-for-Autonomous-Mobile-Robots.git](https://github.com/YOUR_USERNAME/Robust-Navigation-Trajectory-Control-for-Autonomous-Mobile-Robots.git)


## Install system dependencies:

sudo apt install ros-humble-turtlebot3-gazebo ros-humble-navigation2
pip3 install scipy numpy


## Build the package:

cd ~/tb3_ws
colcon build --symlink-install
source install/setup.bash


## Configure Environment:
Add the model type to your environment variables.

export TURTLEBOT3_MODEL=waffle


## 3. Execution Instructions

To replicate the "Messy World" navigation scenario, launch the following commands in three separate terminals:

## Terminal 1: Simulation Environment

Launches Gazebo with the TurtleBot3 Waffle and spawns the environment.

export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py


(Note: You can also spawn additional obstacles manually or using a custom launch file if configured).

## Terminal 2: Visualization

Launches RViz2 to visualize the global plan (Green Path), LiDAR data (Red Points), and Robot Odometry.

ros2 run rviz2 rviz2


Tip: In RViz, manually add the /global_plan (Path) and /scan (LaserScan) topics to see the data visualization.

## Terminal 3: Navigation Controller

Starts the core node for path smoothing and control logic.

source install/setup.bash
ros2 run path_tracking_assignment controller
