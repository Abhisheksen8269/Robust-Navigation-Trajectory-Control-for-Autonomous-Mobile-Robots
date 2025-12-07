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

<img width="1774" height="932" alt="image" src="https://github.com/user-attachments/assets/fe7e691b-5ec3-489a-8993-c395b77bd1a9" />


RViz Visualization

Real-time visualization of the B-Spline path (Green), LiDAR scans (Red), and the robot's odometry frame.
<img width="1774" height="932" alt="image" src="https://github.com/user-attachments/assets/b8a8e13e-1842-4859-9616-27d1120c63da" />


<img width="1789" height="859" alt="image" src="https://github.com/user-attachments/assets/b18acbbd-933a-4b85-ba4c-258011451e11" />


## 3. Setup & Installation

3.1 Prerequisites

OS: Ubuntu 22.04 LTS (Jammy Jellyfish)

Middleware: ROS 2 Humble Hawksbill

Simulation: Gazebo 11 (Classic)

Dependencies: python3-scipy, python3-numpy

## 3.2 Installation

Clone the repository into your ROS 2 workspace src directory:
```
cd ~/tb3_ws/src
git clone [https://github.com/YOUR_USERNAME/Robust-Navigation-Trajectory-Control-for-Autonomous-Mobile-Robots.git](https://github.com/YOUR_USERNAME/Robust-Navigation-Trajectory-Control-for-Autonomous-Mobile-Robots.git) 
```

## Install system dependencies:
```
sudo apt install ros-humble-turtlebot3-gazebo ros-humble-navigation2
pip3 install scipy numpy
```

## Build the package:
```
cd ~/tb3_ws
colcon build --symlink-install
source install/setup.bash
```

## Configure Environment:
Add the model type to your environment variables.
```
export TURTLEBOT3_MODEL=waffle
```

## 4. Execution Instructions

To replicate the "Messy World" navigation scenario, launch the following commands in three separate terminals:

## Terminal 1: Simulation Environment

Launches Gazebo with the TurtleBot3 Waffle and spawns the environment.
```
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo empty_world.launch.py
```

(Note: You can also spawn additional obstacles manually or using a custom launch file if configured).

## Terminal 2: Visualization

Launches RViz2 to visualize the global plan (Green Path), LiDAR data (Red Points), and Robot Odometry.
```
ros2 run rviz2 rviz2
```

Tip: In RViz, manually add the /global_plan (Path) and /scan (LaserScan) topics to see the data visualization.

## Terminal 3: Navigation Controller

Starts the core node for path smoothing and control logic.
```
source install/setup.bash
ros2 run path_tracking_assignment controller
```
5. Technical Architecture & Logic

The system operates as a single high-frequency node (RobustNavigator) running at 20Hz. The control logic is divided into three hierarchical layers.

Layer 1: Path Smoothing (B-Splines)

Raw waypoints provided by global planners are often discrete and create sharp, non-differentiable turns.

Logic: We apply B-Spline Interpolation (scipy.interpolate.splprep) to the raw waypoints.

Mathematics: A cubic spline ($k=3$) is fitted to generate $C^2$ continuity (continuous velocity and acceleration). This prevents the robot from stopping at every waypoint and allows for fluid motion.

Output: A dense array of 1000 coordinate points $(x, y)$ representing the ideal path.

Layer 2: Trajectory Tracking (Pure Pursuit)

To follow the smoothed path, the robot computes the curvature required to reach a moving target point.

Logic: The controller identifies a Lookahead Point at a fixed distance ($L_d = 0.6m$) ahead of the robot.

Sequential Search Optimization: Standard Pure Pursuit searches the entire path for the closest point, which causes failures on looping paths (where Start $\approx$ End). This implementation uses a sliding window search, only looking for target points ahead of the robot's current index.

Control Law:


$$v_{cmd} = v_{target}, \quad \omega_{cmd} = \frac{2v \sin(\alpha)}{L_d}$$


Where $\alpha$ is the heading error between the robot's orientation and the lookahead vector.

Layer 3: Reactive Obstacle Avoidance (Potential Fields)

This layer overrides the tracking controller when obstacles are detected by the LiDAR.

Perception: The 360° LiDAR scan is filtered to a 120° Frontal Cone.

Force Calculation: Each valid laser point ($p_i$) generates a repulsive force vector ($\vec{F}_{rep}$) inversely proportional to its distance ($d_i$):


$$\vec{F}_{rep} = \sum_{i} \frac{-1}{d_i^2} \cdot \hat{u}_i$$

Control Mixing: The final steering command is a weighted sum of the Goal Vector (from Pure Pursuit) and the Repulsive Vector (from LiDAR).


$$\vec{V}_{final} = w_{goal} \cdot \vec{V}_{goal} + w_{obs} \cdot \vec{V}_{rep}$$

Safety Bubble:

Warning Zone (< 0.6m): Robot slows down and blends the repulsive force to "nudge" around the object.

Critical Zone (< 0.35m): Path tracking is suspended. The robot executes an in-place rotation away from the closest obstacle vector until clear.

6. Extension to Real Hardware

To deploy this package on a physical TurtleBot3:

State Estimation: Replace the /odom subscription with an EKF (Extended Kalman Filter) output (robot_localization package) that fuses Wheel Encoders and IMU to correct for drift.

Sensor Noise: Apply a median filter to the real LiDAR data to remove "salt-and-pepper" noise before calculating repulsive forces.

Fail-safe: Implement a "dead-man timer" that stops the motors if the control loop latency exceeds 200ms.

7. AI Tools Usage Statement

Generative AI tools were utilized during development for:

Scaffolding the ROS 2 node class structure and boilerplate.

Verifying the quaternion-to-euler transformation mathematics.

Debugging the Pure Pursuit index search logic for closed-loop paths.

8. License

This project is licensed under the Apache 2.0 License.


