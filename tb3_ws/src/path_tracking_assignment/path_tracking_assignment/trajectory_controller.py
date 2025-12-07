#!/usr/bin/env python3

"""
RobustNavigator: A Reactive Path Tracking Controller for Cluttered Environments
This node implements a hybrid navigation strategy combining global path planning
with local reactive obstacle avoidance. It is designed to guide a differential
drive robot TurtleBot3 through a complex obstacle field without a pre-built map.

Key Algorithms:
1. Path Smoothing: B-Spline interpolation for continuous curvature paths.
2. Path Tracking: Pure Pursuit algorithm with dynamic lookahead.
3. Obstacle Avoidance: Artificial Potential Field based on LiDAR data.

ABHISHEK SEN
Date: 07 December 2025
MOBILE NO : 8269856166
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
import numpy as np
import math
from scipy.interpolate import splprep, splev
from geometry_msgs.msg import Twist, PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from tf_transformations import euler_from_quaternion

class RobustNavigator(Node):
    def __init__(self):
        super().__init__('robust_navigator')
        self.get_logger().info("Initializing Robust Navigator System...")

        # THIS IS CONTROLLER TUNING 
        # Larger values = smoother motion but higher path deviation.
        self.lookahead_dist = 0.5       
        
        # This is Velocity Profile
        self.cruise_speed = 0.22        
        
        # This is Obstacle Thresholds
        self.critical_dist = 0.35       
        self.warning_dist = 0.60        
        #This is my Navigation Goals
        self.goal_tolerance = 0.3      
        
        # THIS IS MY GLOBAL PATH DEFINITION 
        # Discrete waypoints defining a "Sine Wave" pattern to force the robot
        # to traverse the entire width of the obstacle field.
        self.waypoints = [
            [-2.0,  0.0], #This is  Start Point
            [-1.5, -1.5],
            [-1.0,  1.5], 
            [-0.5, -1.0], 
            [ 0.0,  1.0], 
            [ 0.5, -1.0], 
            [ 1.0,  1.5], 
            [ 1.5, -1.5], 
            [ 2.0,  0.0], 
            [ 2.5, -0.5]  # This is Final Goal
        ]
        
        # Generate the continuous reference trajectory at startup
        self.trajectory = self.generate_trajectory(self.waypoints)
        
        # THIS IS  ROBOT STATE 
        self.robot_pos = np.array([-2.0, -0.5])
        self.robot_yaw = 0.0
        self.odom_received = False
        
        # THIS IS NAVIGATION STATE 
        # Tracks progress along the path index (0 to N) to prevent backtracking
        self.current_idx = 0  
        
        # Obstacle state derived from LiDAR
        self.min_dist = 10.0
        self.obstacle_angle = 0.0 

        # FOR ROS 2 COMMUNICATION  
        self.pub_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_path = self.create_publisher(Path, '/global_plan', 10)
        
        # Use BEST_EFFORT QoS for LaserScan to handle high-frequency sensor data
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos)
        
        # Main control loop running at 20Hz
        self.timer = self.create_timer(0.05, self.control_loop)

    def generate_trajectory(self, waypoints):
        """ 
        Task 1: Path Smoothing
        Uses B-Spline interpolation to convert discrete waypoints into a 
        smooth, differentiable trajectory suitable for tracking.
        """
        pts = np.array(waypoints)
        try:
            # k=3 generates a cubic spline and s=0.0 forces the spline to pass through every waypoint exactly
            tck, u = splprep(pts.T, s=0.0, k=3) 
        except:
            # Fallback for edge cases (e.g., collinear points)
            tck, u = splprep(pts.T, s=0.0, k=1)

        # Resample the spline into a dense array of points
        u_new = np.linspace(0, 1, 1000)
        x_new, y_new = splev(u_new, tck)
        return np.vstack((x_new, y_new)).T

    def scan_callback(self, msg):
        """ 
        Sensor Processing: LiDAR Data Analysis
        Filters the raw laser scan to identify the closest obstacle within a 
        frontal field of view (FOV). This is critical for reactive avoidance.
        """
        # Map indices to angles
        ranges = np.array(msg.ranges)
        ranges[ranges == 0.0] = 10.0 
        n = len(ranges)
        limit = int(n / 6) 
        left_side = ranges[0:limit]
        right_side = ranges[-limit:]
        front_cone = np.concatenate((left_side, right_side))
        self.min_dist = np.min(front_cone)
        min_idx_local = np.argmin(front_cone)
        
        # Convert local index back to relative angle which is in radian 
        if min_idx_local < len(left_side):
            self.obstacle_angle = (min_idx_local / n) * 2 * math.pi
        else:
            idx_from_end = len(front_cone) - min_idx_local
            self.obstacle_angle = - (idx_from_end / n) * 2 * math.pi

    def odom_callback(self, msg):
        """ State Estimation: Updates robot pose from odometry """
        self.robot_pos[0] = msg.pose.pose.position.x
        self.robot_pos[1] = msg.pose.pose.position.y
        
        # Convert Quaternion to Euler Yaw for 2D navigation
        q = msg.pose.pose.orientation
        orientation_list = [q.x, q.y, q.z, q.w]
        _, _, self.robot_yaw = euler_from_quaternion(orientation_list)
        self.odom_received = True

    def get_lookahead_point(self):
        """ 
        Task 3: Pure Pursuit Target Selection
        Finds a point on the path at a fixed 'lookahead_distance' ahead of the robot.
        Crucially, this search is sequential to prevent the robot from 'skipping' 
        loops or crossing path segments incorrectly.
        """
        # Optimization: Search only a local window ahead of the last known position
        search_range = 50 
        start = self.current_idx
        end = min(self.current_idx + search_range, len(self.trajectory))
        
        subset = self.trajectory[start:end]
        if len(subset) == 0:
            return self.trajectory[-1] # Default to end point

        # Find the geometrically closest point in the search window
        dists = np.linalg.norm(subset - self.robot_pos, axis=1)
        local_min = np.argmin(dists)
        self.current_idx = start + local_min # Update global path progress
        
        # Advance along the path until the lookahead distance is met
        lookahead_idx = self.current_idx
        while lookahead_idx < len(self.trajectory) - 1:
            d = np.linalg.norm(self.trajectory[lookahead_idx] - self.robot_pos)
            if d > self.lookahead_dist:
                break
            lookahead_idx += 1
            
        return self.trajectory[lookahead_idx]
    # This funtion is for obstacle avoidance 
    def control_loop(self):
        """ 
        Main Control Loop (20Hz)
        Implements a layered control architecture:
        Layer 1: Safety (Obstacle Avoidance)
        Layer 2: Path Following (Pure Pursuit)
        """
        if not self.odom_received:
            return

        cmd = Twist()
        # If an obstacle breaches the critical safety distance, override all navigation
        if self.min_dist < self.critical_dist:
            self.get_logger().warn(f"Collision Risk! Dist: {self.min_dist:.2f}m. Taking evasive action.")
            cmd.linear.x = 0.0
            # Simple heuristic: turn away from the obstacle
            if self.obstacle_angle > 0: 
                cmd.angular.z = -0.6 
            else:
                cmd.angular.z = 0.6  
            self.pub_cmd.publish(cmd)
            return 

        # LAYER 2: PATH FOLLOWING 
        target = self.get_lookahead_point()
        
        # Transform global target to robot-centric frame
        dx = target[0] - self.robot_pos[0]
        dy = target[1] - self.robot_pos[1]
        
        local_x = math.cos(self.robot_yaw) * dx + math.sin(self.robot_yaw) * dy
        local_y = -math.sin(self.robot_yaw) * dx + math.cos(self.robot_yaw) * dy
        
        # Calculate curvature for Pure Pursuit control law: gamma = 2y / L^2
        curvature = 2.0 * local_y / (self.lookahead_dist ** 2)

        # Dynamic Behavior Adjustment
        if self.min_dist < self.warning_dist:
            # Case: Obstacle Nearby -> Slow down and blend repulsive force
            repulsion_bias = -1.0 * (self.obstacle_angle / abs(self.obstacle_angle)) * 0.5
            
            cmd.linear.x = 0.1 
            cmd.angular.z = (curvature * 0.1) + repulsion_bias
        else:
            cmd.linear.x = self.cruise_speed
            cmd.angular.z = curvature * self.cruise_speed

        # THIS IS FOR MISSION STATUS CHECK 
        dist_to_goal = np.linalg.norm(self.trajectory[-1] - self.robot_pos)
        path_percent = self.current_idx / len(self.trajectory)
        
        # Success condition: Near goal AND significant path progress made
        if dist_to_goal < self.goal_tolerance and path_percent > 0.90:
            self.get_logger().info("Mission Accomplished: Goal Reached.")
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.pub_cmd.publish(cmd)
            self.destroy_node() 
            return

        # Publish velocity command and visualization
        self.pub_cmd.publish(cmd)
        self.publish_path_viz()

    def publish_path_viz(self):
        """ Helper to visualize the planned B-Spline in RViz """
        msg = Path()
        msg.header.frame_id = "odom"
        msg.header.stamp = self.get_clock().now().to_msg()
        # Downsample trajectory for efficient visualization
        for pt in self.trajectory[::20]: 
            p = PoseStamped()
            p.pose.position.x = pt[0]
            p.pose.position.y = pt[1]
            msg.poses.append(p)
        self.pub_path.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobustNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
