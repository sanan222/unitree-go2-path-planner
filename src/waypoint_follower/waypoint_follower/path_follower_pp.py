#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class PathFollower(Node):
    """
    A ROS2 node that uses pure pursuit with interpolation for smooth path tracking.
    Subscribes to /planner/path for the target path and a specified odometry topic for the robot's current state,
    and publishes velocity commands to /cmd_vel.
    """

    # NOTE: CANNOT CHANGE
    def __init__(self, odometry_topic):
        super().__init__('path_follower')

        self.setup_parameters()

        # sum of derivative errors (performance metric)
        self.sum_derivate = 0
        self.start_time = 0
        self.end_time = 0

        # Path and waypoints
        self.path = None
        self.current_waypoint_index = 0

        # Previous errors (unused in pure pursuit version)
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0

        # Robot current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False

        # Thresholds
        self.waypoint_threshold = 0.1  # Distance threshold to consider a waypoint reached
        self.orientation_threshold = 0.05  # Orientation threshold for final alignment

        # Publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_pub.publish(Twist())

        # Subscriber to odometry (with the specified topic)
        self.odom_sub = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odom_callback,
            10
        )

        # Subscriber to path
        self.path_sub = self.create_subscription(
            Path,
            '/planner/path',
            self.path_callback,
            10
        )

        # Timer to periodically publish velocity commands
        timer_period = 0.1  # [s] -> 10 Hz
        self.timer = self.create_timer(timer_period, self.control_loop_callback)
        self.get_logger().info(f"Path Follower node started. Subscribing to odometry topic: {odometry_topic}")

    # NOTE: CANNOT CHANGE
    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle     

    # NOTE: CANNOT CHANGE
    def odom_callback(self, msg: Odometry):
        """
        Extracts and stores the robot's current pose from the odometry.
        """
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    # NOTE: CANNOT CHANGE
    def check_ending(self):
        if not self.is_odom_received or self.path is None:
            self.cmd_vel_pub.publish(Twist())
            return True

        # Check if all waypoints are reached
        if self.current_waypoint_index >= len(self.path):
            if self.end_time < 1e-3:
                self.end_time = time.time()
            self.get_logger().info("----- All waypoints reached. Stopping. ---- ")
            self.get_logger().info(f"Sum of error: {self.sum_derivate:.3f}, Cost time: {self.end_time - self.start_time:.3f}s.")
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            return True

        return False 

    # NOTE: STUDENTS ARE ALLOWED TO ADD THEIR OWN FUNCTIONS
    def customized_functions(self):
        pass
        
    def interpolate_path(self, original_path, points_per_segment=5):
        """
        Interpolate additional points between existing waypoints to create a denser path.
        
        Args:
            original_path: List of PoseStamped messages
            points_per_segment: Number of points to insert between each pair of waypoints
            
        Returns:
            List of PoseStamped messages with interpolated points
        """
        if len(original_path) < 2:
            return original_path
            
        interpolated_path = []
        
        for i in range(len(original_path) - 1):
            # Add the current waypoint
            interpolated_path.append(original_path[i])
            
            # Get current and next waypoints
            current = original_path[i].pose
            next_wp = original_path[i+1].pose
            
            # Interpolate between current and next waypoint
            for j in range(1, points_per_segment):
                # Calculate interpolation factor (0 to 1)
                t = j / points_per_segment
                
                # Create a new PoseStamped for the interpolated point
                interpolated_pose = PoseStamped()
                interpolated_pose.header = original_path[i].header
                
                # Linear interpolation of position
                interpolated_pose.pose.position.x = (1 - t) * current.position.x + t * next_wp.position.x
                interpolated_pose.pose.position.y = (1 - t) * current.position.y + t * next_wp.position.y
                interpolated_pose.pose.position.z = (1 - t) * current.position.z + t * next_wp.position.z
                
                # For orientation, we'll use a simple interpolation (assuming 2D motion)
                q_current = current.orientation
                q_next = next_wp.orientation
                
                siny_cosp1 = 2.0 * (q_current.w * q_current.z + q_current.x * q_current.y)
                cosy_cosp1 = 1.0 - 2.0 * (q_current.y * q_current.y + q_current.z * q_current.z)
                angle1 = math.atan2(siny_cosp1, cosy_cosp1)
                
                siny_cosp2 = 2.0 * (q_next.w * q_next.z + q_next.x * q_next.y)
                cosy_cosp2 = 1.0 - 2.0 * (q_next.y * q_next.y + q_next.z * q_next.z)
                angle2 = math.atan2(siny_cosp2, cosy_cosp2)
                
                # Interpolate the angle
                interp_angle = (1 - t) * angle1 + t * angle2
                
                # Convert back to quaternion (simplified for 2D)
                interpolated_pose.pose.orientation.w = math.cos(interp_angle / 2)
                interpolated_pose.pose.orientation.z = math.sin(interp_angle / 2)
                interpolated_pose.pose.orientation.x = 0.0
                interpolated_pose.pose.orientation.y = 0.0
                
                interpolated_path.append(interpolated_pose)
        
        # Add the last waypoint
        interpolated_path.append(original_path[-1])
        
        return interpolated_path
    
    def find_lookahead_point(self):
        """
        Find a point on the path that is lookahead_distance away from the robot.
        Returns the target point coordinates and its index in the path.
        """
        if self.path is None or self.current_waypoint_index >= len(self.path):
            return None, self.current_waypoint_index
            
        # Start checking from the current waypoint
        closest_distance = float('inf')
        closest_index = self.current_waypoint_index
        
        # Find the closest point on the path
        for i in range(self.current_waypoint_index, len(self.path)):
            pose = self.path[i].pose
            dist = math.hypot(pose.position.x - self.current_x, pose.position.y - self.current_y)
            if dist < closest_distance:
                closest_distance = dist
                closest_index = i
        
        # Look for a point approximately lookahead_distance away
        for i in range(closest_index, len(self.path)):
            pose = self.path[i].pose
            dist = math.hypot(pose.position.x - self.current_x, pose.position.y - self.current_y)
            if dist >= self.lookahead_distance:
                return (pose.position.x, pose.position.y), i
        
        # If no point is far enough, use the last point
        last_pose = self.path[-1].pose
        return (last_pose.position.x, last_pose.position.y), len(self.path) - 1

    def calculate_steering_angle(self, lookahead_x, lookahead_y):
        """
        Calculate the steering angle to the lookahead point using pure pursuit.
        """
        # Compute the vector from the robot to the lookahead point
        dx = lookahead_x - self.current_x
        dy = lookahead_y - self.current_y
        
        # Compute the target angle in the global frame
        target_angle = math.atan2(dy, dx)
        
        # Compute and normalize the error between the target angle and the current orientation
        steering_angle = self.normalize_angle(target_angle - self.current_orientation)
        
        return steering_angle

    # NOTE: CAN CHANGE
    def setup_parameters(self):
        # Maximum velocities
        self.max_linear_vel = 1.0  # m/s
        self.max_angular_vel = 0.5  # rad/s

        # Pure Pursuit Parameters
        self.lookahead_distance = 0.30  # meters - distance to look ahead on the path
        self.Kp_linear = 0.6  # Proportional gain for linear velocity (unused in pure pursuit)
        self.Kp_angular = 1.0  # Proportional gain for angular velocity (unused in pure pursuit)
        
        # For smooth movement
        self.min_linear_vel = 0.20  # Minimum linear velocity when moving
        self.slow_down_distance = 0.1  # Distance to start slowing down near final waypoint
        
        # Interpolation parameters
        self.points_per_segment = 3  # Number of points to interpolate between waypoints

    # NOTE: CAN CHANGE
    def path_callback(self, msg: Path):
        """
        Updates the target path when a new message is received.
        Interpolates to create a denser path.
        """
        original_path = msg.poses  # List of PoseStamped messages
        
        # Apply interpolation to create a denser path
        self.path = self.interpolate_path(original_path, self.points_per_segment)
        
        self.current_waypoint_index = 0  # Reset to the first waypoint
        self.get_logger().info(f"Received new path with {len(original_path)} waypoints.")
        self.get_logger().info(f"Interpolated to {len(self.path)} waypoints.")
        self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.end_time = 0
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0
        self.sum_derivate = 0

    # NOTE: MODIFIED TO USE PURE PURSUIT
    def control_loop_callback(self):
        """
        Implements pure pursuit path tracking algorithm.
        """
        if self.check_ending():
            return

        # Find the lookahead point along the path
        lookahead, target_index = self.find_lookahead_point()
        if lookahead is None:
            return
        lookahead_x, lookahead_y = lookahead

        # If the found target index is ahead, update the current waypoint index
        if target_index > self.current_waypoint_index:
            self.current_waypoint_index = target_index
            self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}")

        # Compute the steering angle to the lookahead point
        steering_angle = self.calculate_steering_angle(lookahead_x, lookahead_y)

        # Compute linear velocity.
        # Here, we use a constant (or modulated) linear velocity and slow down if close to the final waypoint.
        final_pose = self.path[-1].pose
        distance_to_final = math.hypot(final_pose.position.x - self.current_x, 
                                       final_pose.position.y - self.current_y)
        linear_vel = self.max_linear_vel
        if distance_to_final < self.slow_down_distance:
            linear_vel = max(self.min_linear_vel, self.max_linear_vel * (distance_to_final / self.slow_down_distance))

        # Pure Pursuit control law for angular velocity:
        # angular_vel = linear_vel * (2 * sin(steering_angle)) / lookahead_distance
        angular_vel = linear_vel * (2.0 * math.sin(steering_angle)) / self.lookahead_distance

        # Cap the angular velocity to the maximum limit
        angular_vel = max(-self.max_angular_vel, min(self.max_angular_vel, angular_vel))

        # Create the Twist message with computed velocities
        twist_msg = Twist()
        twist_msg.linear.x = linear_vel
        twist_msg.angular.z = angular_vel

        # Check if the robot is close enough to the current waypoint and update if so
        current_target = self.path[self.current_waypoint_index].pose
        error_distance = math.hypot(current_target.position.x - self.current_x,
                                    current_target.position.y - self.current_y)
        if error_distance < self.waypoint_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index}.")
            self.current_waypoint_index += 1

        self.cmd_vel_pub.publish(twist_msg)

def main():
    import sys
    args = sys.argv 

    rclpy.init(args=args)

    # Specify the odometry topic (default is '/Odometry')
    odometry_topic = '/Odometry'  # Default value, can be replaced by command-line arg if needed
    node = PathFollower(odometry_topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
