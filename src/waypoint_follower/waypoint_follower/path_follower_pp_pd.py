#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class PathFollower(Node):
    """
    A ROS2 node that uses a PD controller to follow a path (list of waypoints).
    Subscribes to /path for the target path and a specified odometry topic for the robot's current state,
    and publishes velocity commands to /cmd_vel.
    """

    # NOTE: CANNOT CHANGE
    def __init__(self, odometry_topic):
        super().__init__('path_follower')

        self.setup_parameters()

        # sum of derivate
        self.sum_derivate = 0
        self.start_time = 0
        self.end_time = 0

        # Path and waypoints
        self.path = None
        self.current_waypoint_index = 0

        # Previous errors (for derivative term)
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
            self.get_logger().info(f"----- All waypoints reached. Stopping. ---- ")
            self.get_logger().info(f"Sum of error: {self.sum_derivate:.3f}, Cost time: {self.end_time - self.start_time:.3f}s.")
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            return True
        
        return False 

    # NOTE: CANNOT CHANGE
    def compute_traj_error(self):
        ################################## Error Computation
        # NOTE: path error is defined as real_path_length / perfect_path_length
        ideal_path_length = 0.0
        for i in range(len(self.path) - 1):
            wp1 = self.path[i].pose.position
            wp2 = self.path[i + 1].pose.position
            dx = wp2.x - wp1.x
            dy = wp2.y - wp1.y
            ideal_path_length += math.hypot(dx, dy)

        # Update total path length
        if not hasattr(self, 'previous_x'):
            self.previous_x = self.current_x
            self.previous_y = self.current_y
        else:
            dx = self.current_x - self.previous_x
            dy = self.current_y - self.previous_y
            self.sum_path_derivate += math.hypot(dx, dy)
            self.previous_x = self.current_x
            self.previous_y = self.current_y

        duration = time.time() - self.start_time
        if ideal_path_length > 0:
            self.sum_derivate = self.sum_path_derivate / ideal_path_length
        else:
            self.sum_derivate = 0.0

        self.get_logger().info(f"Sum of error: {self.sum_derivate:.3f}, Cost time: {time.time() - self.start_time:.3f}s.")

    # NOTE: STUDENTS ARE ALLOWED TO ADD THEIR OWN FUNCTIONS
    def customized_functions(self):
        pass

    # Helper function for clamping values
    def clamp(self, value, min_value, max_value):
        """
        Clamp a value between min and max.
        """
        return max(min_value, min(value, max_value))

    # NOTE: CAN CHANGE - Using implementation from first code
    def setup_parameters(self):
        # Maximum velocities with slight reduction for precision
        self.max_linear_vel = 0.78  # meter
        self.max_angular_vel = 1.05  # rad
                
        # Fixed PD Controller Gains - with slightly higher angular response
        self.Kp_linear = 2.65
        self.Kd_linear = 0.73
                
        self.Kp_angular = 2.55
        self.Kd_angular = 0.42
                
        # Additional parameters needed for the enhanced control strategy
        self.rotation_threshold = 0.36
        
        # Timeout parameters
        self.waypoint_timeout = 5.0
        self.last_waypoint_progress_time = 0.0
        self.min_progress_distance = 0.02
        self.last_distance_to_waypoint = float('inf')
        
        # Track the last control time for derivative calculations
        self.last_control_time = time.time()
        
        # NEW: Parameters for trajectory error factor
        self.traj_error_gain = 0.5  # How much to scale the trajectory error
        self.traj_error_max = 1.5   # Maximum multiplier for control gains

    # NOTE: CAN CHANGE - Using implementation from first code
    def path_callback(self, msg: Path):
        """
        Updates the target path when a new message is received.
        """
        self.path = msg.poses
        self.current_waypoint_index = 0  # Reset to the first waypoint
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints.")
        
        # Reset control variables
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0
        
        # Basic path statistics
        total_length = 0.0
        
        if len(self.path) >= 2:
            for i in range(len(self.path) - 1):
                p1 = self.path[i].pose.position
                p2 = self.path[i+1].pose.position
                segment_length = math.hypot(p2.x - p1.x, p2.y - p1.y)
                total_length += segment_length
        
        self.get_logger().info(f"Received path with {len(self.path)} waypoints, length: {total_length:.2f}m")
        
        # Reset timing and error variables
        self.last_waypoint_progress_time = time.time()
        self.last_control_time = time.time()
        self.last_distance_to_waypoint = float('inf')
        self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.sum_path_derivate = 0
        self.end_time = 0

    # Helper function for debugging
    def log_debug_info(self, distance_to_waypoint, traj_error_factor=None):
        """
        Log debug information about the current state of the path follower.
        """
        if self.path is None:
            return
        
        current_index = self.current_waypoint_index
        if current_index < len(self.path):
            time_since_progress = time.time() - self.last_waypoint_progress_time
            
            self.get_logger().info(f"Distance to waypoint {current_index}: {distance_to_waypoint:.3f} (threshold: {self.waypoint_threshold:.3f})")
            self.get_logger().info(f"Time since progress: {time_since_progress:.1f}s")
        
        self.get_logger().info(f"Position: ({self.current_x:.3f}, {self.current_y:.3f}), Angle: {self.current_orientation:.2f}")
        self.get_logger().info(f"Waypoint: {current_index}/{len(self.path)-1}")
        
        if traj_error_factor is not None:
            self.get_logger().info(f"Trajectory error factor: {traj_error_factor:.3f}")

    # NOTE: CAN CHANGE - Enhanced with trajectory error integration
    def control_loop_callback(self):
        """
        Enhanced control loop that incorporates trajectory error into PD control.
        """
        # Use the required check_ending function
        if self.check_ending():
            return

        # Get current time for dt calculation
        current_time = time.time()
        dt = current_time - self.last_control_time
        self.last_control_time = current_time
        dt = min(dt, 0.1)  # Limit dt for stability
        
        # Get the current waypoint for waypoint tracking purposes
        current_waypoint = self.path[self.current_waypoint_index].pose
        
        # Use look-ahead point for targeting, but preserve waypoint switching logic
        look_ahead = self.get_look_ahead_point()
        if look_ahead is not None:
            target_x, target_y = look_ahead
            error_x = target_x - self.current_x
            error_y = target_y - self.current_y
        else:
            # Fallback to current waypoint
            error_x = current_waypoint.position.x - self.current_x
            error_y = current_waypoint.position.y - self.current_y
        
        # Calculate distance to actual waypoint (for waypoint switching logic)
        distance_to_waypoint = math.hypot(
            current_waypoint.position.x - self.current_x,
            current_waypoint.position.y - self.current_y
        )
        
        # Calculate heading error
        target_heading = math.atan2(error_y, error_x)
        error_theta = self.normalize_angle(target_heading - self.current_orientation)
        
        # Calculate derivatives for error tracking
        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y
        derivative_theta = error_theta - self.prev_error_theta
        
        # Use the compute_traj_error function for consistent error calculation
        self.compute_traj_error()
        
        # Calculate derivatives properly scaled for control
        derivative_x_control = derivative_x / dt if dt > 0 else 0
        derivative_y_control = derivative_y / dt if dt > 0 else 0
        derivative_theta_control = derivative_theta / dt if dt > 0 else 0
        
        # NEW: Calculate trajectory error compensation factor
        # As trajectory error increases, we increase control effort
        # But limit it to prevent instability
        traj_error_factor = min(1.0 + self.traj_error_gain * self.sum_derivate, self.traj_error_max)
        
        # Check for progress and timeout
        if self.last_distance_to_waypoint - distance_to_waypoint > self.min_progress_distance:
            self.last_waypoint_progress_time = current_time
        
        # Check for timeout
        if current_time - self.last_waypoint_progress_time > self.waypoint_timeout:
            self.get_logger().warn(f"Timeout reaching waypoint {self.current_waypoint_index}, moving to next one")
            self.current_waypoint_index += 1
            self.last_waypoint_progress_time = current_time
            self.last_distance_to_waypoint = float('inf')
            
            # Use check_ending to handle the case where we've reached all waypoints
            if self.check_ending():
                return
            
            current_waypoint = self.path[self.current_waypoint_index].pose
            error_x = current_waypoint.position.x - self.current_x
            error_y = current_waypoint.position.y - self.current_y
            distance_to_waypoint = math.hypot(error_x, error_y)
        
        # Store current distance for progress tracking
        self.last_distance_to_waypoint = distance_to_waypoint
        
        # Check if we've reached the current waypoint - use the threshold from init
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_index += 1
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index - 1}/{len(self.path)-1}")
            self.last_waypoint_progress_time = current_time
            self.last_distance_to_waypoint = float('inf')
            
            # Use check_ending to handle the case where we've reached all waypoints
            if self.check_ending():
                return
            
            current_waypoint = self.path[self.current_waypoint_index].pose
            error_x = current_waypoint.position.x - self.current_x
            error_y = current_waypoint.position.y - self.current_y
        
        # Recalculate heading error
        target_heading = math.atan2(error_y, error_x)
        error_theta = self.normalize_angle(target_heading - self.current_orientation)
        
        # Enhanced PD control using trajectory error factor
        vx = self.Kp_linear * error_x * traj_error_factor + self.Kd_linear * derivative_x_control
        vy = self.Kp_linear * error_y * traj_error_factor + self.Kd_linear * derivative_y_control
        vtheta = self.Kp_angular * error_theta * traj_error_factor + self.Kd_angular * derivative_theta_control
        
        # Update previous error terms
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_theta = error_theta
        
        # Create velocity command
        twist_msg = Twist()
        
        # Simple control logic: prioritize rotation when misaligned
        if abs(error_theta) > self.rotation_threshold:
            # Rotate in place
            twist_msg.angular.z = self.clamp(vtheta, -self.max_angular_vel, self.max_angular_vel)
            twist_msg.linear.x = 0.1 * self.max_linear_vel  # Slight forward motion
            self.get_logger().info(f"Rotating to align with waypoint {self.current_waypoint_index}")
        else:
            # Normal movement
            linear_speed = min(self.max_linear_vel, math.hypot(vx, vy))
            
            # Ensure minimum speed
            linear_speed = max(0.2, linear_speed) 
            
            twist_msg.linear.x = linear_speed
            twist_msg.angular.z = self.clamp(vtheta, -self.max_angular_vel, self.max_angular_vel)
            self.get_logger().info(f"Moving to waypoint {self.current_waypoint_index}, dist: {distance_to_waypoint:.2f}")
        
        self.cmd_vel_pub.publish(twist_msg)
        
        # Log debug info periodically
        if self.get_clock().now().nanoseconds % 1000000000 < 100000000:
            self.log_debug_info(distance_to_waypoint, traj_error_factor)

    # Add this method after customized_functions
    def get_look_ahead_point(self):
        """
        Calculate a look-ahead point on the path to make navigation smoother and faster.
        """
        if self.path is None or self.current_waypoint_index >= len(self.path):
            return None
        
        # Base point is current waypoint
        current_waypoint = self.path[self.current_waypoint_index].pose
        
        # Calculate distance to current waypoint
        distance_to_waypoint = math.hypot(
            current_waypoint.position.x - self.current_x,
            current_waypoint.position.y - self.current_y
        )
        
        # Calculate path curvature to adapt look-ahead distance
        path_curvature = 0.0
        if self.current_waypoint_index > 0 and self.current_waypoint_index + 1 < len(self.path):
            prev_wp = self.path[self.current_waypoint_index - 1].pose
            curr_wp = current_waypoint
            next_wp = self.path[self.current_waypoint_index + 1].pose
            
            # Vector from prev to curr
            v1_x = curr_wp.position.x - prev_wp.position.x
            v1_y = curr_wp.position.y - prev_wp.position.y
            
            # Vector from curr to next
            v2_x = next_wp.position.x - curr_wp.position.x
            v2_y = next_wp.position.y - curr_wp.position.y
            
            # Normalize vectors
            v1_len = math.hypot(v1_x, v1_y)
            v2_len = math.hypot(v2_x, v2_y)
            
            if v1_len > 0 and v2_len > 0:
                v1_x /= v1_len
                v1_y /= v1_len
                v2_x /= v2_len
                v2_y /= v2_len
                
                # Dot product gives cosine of angle
                dot_product = v1_x * v2_x + v1_y * v2_y
                angle = math.acos(max(-1.0, min(1.0, dot_product)))
                
                # Higher angle = sharper turn = higher curvature
                path_curvature = angle / math.pi  # Normalize to [0, 1]
        
        # Adaptive look-ahead distance based on:
        # 1. Current speed - faster = look further (but less than before)
        # 2. Path curvature - higher curvature = look less far ahead
        # 3. Distance to waypoint - closer = more blending
        speed = min(math.hypot(self.prev_error_x, self.prev_error_y) / 0.1, 1.0)  # Limit speed influence
        
        # Reduce look-ahead distance with curvature (more conservative)
        curvature_factor = 1.0 - (0.7 * path_curvature)  # Higher curvature = smaller factor
        
        # Calculate adaptive look-ahead distance - make it smaller overall
        look_ahead_distance = 0.2 + (speed * 0.3 * curvature_factor)
        
        # If possible, blend with future waypoints
        if self.current_waypoint_index + 1 < len(self.path):
            next_waypoint = self.path[self.current_waypoint_index + 1].pose
            
            # If close to current waypoint, start blending with next
            if distance_to_waypoint < look_ahead_distance and distance_to_waypoint > 0:
                # More conservative blending
                blend_factor = (1.0 - (distance_to_waypoint / look_ahead_distance)) * 0.6
                blend_factor = min(0.6, blend_factor)  # Cap the blend factor lower
                
                # Create blended point
                blended_x = (1-blend_factor) * current_waypoint.position.x + blend_factor * next_waypoint.position.x
                blended_y = (1-blend_factor) * current_waypoint.position.y + blend_factor * next_waypoint.position.y
                
                return (blended_x, blended_y)
        
        # Default to current waypoint
        return (current_waypoint.position.x, current_waypoint.position.y)


def main():
    import sys
    args = sys.argv 

    rclpy.init(args=args)

    # Specify the odometry topic (default is '/Odometry')
    odometry_topic = '/Odometry'  # Default value
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