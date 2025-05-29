#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time
import numpy as np
from scipy.optimize import minimize
from scipy.interpolate import CubicSpline

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class PathFollower(Node):
    """
    A ROS2 node that uses a PID controller to follow a path (list of waypoints).
    Subscribes to /planner/path for the target path and a specified odometry topic for the robot's current state,
    and publishes velocity commands to /cmd_vel.
    """

    def __init__(self, odometry_topic):
        super().__init__('path_follower')

        self.setup_parameters()

        # Optionally run PID auto-tuning
        if self.auto_tune:
            self.auto_tune_pid()

        # sum of derivative for logging
        self.sum_derivate = 0
        self.start_time = 0
        self.end_time = 0

        # Path and waypoints
        self.path = None
        self.original_path = None
        self.current_waypoint_index = 0

        # Previous errors (for derivative term)
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0

        # Integral errors (for integral term in PID)
        self.integral_error_x = 0.0
        self.integral_error_y = 0.0
        self.integral_error_theta = 0.0
        
        # Robot current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False

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

    def setup_parameters(self):
        # Maximum velocities
        self.max_linear_vel = 0.9  # reduced for better cornering
        self.max_angular_vel = 1.5  # increased for sharper turns
        
        # Controller type: using PID
        self.controller_type = 'PID'
        
        # PID Controller Gains (initial/default values)
        self.Kp_linear = 1.5
        self.Kd_linear = 0.3
        self.Kp_angular = 2.0
        self.Kd_angular = 0.4
        self.Ki_linear = 0.2
        self.Ki_angular = 0.3
        
        # Anti-windup: max integral error to prevent excessive buildup
        self.max_integral_error = 1.5
        
        # Timing for integral calculation
        self.dt = 0.1  # Assuming control loop runs at 10Hz
        
        # Path smoothing parameters - using collocation
        self.enable_smoothing = True
        self.smoothing_technique = 'collocation'
        
        # Path simplification parameters
        self.enable_simplification = True
        self.simplification_tolerance = 0.08
        
        # Collocation parameters
        self.collocation_points = 30
        self.collocation_weight_path = 1.5
        self.collocation_weight_smoothness = 0.3
        
        # Auto-tuning flag for PID parameters
        self.auto_tune = True

    def binomial_coefficient(self, n, k):
        """Calculate binomial coefficient (n choose k)"""
        return math.factorial(n) // (math.factorial(k) * math.factorial(n - k))
    
    def bezier_point(self, control_points, t):
        """Calculate point on Bezier curve at parameter t using Bernstein polynomials"""
        n = len(control_points) - 1
        point = [0.0, 0.0]
        
        for i in range(n + 1):
            binomial = self.binomial_coefficient(n, i)
            bernstein = binomial * (t ** i) * ((1 - t) ** (n - i))
            point[0] += control_points[i][0] * bernstein
            point[1] += control_points[i][1] * bernstein
            
        return point

    def smooth_path_collocation(self, points):
        """
        Applies direct collocation method to smooth the trajectory.
        
        Args:
            points: List of [x, y] coordinates
            
        Returns:
            Smoothed list of [x, y] coordinates
        """
        if len(points) < 3:
            return points
            
        n_points = len(points)
        n_output = self.collocation_points
        
        # Define the parameterization for the path (normalized arc length)
        total_length = 0.0
        segment_lengths = []
        
        for i in range(1, n_points):
            dx = points[i][0] - points[i-1][0]
            dy = points[i][1] - points[i-1][1]
            segment_length = math.sqrt(dx*dx + dy*dy)
            total_length += segment_length
            segment_lengths.append(segment_length)
            
        # Create parameterization
        t_orig = [0.0]
        for i in range(len(segment_lengths)):
            t_orig.append(t_orig[-1] + segment_lengths[i]/total_length)
            
        # Ensure t_orig spans [0, 1]
        t_orig = np.array(t_orig)
        
        # Desired output times
        t_out = np.linspace(0, 1, n_output)
        
        # Create separate splines for x and y coordinates
        x_vals = np.array([p[0] for p in points])
        y_vals = np.array([p[1] for p in points])
        
        try:
            # Create cubic splines for initial guess
            cs_x = CubicSpline(t_orig, x_vals)
            cs_y = CubicSpline(t_orig, y_vals)
            
            # Initial position values at collocation points
            x_init = cs_x(t_out)
            y_init = cs_y(t_out)
            
            # Combine into a single parameter vector
            init_params = np.concatenate((x_init, y_init))
            
            # Define the objective function that balances path following and smoothness
            def objective(params):
                # Extract x and y positions
                x = params[:n_output]
                y = params[n_output:]
                
                # Path following error (distance from original spline)
                path_error = 0.0
                for i in range(n_output):
                    spline_x = cs_x(t_out[i])
                    spline_y = cs_y(t_out[i])
                    path_error += ((x[i] - spline_x)**2 + (y[i] - spline_y)**2)
                
                # Smoothness term (sum of squared second differences - approximation of curvature)
                smoothness = 0.0
                for i in range(1, n_output-1):
                    x_diff2 = x[i+1] - 2*x[i] + x[i-1]
                    y_diff2 = y[i+1] - 2*y[i] + y[i-1]
                    smoothness += x_diff2**2 + y_diff2**2
                
                # Combined objective with weights
                return self.collocation_weight_path * path_error + self.collocation_weight_smoothness * smoothness
            
            # End point constraints (to ensure we start and end at the original points)
            constraints = []
            
            # Start point constraint
            def constraint_start_x(params):
                return params[0] - points[0][0]  # x₀ = x_start
                
            def constraint_start_y(params):
                return params[n_output] - points[0][1]  # y₀ = y_start
                
            # End point constraint
            def constraint_end_x(params):
                return params[n_output-1] - points[-1][0]  # x_end = x_final
                
            def constraint_end_y(params):
                return params[2*n_output-1] - points[-1][1]  # y_end = y_final
                
            constraints.append({'type': 'eq', 'fun': constraint_start_x})
            constraints.append({'type': 'eq', 'fun': constraint_start_y})
            constraints.append({'type': 'eq', 'fun': constraint_end_x})
            constraints.append({'type': 'eq', 'fun': constraint_end_y})
            
            # Run the optimization
            result = minimize(objective, init_params, method='SLSQP', constraints=constraints, 
                              options={'maxiter': 100, 'disp': False})
            
            if result.success:
                self.get_logger().info("Collocation optimization successful!")
                # Extract the optimized trajectory
                x_opt = result.x[:n_output]
                y_opt = result.x[n_output:]
                
                # Create resulting points list
                smoothed_points = []
                for i in range(n_output):
                    smoothed_points.append([x_opt[i], y_opt[i]])
                    
                return smoothed_points
            else:
                self.get_logger().warn(f"Collocation optimization failed: {result.message}. Using spline approximation.")
                # Fall back to spline if optimization fails
                smoothed_points = []
                for t in t_out:
                    smoothed_points.append([cs_x(t), cs_y(t)])
                return smoothed_points
                
        except Exception as e:
            self.get_logger().error(f"Error in collocation smoothing: {e}. Using original path.")
            return points

    def smooth_path(self, poses):
        """
        Smooths the path using direct collocation.
        
        Args:
            poses: List of PoseStamped messages
            
        Returns:
            List of PoseStamped messages (smoothed path)
        """
        if len(poses) < 3:
            self.get_logger().info("Path too short for smoothing, returning original path")
            return poses
        
        # Extract x, y coordinates from poses
        points = []
        for pose in poses:
            points.append([pose.pose.position.x, pose.pose.position.y])
        
        # Simplify the path to remove unnecessary waypoints
        if self.enable_simplification:
            simplified_points = self.simplify_path(points, self.simplification_tolerance)
            self.get_logger().info(f"Path simplified from {len(points)} to {len(simplified_points)} points")
            points = simplified_points
        
        if len(points) < 3:
            # After simplification, if we don't have enough points, generate poses from them
            smoothed_poses = []
            for i, point in enumerate(points):
                new_pose = PoseStamped()
                new_pose.header = poses[0].header
                new_pose.pose.position.x = point[0]
                new_pose.pose.position.y = point[1]
                
                # Calculate orientation (direction to next point)
                if i < len(points) - 1:
                    dx = points[i+1][0] - point[0]
                    dy = points[i+1][1] - point[1]
                    theta = math.atan2(dy, dx)
                else:
                    # For the last point, use the orientation from the previous point
                    dx = point[0] - points[i-1][0]
                    dy = point[1] - points[i-1][1]
                    theta = math.atan2(dy, dx)
                
                # Convert theta to quaternion (simplified for 2D)
                new_pose.pose.orientation.z = math.sin(theta / 2)
                new_pose.pose.orientation.w = math.cos(theta / 2)
                
                smoothed_poses.append(new_pose)
                
            return smoothed_poses
        
        # Apply direct collocation smoothing technique
        try:
            smoothed_points = self.smooth_path_collocation(points)
            self.get_logger().info(f"Path smoothed using collocation: {len(points)} to {len(smoothed_points)} points")
            
            # Create PoseStamped messages from the smoothed points
            smoothed_poses = []
            for i, point in enumerate(smoothed_points):
                new_pose = PoseStamped()
                new_pose.header = poses[0].header
                new_pose.pose.position.x = point[0]
                new_pose.pose.position.y = point[1]
                
                # Calculate orientation based on path direction
                if i < len(smoothed_points) - 1:
                    dx = smoothed_points[i+1][0] - point[0]
                    dy = smoothed_points[i+1][1] - point[1]
                    theta = math.atan2(dy, dx)
                else:
                    # For the last point, use the orientation from the previous point
                    dx = point[0] - smoothed_points[i-1][0]
                    dy = point[1] - smoothed_points[i-1][1]
                    theta = math.atan2(dy, dx)
                
                # Convert theta to quaternion (simplified for 2D)
                new_pose.pose.orientation.z = math.sin(theta / 2)
                new_pose.pose.orientation.w = math.cos(theta / 2)
                
                smoothed_poses.append(new_pose)
            
            return smoothed_poses
        
        except Exception as e:
            self.get_logger().error(f"Error in path smoothing: {e}")
            return poses

    def simplify_path(self, points, tolerance):
        """
        Simplifies the path using the Ramer-Douglas-Peucker algorithm.
        
        Args:
            points: List of [x, y] coordinates
            tolerance: Tolerance for simplification
            
        Returns:
            List of simplified [x, y] coordinates
        """
        if len(points) < 3:
            return points

        # Recursive function to simplify the path
        def recursive_simplify(start, end):
            if end <= start + 1:
                return [points[start], points[end]] if end > start else [points[start]]

            # Calculate the line segment from start to end
            line_vec = np.array(points[end]) - np.array(points[start])
            line_len = np.linalg.norm(line_vec)
            line_unit_vec = line_vec / line_len if line_len > 0 else np.zeros(2)

            # Calculate the distance from each point to the line segment
            distances = []
            for i in range(start + 1, end):
                point_vec = np.array(points[i]) - np.array(points[start])
                projection_length = np.dot(point_vec, line_unit_vec)
                projection = projection_length * line_unit_vec
                distance = np.linalg.norm(point_vec - projection)
                distances.append((distance, i))

            # Find the point with the maximum distance
            max_distance, index = max(distances, key=lambda x: x[0])

            if max_distance > tolerance:
                # Recursively simplify the segments
                return (recursive_simplify(start, index)[:-1] + 
                        recursive_simplify(index, end))
            else:
                return [points[start], points[end]]

        return recursive_simplify(0, len(points) - 1)

    def auto_tune_pid(self):
        """
        Automatically tunes the PID gains using a simple simulation of a step response.
        A basic integrator model is used to simulate the system's response.
        The cost function is defined as the integrated squared error over a fixed time horizon.
        """
        # --- Tune Linear Controller Gains ---
        self.get_logger().info("Starting PID auto-tuning for linear controller...")
        initial_guess_linear = [self.Kp_linear, self.Ki_linear, self.Kd_linear]
        bounds_linear = [(0.0, 10.0), (0.0, 10.0), (0.0, 10.0)]
        res_linear = minimize(
            lambda params: self.simulate_pid(params[0], params[1], params[2], setpoint=1.0, dt=self.dt, T=5.0),
            initial_guess_linear, bounds=bounds_linear, method='SLSQP'
        )
        if res_linear.success:
            self.Kp_linear, self.Ki_linear, self.Kd_linear = res_linear.x
            self.get_logger().info(f"Linear PID tuned: Kp={self.Kp_linear:.3f}, Ki={self.Ki_linear:.3f}, Kd={self.Kd_linear:.3f}")
        else:
            self.get_logger().warn("Auto-tuning for linear PID failed. Retaining default gains.")
        
        # --- Tune Angular Controller Gains ---
        self.get_logger().info("Starting PID auto-tuning for angular controller...")
        initial_guess_angular = [self.Kp_angular, self.Ki_angular, self.Kd_angular]
        bounds_angular = [(0.0, 10.0), (0.0, 10.0), (0.0, 10.0)]
        res_angular = minimize(
            lambda params: self.simulate_pid(params[0], params[1], params[2], setpoint=0.5, dt=self.dt, T=5.0),
            initial_guess_angular, bounds=bounds_angular, method='SLSQP'
        )
        if res_angular.success:
            self.Kp_angular, self.Ki_angular, self.Kd_angular = res_angular.x
            self.get_logger().info(f"Angular PID tuned: Kp={self.Kp_angular:.3f}, Ki={self.Ki_angular:.3f}, Kd={self.Kd_angular:.3f}")
        else:
            self.get_logger().warn("Auto-tuning for angular PID failed. Retaining default gains.")

    def simulate_pid(self, Kp, Ki, Kd, setpoint, dt, T):
        """
        Simulates a simple PID-controlled system with an integrator model.
        Args:
            Kp, Ki, Kd: PID gains.
            setpoint: Desired value (step input).
            dt: Time step.
            T: Total simulation time.
        Returns:
            Integrated squared error (cost) over the simulation.
        """
        n_steps = int(T / dt)
        x = 0.0
        integral = 0.0
        prev_error = setpoint - x
        cost = 0.0

        for _ in range(n_steps):
            error = setpoint - x
            integral += error * dt
            derivative = (error - prev_error) / dt
            u = Kp * error + Ki * integral + Kd * derivative
            # Simple integrator system: next state is current state plus control effort times dt
            x += u * dt
            cost += error**2 * dt
            prev_error = error
        return cost

    def path_callback(self, msg: Path):
        """
        Updates the target path when a new message is received.
        """
        self.original_path = msg.poses  # Save the original path
        
        if self.enable_smoothing:
            self.path = self.smooth_path(msg.poses)  # Apply smoothing and simplification
        else:
            self.path = msg.poses  # Use original path
            
        self.current_waypoint_index = 0  # Reset to the first waypoint
        self.get_logger().info(f"Received new path with {len(self.original_path)} waypoints, processed to {len(self.path)} waypoints.")
        self.start_time = time.time()
        self.end_time = 0

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

    def control_loop_callback(self):
        """
        Periodic control loop callback. Computes PID control and publishes velocity commands.
        """
        # Thresholds
        self.waypoint_threshold = 0.1  # Distance threshold to consider a waypoint reached
        self.orientation_threshold = 0.05  # Orientation threshold for final alignment

        if not self.is_odom_received or self.path is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # Check if all waypoints are reached
        if self.current_waypoint_index >= len(self.path):
            self.get_logger().info(f"----- All waypoints reached. Stopping. ---- ")
            self.get_logger().info(f"Sum of error: {self.sum_derivate:.3f}, Cost time: {self.end_time - self.start_time:.3f}s.")
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            if self.end_time < 1e-3:
                self.end_time = time.time()
            return

        # Get the current target waypoint
        target_pose = self.path[self.current_waypoint_index].pose
        x_target = target_pose.position.x
        y_target = target_pose.position.y

        # Convert quaternion to yaw (theta) for the target orientation
        q = target_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        orientation_target = math.atan2(siny_cosp, cosy_cosp)

        # 1) Compute errors
        error_x = x_target - self.current_x
        error_y = y_target - self.current_y
        error_theta = self.normalize_angle(math.atan2(error_y, error_x) - self.current_orientation)
        error_orientation = self.normalize_angle(orientation_target - self.current_orientation)

        # 2) Compute derivative of errors
        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y
        derivative_theta = error_theta - self.prev_error_theta
        self.sum_derivate += math.fabs(derivative_x) + math.fabs(derivative_y) + math.fabs(derivative_theta) / 180.0 * math.pi

        # 3) Update integral errors for PID controller
        self.integral_error_x += error_x * self.dt
        self.integral_error_y += error_y * self.dt
        self.integral_error_theta += error_theta * self.dt
        
        # Apply anti-windup (limit the integral term)
        self.integral_error_x = max(min(self.integral_error_x, self.max_integral_error), -self.max_integral_error)
        self.integral_error_y = max(min(self.integral_error_y, self.max_integral_error), -self.max_integral_error)
        self.integral_error_theta = max(min(self.integral_error_theta, self.max_integral_error), -self.max_integral_error)
        
        # Reset integral when near a waypoint for smoother transitions
        distance_to_waypoint = math.hypot(error_x, error_y)
        if distance_to_waypoint < self.waypoint_threshold * 1.5:
            self.integral_error_x = 0.0
            self.integral_error_y = 0.0
            self.integral_error_theta = 0.0

        # 4) PID control for linear and angular velocities
        vx = self.Kp_linear * error_x + self.Ki_linear * self.integral_error_x + self.Kd_linear * derivative_x
        vy = self.Kp_linear * error_y + self.Ki_linear * self.integral_error_y + self.Kd_linear * derivative_y
        vtheta = self.Kp_angular * error_theta + self.Ki_angular * self.integral_error_theta + self.Kd_angular * derivative_theta

        # 5) Update previous error terms
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_theta = error_theta

        # 6) Publish velocity commands
        twist_msg = Twist()

        # Check if the robot has reached the current waypoint
        distance_to_waypoint = math.hypot(error_x, error_y)
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_index += 1  # Move to the next waypoint
        else:
            # Move toward the current waypoint
            if abs(error_theta) > 1.0 and distance_to_waypoint > self.waypoint_threshold:
                twist_msg.angular.z = min(vtheta, self.max_angular_vel)
                self.get_logger().info("Rotating before moving forward (using PID controller)")
            else:
                twist_msg.linear.x = min(math.hypot(vx, vy), self.max_linear_vel)
                twist_msg.angular.z = min(vtheta, self.max_angular_vel)
                self.get_logger().info("Moving forward (using PID controller)")

        self.cmd_vel_pub.publish(twist_msg)

    def normalize_angle(self, angle):
        """
        Normalize an angle to the range [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


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
