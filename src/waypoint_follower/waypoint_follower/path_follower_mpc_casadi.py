#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time
import numpy as np

# Replace cvxpy with casadi for true nonlinear optimization
import casadi as ca

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped


class PathFollower(Node):
    """
    A ROS2 node that uses an MPC controller to follow a path (list of waypoints).
    Subscribes to /planner/path for the target path and a specified odometry topic for the robot's current state,
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
        self.waypoint_threshold = 0.15  # Distance threshold to consider a waypoint reached
        self.orientation_threshold = 0.1  # Orientation threshold for final alignment

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

    # NOTE: STUDENTS ARE ALLOWED TO ADD THEIR OWN FUNCTIONS
    def customized_functions(self):
        pass

    # NOTE: CAN CHANGE
    def setup_parameters(self):
        # Maximum velocities
        self.max_linear_vel = 1.0  # meter/s
        self.max_angular_vel = 0.8 # rad/s

        # PD Controller Gains (kept for fallback)
        self.Kp_linear = 1.0
        self.Kd_linear = 0.1
        self.Kp_angular = 1.0
        self.Kd_angular = 0.1

        # === Nonlinear MPC parameters ===
        self.dt = 0.1    # Time step (s)
        self.N = 10      # Prediction horizon

        # Weights for the cost function - modified for better performance
        self.Q_x = 30.0       # Increased position tracking weight (x)
        self.Q_y = 30.0       # Increased position tracking weight (y)
        self.Q_theta = 5.0    # Reduced orientation tracking weight
        self.R_v = 5.0        # Reduced control effort weight
        self.R_omega = 5.0    # Reduced control effort weight
        
        # Terminal weights
        self.Q_x_terminal = 50.0
        self.Q_y_terminal = 50.0
        self.Q_theta_terminal = 10.0  # Reduced terminal orientation weight
        
        # Thresholds
        self.waypoint_threshold = 0.40  # Increased from 0.1 to 0.15
        self.orientation_threshold = 0.30  # Relaxed orientation threshold
        
        # Hysteresis for waypoint transitions to prevent oscillation
        self.waypoint_reached_flag = False
        
        # Initialize CasADi solver (done once for efficiency)
        self.setup_nlp_solver()

    def setup_nlp_solver(self):
        """
        Set up the nonlinear programming (NLP) solver using CasADi.
        This creates a parametric optimization problem that can be solved efficiently.
        """
        # State variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.size1()
        
        # Control inputs
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        n_controls = controls.size1()
        
        # RHS of the ODE (system dynamics)
        rhs = ca.vertcat(
            v * ca.cos(theta),  # dx/dt = v * cos(theta)
            v * ca.sin(theta),  # dy/dt = v * sin(theta)
            omega               # dtheta/dt = omega
        )
        
        # Create the dynamics function
        f = ca.Function('f', [states, controls], [rhs])
        
        # Decision variables
        U = ca.SX.sym('U', n_controls, self.N)  # Control inputs over the horizon
        X = ca.SX.sym('X', n_states, self.N+1)  # States over the horizon
        P = ca.SX.sym('P', n_states + n_states)  # Parameters: [initial_state, reference_state]
        
        # Cost function
        obj = 0
        
        # Define the cost function and constraints
        for k in range(self.N):
            # State tracking cost
            state_error = X[:, k+1] - P[n_states:2*n_states]
            
            # Special handling for angle differences
            angle_error = state_error[2]
            # Normalize angle difference in CasADi (using sin/cos)
            normalized_angle_error = ca.atan2(ca.sin(angle_error), ca.cos(angle_error))
            state_error = ca.vertcat(state_error[0], state_error[1], normalized_angle_error)
            
            if k < self.N - 1:
                obj += self.Q_x * state_error[0]**2
                obj += self.Q_y * state_error[1]**2
                obj += self.Q_theta * state_error[2]**2
            else:
                # Terminal cost
                obj += self.Q_x_terminal * state_error[0]**2
                obj += self.Q_y_terminal * state_error[1]**2
                obj += self.Q_theta_terminal * state_error[2]**2
            
            # Control cost
            obj += self.R_v * U[0, k]**2
            obj += self.R_omega * U[1, k]**2
            
            # Control rate penalties
            if k > 0:
                obj += 1.0 * (U[0, k] - U[0, k-1])**2
                obj += 1.0 * (U[1, k] - U[1, k-1])**2
        
        # Formulate the NLP
        OPT_variables = ca.vertcat(
            X.reshape((-1, 1)),  # State variables as decision variables
            U.reshape((-1, 1))   # Control inputs as decision variables
        )
        
        nlp_prob = {
            'f': obj,                     # Objective function
            'x': OPT_variables,           # Decision variables
            'p': P,                       # Parameters
            'g': ca.vertcat(*[])          # Constraints (will be added)
        }
        
        constraints = []
        lbg = []  # Lower bounds of constraints
        ubg = []  # Upper bounds of constraints
        
        # Initial condition constraints
        constraints.append(X[:, 0] - P[:n_states])
        lbg.extend([0.0, 0.0, 0.0])
        ubg.extend([0.0, 0.0, 0.0])
        
        # Add dynamics constraints for state propagation
        for k in range(self.N):
            # Discretize using Euler integration
            x_next = X[:, k] + self.dt * f(X[:, k], U[:, k])
            constraints.append(X[:, k+1] - x_next)
            lbg.extend([0.0, 0.0, 0.0])
            ubg.extend([0.0, 0.0, 0.0])
        
        nlp_prob['g'] = ca.vertcat(*constraints)
        
        # Options for IPOPT solver
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'ipopt.max_iter': 100,
            'print_time': 0
        }
        
        # Create the solver
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        
        # Prepare for reuse
        self.lbg = lbg
        self.ubg = ubg
        self.n_states = n_states
        self.n_controls = n_controls
        self.X = X
        self.U = U
        self.P = P
    
    def casadi_mpc_control(self, x_ref, y_ref, theta_ref):
        """
        Solve the nonlinear MPC problem using CasADi.
        Returns optimal control actions.
        """
        # Initial state and reference state parameters
        p = np.zeros(2 * self.n_states)
        p[:self.n_states] = [self.current_x, self.current_y, self.current_orientation]
        p[self.n_states:] = [x_ref, y_ref, theta_ref]
        
        # Initial guess for optimization variables
        x0 = np.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        
        # Control input bounds
        lbx = np.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        ubx = np.zeros((self.n_states*(self.N+1) + self.n_controls*self.N, 1))
        
        # State bounds (no specific bounds except for angles)
        for i in range(self.N+1):
            lbx[i*self.n_states:(i+1)*self.n_states, 0] = [-np.inf, -np.inf, -np.inf]
            ubx[i*self.n_states:(i+1)*self.n_states, 0] = [np.inf, np.inf, np.inf]
        
        # Control bounds
        for i in range(self.N):
            lbx[self.n_states*(self.N+1) + i*self.n_controls:self.n_states*(self.N+1) + (i+1)*self.n_controls, 0] = \
                [-self.max_linear_vel, -self.max_angular_vel]
            ubx[self.n_states*(self.N+1) + i*self.n_controls:self.n_states*(self.N+1) + (i+1)*self.n_controls, 0] = \
                [self.max_linear_vel, self.max_angular_vel]
        
        # Solve the NLP
        try:
            sol = self.solver(
                x0=x0,
                lbx=lbx,
                ubx=ubx,
                lbg=self.lbg,
                ubg=self.ubg,
                p=p
            )
            
            # Extract the control inputs
            u_opt = sol['x'][self.n_states*(self.N+1):self.n_states*(self.N+1) + self.n_controls].full().flatten()
            
            v_cmd = float(u_opt[0])
            omega_cmd = float(u_opt[1])
            
            self.get_logger().info(f"True Nonlinear MPC solution found: v={v_cmd:.2f}, omega={omega_cmd:.2f}")
            return v_cmd, omega_cmd
            
        except Exception as e:
            self.get_logger().error(f"Nonlinear MPC optimization failed: {e}")
            return self.pid_fallback(x_ref, y_ref, theta_ref)

    def pid_fallback(self, x_ref, y_ref, theta_ref):
        """
        Simple PID controller as a fallback when MPC fails to converge
        """
        # Calculate position error
        error_x = x_ref - self.current_x
        error_y = y_ref - self.current_y
        distance = math.sqrt(error_x**2 + error_y**2)
        
        # Calculate angle to target
        target_angle = math.atan2(error_y, error_x)
        angle_error = self.normalize_angle(target_angle - self.current_orientation)
        
        # Calculate heading error (for final orientation)
        heading_error = self.normalize_angle(theta_ref - self.current_orientation)
        
        # Derivative terms
        d_error_x = (error_x - self.prev_error_x) / self.dt
        d_error_y = (error_y - self.prev_error_y) / self.dt
        
        # Update previous errors
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        
        # Linear velocity based on distance and alignment
        v = self.Kp_linear * distance + self.Kd_linear * (d_error_x * math.cos(self.current_orientation) + 
                                                          d_error_y * math.sin(self.current_orientation))
        
        # Use angle error for path following, but transition to heading error when close
        if distance < 0.3:
            # Blend between path following and final orientation
            blend = max(0, min(1, (0.3 - distance) / 0.3))
            angle_to_use = (1 - blend) * angle_error + blend * heading_error
        else:
            angle_to_use = angle_error
        
        # Angular velocity
        omega = self.Kp_angular * angle_to_use
        
        # Apply limits
        v = max(-self.max_linear_vel, min(v, self.max_linear_vel))
        omega = max(-self.max_angular_vel, min(omega, self.max_angular_vel))
        
        return v, omega

    # NOTE: CAN CHANGE
    def path_callback(self, msg: Path):
        """
        Updates the target path when a new message is received.
        """
        self.path = msg.poses  # List of PoseStamped messages
        self.current_waypoint_index = 0  # Reset to the first waypoint
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints.")
        self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.end_time = 0

    # NOTE: CAN CHANGE
    def control_loop_callback(self):
        """
        Periodic control loop callback. Computes nonlinear MPC control and publishes velocity commands.
        """
        if self.check_ending():
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

        # Compute distance to the current waypoint
        error_x = x_target - self.current_x
        error_y = y_target - self.current_y
        distance_to_waypoint = math.hypot(error_x, error_y)

        twist_msg = Twist()

        # Improved waypoint transition logic with hysteresis
        if not self.waypoint_reached_flag and distance_to_waypoint < self.waypoint_threshold:
            # We've just entered the waypoint radius
            self.waypoint_reached_flag = True
            self.get_logger().info(f"Entered waypoint {self.current_waypoint_index} radius.")
        
        elif self.waypoint_reached_flag and distance_to_waypoint < self.waypoint_threshold * 0.8:
            # We're well within the waypoint - move to the next one
            self.current_waypoint_index += 1
            self.waypoint_reached_flag = False  # Reset flag for next waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index - 1}. Moving to the next one.")
        
        # Check if we've moved away from waypoint without properly reaching it
        elif self.waypoint_reached_flag and distance_to_waypoint > self.waypoint_threshold * 1.2:
            # We've exited the waypoint radius without properly reaching it
            self.waypoint_reached_flag = False
            self.get_logger().info(f"Exited waypoint {self.current_waypoint_index} radius without properly reaching it.")

        # Compute and apply control
        # Call the true nonlinear MPC controller to compute control commands
        v_cmd, omega_cmd = self.casadi_mpc_control(x_target, y_target, orientation_target)
        
        # Apply a minimum velocity to prevent getting stuck
        if abs(v_cmd) < 0.05 and distance_to_waypoint > 0.05:
            v_cmd = 0.05 * (1.0 if v_cmd >= 0 else -1.0)
        
        twist_msg.linear.x = v_cmd
        twist_msg.angular.z = omega_cmd
        self.get_logger().info(f"Applying nonlinear MPC control - dist: {distance_to_waypoint:.3f}")

        self.cmd_vel_pub.publish(twist_msg)


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
