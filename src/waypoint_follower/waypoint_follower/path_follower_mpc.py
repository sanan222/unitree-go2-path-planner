#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time
import cvxpy as cp  # Make sure CVXPY is installed

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

    # NOTE: STUDENTS ARE ALLOWED TO ADD THEIR OWN FUNCTIONS
    def customized_functions(self):
        pass

    # NOTE: CAN CHANGE
    def setup_parameters(self):
        # Maximum velocities
        self.max_linear_vel = 1.0  # meter/s
        self.max_angular_vel = 0.8 # rad/s

        # PD Controller Gains (kept for fallback, but not used here)
        self.Kp_linear = 1.0
        self.Kd_linear = 0.1
        self.Kp_angular = 1.0
        self.Kd_angular = 0.1

        # === MPC parameters ===
        self.dt = 0.1    # Time step (s)
        self.N = 15      # Reduced from 20 for better computational performance

        # Weights for the cost function (tune these based on performance)
        self.Q_x = 10.0   # Increased from 1.0 to emphasize position tracking
        self.Q_y = 10.0   # Increased from 1.0 to emphasize position tracking
        self.Q_theta = 5.0  # Increased from 0.5 to improve orientation tracking at turns
        self.R_v = 0.1   # Reduced from 0.5 to allow more aggressive velocity control
        self.R_omega = 0.1  # Reduced from 0.5 to allow more aggressive turning

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

    # === New MPC control function ===
    def mpc_control(self, x_ref, y_ref, theta_ref):
        """
        Implements a linearized MPC controller to track the reference waypoint.
        Uses a linearized unicycle model with state: [x, y, theta] and inputs: [v, omega].
        """
        # Define optimization variables for the prediction horizon
        v = cp.Variable(self.N)
        omega = cp.Variable(self.N)
        x = cp.Variable(self.N + 1)
        y = cp.Variable(self.N + 1)
        theta = cp.Variable(self.N + 1)

        cost = 0
        constraints = []

        # Set initial state constraints using current robot state
        constraints += [
            x[0] == self.current_x,
            y[0] == self.current_y,
            theta[0] == self.current_orientation
        ]

        # Current values for linearization
        current_theta = self.current_orientation
        
        # Build the optimization problem over the prediction horizon
        for k in range(self.N):
            # Linearized discrete-time unicycle dynamics
            # We linearize around the current heading for the first step
            # and use the predicted heading for subsequent steps
            if k == 0:
                cos_theta = math.cos(current_theta)
                sin_theta = math.sin(current_theta)
            else:
                # For later steps, we use a simpler linear model
                # which is less accurate but still works for short horizons
                cos_theta = math.cos(theta_ref)
                sin_theta = math.sin(theta_ref)
            
            constraints += [
                x[k+1] == x[k] + v[k] * cos_theta * self.dt,
                y[k+1] == y[k] + v[k] * sin_theta * self.dt,
                theta[k+1] == theta[k] + omega[k] * self.dt
            ]
            
            # Accumulate cost: state tracking error + control effort
            cost += (self.Q_x * cp.square(x[k+1] - x_ref) +
                     self.Q_y * cp.square(y[k+1] - y_ref) +
                     self.Q_theta * cp.square(theta[k+1] - theta_ref))
            cost += self.R_v * cp.square(v[k]) + self.R_omega * cp.square(omega[k])
            
            # Input constraints
            constraints += [
                cp.abs(v[k]) <= self.max_linear_vel,
                cp.abs(omega[k]) <= self.max_angular_vel
            ]

        # Formulate and solve the optimization problem
        prob = cp.Problem(cp.Minimize(cost), constraints)
        try:
            prob.solve(solver=cp.OSQP)
        except Exception as e:
            self.get_logger().error(f"MPC optimization error: {e}")
            return 0.0, 0.0  # fallback commands

        if prob.status not in ["infeasible", "unbounded"]:
            v0 = v.value[0]
            omega0 = omega.value[0]
        else:
            self.get_logger().warn("MPC problem infeasible or unbounded, using safe fallback commands.")
            v0, omega0 = 0.0, 0.0

        return v0, omega0

    # NOTE: CAN CHANGE
    def control_loop_callback(self):
        """
        Periodic control loop callback. Computes MPC control and publishes velocity commands.
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

        # Check if the waypoint is reached
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_index += 1  # Move to the next waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index - 1}. Moving to the next one.")
        else:
            # Call the MPC controller to compute control commands
            v_cmd, omega_cmd = self.mpc_control(x_target, y_target, orientation_target)
            twist_msg.linear.x = v_cmd
            twist_msg.angular.z = omega_cmd
            self.get_logger().info("Applying MPC control")

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
