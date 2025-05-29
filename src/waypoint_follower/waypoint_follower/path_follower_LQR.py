# python3 src/waypoint_follower/waypoint_follower/path_follower.py /custom_odometry_topic

#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time
import numpy as np
import scipy.linalg as la

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped
# from utils.angle import angle_mod

class State:
    def __init__(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v


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

    # NOTE: STUDENTS ARE ALLOWED TO ADD THEIR OWN FUNCTIONS

    def update(self, state, a, delta):

        if delta >= self.max_steer:
            delta = self.max_steer
        if delta <= - self.max_steer:
            delta = - self.max_steer

        state.x = state.x + state.v * math.cos(state.yaw) * self.dt
        state.y = state.y + state.v * math.sin(state.yaw) * self.dt
        state.yaw = state.yaw + state.v / self.L * math.tan(delta) * self.dt
        state.v = state.v + a * self.dt

        return state
    
    def solve_dare(self, A, B, Q, R):
        """
        solve a discrete time Algebraic Riccati equation (DARE)
        """
        x = Q
        x_next = Q
        max_iter = 150
        eps = 0.01

        for i in range(max_iter):
            x_next = A.T @ x @ A - A.T @ x @ B @ \
                    la.inv(R + B.T @ x @ B) @ B.T @ x @ A + Q
            if (abs(x_next - x)).max() < eps:
                break
            x = x_next

        return x_next
    
    def dlqr(self,A, B, Q, R):
        """Solve the discrete time lqr controller.
        x[k+1] = A x[k] + B u[k]
        cost = sum x[k].T*Q*x[k] + u[k].T*R*u[k]
        # ref Bertsekas, p.151
        """

        # first, try to solve the ricatti equation
        X = self.solve_dare(A, B, Q, R)

        # compute the LQR gain
        K = la.inv(B.T @ X @ B + R) @ (B.T @ X @ A)

        eig_result = la.eig(A - B @ K)

        return K, X, eig_result[0]
    
    def lqr_speed_steering_control(self, state, cx, cy, cyaw, ck, pe, pth_e, sp, Q, R):
        ind, e = self.calc_nearest_index(state, cx, cy, cyaw)

        tv = sp[ind]

        k = ck[ind]
        v = state.v
        th_e = self.normalize_angle(state.yaw - cyaw[ind])

        # A = [1.0, dt, 0.0, 0.0, 0.0
        #      0.0, 0.0, v, 0.0, 0.0]
        #      0.0, 0.0, 1.0, dt, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 0.0]
        #      0.0, 0.0, 0.0, 0.0, 1.0]
        A = np.zeros((5, 5))
        A[0, 0] = 1.0
        A[0, 1] = self.dt
        A[1, 2] = v
        A[2, 2] = 1.0
        A[2, 3] = self.dt
        A[4, 4] = 1.0

        # B = [0.0, 0.0
        #     0.0, 0.0
        #     0.0, 0.0
        #     v/L, 0.0
        #     0.0, dt]
        B = np.zeros((5, 2))
        B[3, 0] = v / self.L
        B[4, 1] = self.dt

        K, _, _ = self.dlqr(A, B, Q, R)

        # state vector
        # x = [e, dot_e, th_e, dot_th_e, delta_v]
        # e: lateral distance to the path
        # dot_e: derivative of e
        # th_e: angle difference to the path
        # dot_th_e: derivative of th_e
        # delta_v: difference between current speed and target speed
        x = np.zeros((5, 1))
        x[0, 0] = e
        x[1, 0] = (e - pe) / self.dt
        x[2, 0] = th_e
        x[3, 0] = (th_e - pth_e) / self.dt
        x[4, 0] = v - tv

        # input vector
        # u = [delta, accel]
        # delta: steering angle
        # accel: acceleration
        ustar = -K @ x

        # calc steering input
        ff = math.atan2(self.L * k, 1)  # feedforward steering angle
        fb = self.normalize_angle(ustar[0, 0])  # feedback steering angle
        delta = ff + fb

        # calc accel input
        accel = ustar[1, 0]

        return delta, ind, e, th_e, accel
    
    def calc_nearest_index(self, state, cx, cy, cyaw):
        dx = [state.x - icx for icx in cx]
        dy = [state.y - icy for icy in cy]

        d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

        mind = min(d)

        ind = d.index(mind)

        mind = math.sqrt(mind)

        dxl = cx[ind] - state.x
        dyl = cy[ind] - state.y

        angle = self.normalize_angle(cyaw[ind] - math.atan2(dyl, dxl))
        if angle < 0:
            mind *= -1

        return ind, mind
    
    def calc_speed_profile(self, cyaw, target_speed):
        speed_profile = [target_speed] * len(cyaw)

        direction = 1.0

        # Set stop point
        for i in range(len(cyaw) - 1):
            dyaw = abs(cyaw[i + 1] - cyaw[i])
            switch = math.pi / 4.0 <= dyaw < math.pi / 2.0

            if switch:
                direction *= -1

            if direction != 1.0:
                speed_profile[i] = -target_speed
            else:
                speed_profile[i] = target_speed

            if switch:
                speed_profile[i] = 0.0

        # speed down
        for i in range(min(40, len(speed_profile))):  # Ensure we don't exceed the list length
            speed_profile[-(i + 1)] = target_speed / (50 - i)  # Use -(i + 1) to access from the end
            if speed_profile[-(i + 1)] <= 1.0 / 3.6:
                speed_profile[-(i + 1)] = 1.0 / 3.6

        return speed_profile
    

    # NOTE: CAN CHANGE
    def setup_parameters(self):
        # Maximum velocities
        self.max_linear_vel = 1.0  # meter
        self.max_angular_vel = 0.8  # rad

        # LQR parameters
        self.lqr_Q = np.diag([50.0, 50.0, 40.0, 40.0, 40.0])  # Adjusted weights for state variables
        self.lqr_R = np.diag([50.0, 50.0])  # Adjusted weights for control inputs
        self.dt = 0.1  # time tick[s]
        self.L = 0.5  # Wheel base of the vehicle [m]
        self.max_steer = np.deg2rad(90.0)  # maximum steering angle[rad]


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
        Periodic control loop callback. Computes LQR control and publishes velocity commands.
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

        # Convert path to arrays for LQR controller
        cx = []  # x-coordinates of path
        cy = []  # y-coordinates of path
        cyaw = []  # yaw angles of path
        
        # Extract path points starting from current waypoint
        for i in range(self.current_waypoint_index, len(self.path)):
            pose = self.path[i].pose
            cx.append(pose.position.x)
            cy.append(pose.position.y)
            
            # Extract orientation
            q = pose.orientation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            yaw = math.atan2(siny_cosp, cosy_cosp)
            cyaw.append(yaw)

        # Ensure we have at least 2 points
        if len(cx) < 2:
            self.get_logger().warning("Not enough path points for LQR control")
            twist_msg = Twist()
            self.cmd_vel_pub.publish(twist_msg)
            return

        # Calculate path curvature
        ck = [0] * len(cx)  # Initialize with zeros
        for i in range(len(cx) - 1):
            dx = cx[i+1] - cx[i]
            dy = cy[i+1] - cy[i]
            distance = math.sqrt(dx**2 + dy**2)
            if distance > 0.01:  # Avoid division by very small numbers
                dyaw = self.normalize_angle(cyaw[i+1] - cyaw[i])
                ck[i] = dyaw / distance
        # Last point has same curvature as second-to-last
        if len(ck) > 1:
            ck[-1] = ck[-2]

        # Calculate speed profile
        target_speed = self.max_linear_vel
        sp = self.calc_speed_profile(cyaw, target_speed)
        
        # Create state object for current robot state
        state = State(
            x=self.current_x,
            y=self.current_y,
            yaw=self.current_orientation,
            v=0.1  # Initial velocity approximation
        )
        
        # LQR control
        delta, target_idx, e, th_e, ai = self.lqr_speed_steering_control(
            state, cx, cy, cyaw, ck, 
            self.prev_error_x, self.prev_error_theta, 
            sp, self.lqr_Q, self.lqr_R
        )
        
        # Update state with calculated control inputs
        updated_state = self.update(state, ai, delta)
        
        # Calculate velocities for Twist message
        vx = updated_state.v * math.cos(updated_state.yaw)
        vy = updated_state.v * math.sin(updated_state.yaw)
        vtheta = updated_state.v * math.tan(delta) / self.L
        
        
        # Check if the robot has reached the current waypoint
        error_x = x_target - self.current_x
        error_y = y_target - self.current_y
        error_theta = self.normalize_angle(math.atan2(error_y, error_x) - self.current_orientation) # degree
        error_orientation = self.normalize_angle(orientation_target - self.current_orientation)

        # Compute derivative of errors (for tracking performance metrics)
        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y
        derivative_theta = error_theta - self.prev_error_theta
        self.sum_derivate += math.fabs(derivative_x) + math.fabs(derivative_y) + math.fabs(derivative_theta) / 180.0 * math.pi
        
        # Update previous error terms
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_theta = error_theta

        # Create Twist message
        twist_msg = Twist()

        # Check if the robot has reached the current waypoint
        distance_to_waypoint = math.hypot(error_x, error_y)
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_index += 1  # Move to the next waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index - 1}. Moving to the next one.")
        else:
            # Move toward the current waypoint
            if abs(error_theta) > 1.0 and distance_to_waypoint > self.waypoint_threshold:
                twist_msg.angular.z = min(vtheta, self.max_angular_vel)
                self.get_logger().info("Rotating before moving forward")
            else:
                twist_msg.linear.x = min(math.hypot(vx, vy), self.max_linear_vel)
                twist_msg.angular.z = min(vtheta, self.max_angular_vel)
                self.get_logger().info("Moving forward")

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