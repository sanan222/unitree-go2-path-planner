#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

# ---------------- Helper Functions ----------------

def ccw(A, B, C):
    """Returns True if the points A, B, and C are arranged in a counterclockwise order."""
    return (C[1]-A[1])*(B[0]-A[0]) > (B[1]-A[1])*(C[0]-A[0])

def segments_intersect(A, B, C, D):
    """Returns True if line segments AB and CD intersect."""
    return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

def smooth_edges(points, min_edge_length=0.1, angle_threshold=0.25):
    """Smooth edges by removing points that are too close together."""
    if len(points) < 3:
        return points
    smoothed = [points[0]]
    for i in range(1, len(points)-1):
        prev_vector = points[i] - points[i-1]
        next_vector = points[i+1] - points[i]
        if np.linalg.norm(prev_vector) < min_edge_length or np.linalg.norm(next_vector) < min_edge_length:
            continue
        prev_dir = prev_vector / np.linalg.norm(prev_vector)
        next_dir = next_vector / np.linalg.norm(next_vector)
        angle = np.arccos(np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0))
        if angle > angle_threshold:
            smoothed.append(points[i])
    smoothed.append(points[-1])
    return smoothed

def curvature_filter(points, min_edge_length=0.01, curvature_threshold=0.4):
    """
    Filter points based on local curvature
    Keeps points with high curvature (corners) and removes points with low curvature
    """
    if len(points) < 3:
        return points
    
    # Always keep first and last points
    filtered = [points[0]]
    
    for i in range(1, len(points)-1):
        # Calculate vectors to previous and next points
        prev_vec = points[i] - points[i-1]
        next_vec = points[i+1] - points[i]
        
        # Skip very short edges
        prev_len = np.linalg.norm(prev_vec)
        next_len = np.linalg.norm(next_vec)
        if prev_len < min_edge_length or next_len < min_edge_length:
            continue
        
        # Normalize vectors
        prev_dir = prev_vec / prev_len
        next_dir = next_vec / next_len
        
        # Calculate angle between vectors (curvature)
        dot_product = np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0)
        angle = np.arccos(dot_product)
        
        # Keep point if curvature is high (angle is large)
        if angle > curvature_threshold:
            filtered.append(points[i])
    
    filtered.append(points[-1])  # Add last point
    return filtered

def chaikin_smooth(points, iterations=2):
    """
    Apply Chaikin's corner cutting algorithm to smooth a polyline
    This creates a smoother curve while preserving the general shape
    """
    if len(points) < 3:
        return points
    
    result = np.array(points)
    
    for _ in range(iterations):
        new_points = []
        new_points.append(result[0])  # Keep first point
        
        for i in range(len(result) - 1):
            p0 = result[i]
            p1 = result[i+1]
            
            # Create two new points that are 1/4 and 3/4 along each line segment
            q = p0 + 0.25 * (p1 - p0)
            r = p0 + 0.75 * (p1 - p0)
            
            new_points.append(q)
            new_points.append(r)
        
        new_points.append(result[-1])  # Keep last point
        result = np.array(new_points)
    
    return result

def improved_smooth_edges(points, min_edge_length=0.01, curvature_threshold=0.4, chaikin_iterations=2):
    """
    Improved edge smoothing using curvature-based filtering and Chaikin's algorithm
    This approach preserves important corners while smoothing noise
    """
    if len(points) < 3:
        return points
    
    # First filter points based on curvature
    filtered_points = curvature_filter(points, min_edge_length, curvature_threshold)
    
    # Then apply Chaikin's corner cutting for smoother curves
    smoothed_points = chaikin_smooth(filtered_points, chaikin_iterations)
    
    return smoothed_points

def process_edges(edges, smoothing_mode=0, **kwargs):
    """
    Processes edge data based on the smoothing mode:
      - smoothing_mode == 0: Smooth edges by removing points that are too close together.
      - smoothing_mode == 1: Improved smoothing with curvature filtering and Chaikin's algorithm.
      - smoothing_mode == 5: No smoothing (raw edges).
    """
    if smoothing_mode == 0:
        # Extract points from edges
        points = [edges[0][0]]
        for edge in edges:
            points.append(edge[1])
        points = np.array(points)
        
        # Apply smoothing
        smoothed_points = smooth_edges(points)
        
        # Convert back to edges
        smoothed_edges = []
        for i in range(len(smoothed_points)-1):
            smoothed_edges.append((smoothed_points[i], smoothed_points[i+1]))
        
        return smoothed_edges
    elif smoothing_mode == 1:
        # Extract points from edges
        points = [edges[0][0]]
        for edge in edges:
            points.append(edge[1])
        points = np.array(points)
        
        # Apply improved smoothing
        smoothed_points = improved_smooth_edges(points)
        
        # Convert back to edges
        smoothed_edges = []
        for i in range(len(smoothed_points)-1):
            smoothed_edges.append((smoothed_points[i], smoothed_points[i+1]))
        
        return smoothed_edges
    elif smoothing_mode == 5:
        return edges
    else:
        # For any other mode, default to the new smoothing
        return process_edges(edges, smoothing_mode=0)

# ---------------- Standard Bug1 Algorithm Implementation ----------------

class Bug1Node(Node):
    def __init__(self):
        super().__init__('bug1_node')
        
        # Constants
        self.SAFETY_MARGIN = 0.9     # meters
        self.INCREMENT_DISTANCE = 2.0 # meters
        self.UPDATE_RATE = 0.5        # seconds
        self.GOAL_THRESHOLD = 0.5     # meters, distance to consider goal reached
        self.HIT_POINT_THRESHOLD = 1.5 # meters, distance to consider hit point reached
        self.LEAVE_POINT_THRESHOLD = 1.5 # meters, distance to consider leave point reached
        
        # Threshold for obstacle detection
        self.OBSTACLE_DETECTION_THRESHOLD = 1.0  # meters, distance to consider close to obstacle
        
        # Add constants for robot stoppage detection
        self.MOVEMENT_THRESHOLD = 0.04  # meters, distance to consider robot stopped
        self.STOP_CHECK_TIME = 5.0      # seconds, time to check if robot is stopped
        self.RECOVERY_TIMEOUT = 20.0    # seconds, increased from 5.0 to wait longer before giving up
        
        # Add flag to track if we're recovering from being stuck
        self.recovering_from_stuck = False
        
        # Add variables for specific recovery maneuvers
        self.in_recovery_maneuver = False
        self.recovery_stage = 0  # 0: initial, 1: forward movement, 2: right movement
        self.recovery_waypoint = None
        self.recovery_start_time = None
        
        # Add hit point reset counter
        self.hit_point_reset_count = 0
        self.MAX_HIT_POINT_RESETS = 2  # Maximum number of times hit point can be reset
        
        # Declare parameter "smoothing_mode"
        # 0: both RANSAC and adaptive smoothing,
        # 1: only RANSAC,
        # 2: only adaptive smoothing,
        # 3: only moving average smoothing,
        # 4: angular smoothing,
        # 5: no smoothing.
        self.declare_parameter("smoothing_mode", 0)
        self.smoothing_mode = self.get_parameter("smoothing_mode").value
        
        # Bug1 state machine: "go_to_goal", "follow_boundary_to_hit_point", "follow_boundary_to_leave_point", "goal_reached"
        self.mode = "go_to_goal"  # Initial mode
        
        # Goal position in world frame (to be set externally)
        self.goal_position = None
        
        # Bug1 specific variables
        self.hit_point = None  # Point where obstacle was first encountered
        self.leave_point = None  # Point with minimum distance to goal along boundary
        self.min_dist_to_goal = float('inf')  # Minimum distance to goal found during boundary following
        self.hit_point_found_again = False  # Flag to indicate if we have returned to hit point
        self.completed_full_circuit = False  # Flag to indicate if we have fully circled the obstacle
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False
        self.current_edges = []  # List of (start_point, end_point) tuples
        
        # Add variables for stoppage detection
        self.last_position = None
        self.last_position_time = None
        self.stopped_time = None
        
        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, 'current_waypoint', 10)
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)
        self.hit_point_marker_pub = self.create_publisher(Marker, 'hit_point', 10)
        self.leave_point_marker_pub = self.create_publisher(Marker, 'leave_point', 10)
        self.goal_marker_pub = self.create_publisher(Marker, 'goal_marker', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'Odometry',
            self.odom_callback,
            10)
            
        self.line_sub = self.create_subscription(
            Marker,
            'local_map_lines',
            self.line_callback,
            10)
        
        # Add subscriber for goal messages
        self.goal_sub = self.create_subscription(
            Pose2D,
            'goal',
            self.goal_callback,
            10)
            
        # Timer
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)
        
        # Add a timestamp for boundary following
        self.boundary_following_start_time = None
        self.MIN_BOUNDARY_FOLLOWING_TIME = 20.0  # seconds
        
        self.get_logger().info('Bug1 node initialized, smoothing_mode: {}'.format(self.smoothing_mode))
    
    def goal_callback(self, msg):
        """
        Callback function for handling new goal messages.
        """
        new_goal = np.array([msg.x, msg.y])
        
        # Check if this is truly a different goal to avoid resetting for the same goal
        if self.goal_position is None or not np.array_equal(new_goal, self.goal_position):
            self.goal_position = new_goal
            
            # Reset Bug1 state for new goal
            self.hit_point = None
            self.leave_point = None
            self.min_dist_to_goal = float('inf')
            self.hit_point_found_again = False
            self.completed_full_circuit = False
            self.mode = "go_to_goal"
            self.get_logger().info(f"New goal received: ({msg.x:.2f}, {msg.y:.2f}). Starting navigation.")
    
    def odom_callback(self, msg):
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)
    
    def transform_to_camera_init(self, point):
        """
        Transforms a point from the robot's livox frame to the fixed world (camera_init) frame.
        """
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        x = point[0] * c - point[1] * s + self.current_x
        y = point[0] * s + point[1] * c + self.current_y
        return np.array([x, y])
    
    def transform_to_base_link(self, point):
        """
        Transforms a point from the world (camera_init) frame to the robot's livox frame.
        """
        dx = point[0] - self.current_x
        dy = point[1] - self.current_y
        c = math.cos(-self.current_orientation)
        s = math.sin(-self.current_orientation)
        x = dx * c - dy * s
        y = dx * s + dy * c
        return np.array([x, y])
    
    def line_callback(self, msg):
        """Processes incoming line segments and creates edge segments."""
        if len(msg.points) < 2:
            return
        
        points = [np.array([point.x, point.y]) for point in msg.points]
        raw_edges = [(points[i], points[i + 1]) for i in range(len(points) - 1)]
        
        # Process edges based on the smoothing_mode parameter
        self.current_edges = process_edges(raw_edges, 
                                           smoothing_mode=self.smoothing_mode)
    
    def check_line_of_sight(self):
        """
        Checks if the direct line from the robot's livox frame (0,0) to the goal (transformed from world frame)
        intersects any of the detected edges.
        """
        if not self.current_edges:
            return True
            
        # Robot's livox frame position is (0,0)
        robot_pos = np.array([0.0, 0.0])
        # Transform the world-frame goal to the robot's livox frame
        goal_in_livox = self.transform_to_base_link(self.goal_position)
        
        for edge in self.current_edges:
            A, B = robot_pos, goal_in_livox
            C, D = edge  # Edges are published in livox frame
            if segments_intersect(A, B, C, D):
                return False
        return True

    def dist_to_goal(self, point_in_livox=None):
        """
        Calculates distance from a point (in livox frame) to the goal.
        If no point is provided, calculates distance from robot to goal.
        """
        if self.goal_position is None:
            return float('inf')
            
        if point_in_livox is None:
            point_in_livox = np.array([0.0, 0.0])  # Robot's position in livox frame
            
        goal_in_livox = self.transform_to_base_link(self.goal_position)
        return np.linalg.norm(goal_in_livox - point_in_livox)
    
    def is_near_hit_point(self):
        """Checks if the robot is near the hit point again."""
        if self.hit_point is None:
            return False
            
        hit_point_in_livox = self.transform_to_base_link(self.hit_point)
        # Calculate distance from robot (0,0) to hit point in livox frame
        dist = np.linalg.norm(hit_point_in_livox - np.array([0.0, 0.0]))
        return dist < self.HIT_POINT_THRESHOLD
    
    def is_near_leave_point(self):
        """Checks if the robot is near the leave point."""
        if self.leave_point is None:
            return False
            
        leave_point_in_livox = self.transform_to_base_link(self.leave_point)
        # Calculate distance from robot (0,0) to leave point in livox frame
        dist = np.linalg.norm(leave_point_in_livox - np.array([0.0, 0.0]))
        return dist < self.LEAVE_POINT_THRESHOLD
    
    def is_goal_reached(self):
        """Checks if the robot has reached the goal."""
        dist = self.dist_to_goal()
        if dist < self.GOAL_THRESHOLD:
            self.get_logger().debug(f"Current distance to goal: {dist:.2f}m (threshold: {self.GOAL_THRESHOLD}m)")
            return True
        return False
    
    def find_closest_edge_distance(self):
        """
        Finds the distance to the closest edge from the robot.
        Returns: (min_distance, closest_point) or (None, None) if no edges.
        """
        if not self.current_edges:
            return None, None
            
        robot_pos = np.array([0.0, 0.0])  # Robot position in livox frame
        min_distance = float('inf')
        closest_point = None
        
        for edge in self.current_edges:
            start_point, end_point = edge
            
            # Skip edges that are too far away (more than 3 meters from robot)
            start_distance = np.linalg.norm(start_point - robot_pos)
            end_distance = np.linalg.norm(end_point - robot_pos)
            if start_distance > 1.5 and end_distance > 1.5:
                continue
            
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            if edge_length < 0.01:
                continue
                
            edge_direction = edge_vector / edge_length
            to_robot = robot_pos - start_point
            projection = np.dot(to_robot, edge_direction)
            projection = max(0, min(edge_length, projection))
            closest_point_on_edge = start_point + projection * edge_direction
            distance = np.linalg.norm(closest_point_on_edge - robot_pos)
            
            if distance < min_distance:
                min_distance = distance
                closest_point = closest_point_on_edge
        
        return min_distance, closest_point
    
    def find_boundary_following_waypoint(self):
        """
        Finds a waypoint for boundary following.
        Also checks for the point with minimum distance to goal during traversal.
        """
        if not self.current_edges:
            return None
            
        # In livox frame, robot is at (0,0)
        robot_pos = np.array([0.0, 0.0])
        closest_edge = None
        closest_point = None
        min_distance = float('inf')
        closest_edge_index = 0
        
        for i, edge in enumerate(self.current_edges):
            start_point, end_point = edge
            
            # Skip edges that are too far away (more than 1.5 meters from robot)
            start_distance = np.linalg.norm(start_point - robot_pos)
            end_distance = np.linalg.norm(end_point - robot_pos)
            if start_distance > 2 and end_distance > 2:
                continue
            
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            if edge_length < 0.01:
                continue
            edge_direction = edge_vector / edge_length
            to_robot = robot_pos - start_point
            projection = np.dot(to_robot, edge_direction)
            projection = max(0, min(edge_length, projection))
            point_on_edge = start_point + projection * edge_direction
            distance = np.linalg.norm(point_on_edge - robot_pos)
            if distance < min_distance:
                min_distance = distance
                closest_edge = edge
                closest_point = point_on_edge
                closest_edge_index = i
        
        if closest_edge is None:
            return None

        self.closest_edge_point = closest_point
        
        # Calculate distance to goal from this point
        current_dist_to_goal = self.dist_to_goal(closest_point)
        
        # Update leave point ONLY during the first circuit (when hit_point_found_again is False)
        if self.mode == "follow_boundary_to_hit_point" and not self.hit_point_found_again and current_dist_to_goal < self.min_dist_to_goal:
            self.min_dist_to_goal = current_dist_to_goal
            self.leave_point = self.transform_to_camera_init(closest_point)
            self.get_logger().info(f"New leave point found at distance {current_dist_to_goal:.2f} to goal")

        start_point, end_point = closest_edge
        edge_vector = end_point - start_point
        edge_direction = edge_vector / np.linalg.norm(edge_vector)
        to_robot = robot_pos - closest_point
        cross_z = edge_direction[0] * to_robot[1] - edge_direction[1] * to_robot[0]
        moving_forward = cross_z > 0

        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE
        current_point = closest_point

        if moving_forward:
            while increment_left > 0 and current_index < len(self.current_edges):
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(end - current_point)
                if increment_left <= remaining_distance:
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point + edge_direction * increment_left
                    break
                else:
                    increment_left -= remaining_distance
                    current_index += 1
                    if current_index >= len(self.current_edges):
                        current_index = len(self.current_edges) - 1
                        last_edge = self.current_edges[current_index]
                        last_start, last_end = last_edge
                        
                        # Calculate the direction of the last edge
                        last_direction = (last_end - last_start) / np.linalg.norm(last_end - last_start)
                        
                        # Add 60cm (0.6m) movement in the same direction beyond the last edge
                        extended_point = last_end + last_direction * 0.6
                        self.get_logger().info("Reached end of edge list, extending 60cm forward")
                        
                        # Set this as our current point
                        current_point = extended_point
                        break
        else:
            while increment_left > 0 and current_index >= 0:
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(current_point - start)
                if increment_left <= remaining_distance:
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point - edge_direction * increment_left
                    break
                else:
                    increment_left -= remaining_distance
                    current_index -= 1
                    if current_index < 0:
                        current_index = 0
                        first_edge = self.current_edges[current_index]
                        first_start, first_end = first_edge
                        
                        # Calculate the direction of the first edge
                        first_direction = (first_start - first_end) / np.linalg.norm(first_start - first_end)
                        
                        # Add 60cm (0.6m) movement in the same direction beyond the first edge
                        extended_point = first_start + first_direction * 0.6
                        self.get_logger().info("Reached start of edge list, extending 60cm backward")
                        
                        # Set this as our current point
                        current_point = extended_point
                        break

        self.incremented_point = current_point

        if current_index >= 0 and current_index < len(self.current_edges):
            current_edge = self.current_edges[current_index]
            start, end = current_edge
            edge_direction = (end - start) / np.linalg.norm(end - start)
            perpendicular = np.array([-edge_direction[1], edge_direction[0]])
            to_robot = robot_pos - current_point
            if np.dot(perpendicular, to_robot) < 0:
                perpendicular = -perpendicular
            waypoint = current_point + perpendicular * self.SAFETY_MARGIN
            return waypoint
        else:
            # If we're at an extended point beyond the edge list
            return current_point

    def check_robot_stopped(self):
        """
        Checks if the robot has stopped moving for a significant amount of time.
        If stuck, initiates a recovery maneuver before giving up.
        Returns True if robot is considered permanently stuck, False otherwise.
        """
        current_position = np.array([self.current_x, self.current_y])
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        # If we're in recovery maneuver, check if it's completed or timed out
        if self.in_recovery_maneuver:
            # Calculate distance moved since recovery started
            distance_moved = np.linalg.norm(current_position - self.last_position)
            time_in_recovery = current_time - self.recovery_start_time
            
            # If robot has moved enough during recovery, consider it successful
            if distance_moved > self.MOVEMENT_THRESHOLD * 2:
                self.get_logger().info(f"Recovery maneuver successful - robot is moving again")
                self.in_recovery_maneuver = False
                self.recovery_stage = 0
                self.recovery_waypoint = None
                self.last_position = current_position
                self.last_position_time = current_time
                self.stopped_time = None
                return False
                
            # If recovery has timed out, consider robot permanently stuck
            if time_in_recovery > self.RECOVERY_TIMEOUT:
                self.get_logger().info(f"Recovery maneuver failed - robot still stuck after {self.RECOVERY_TIMEOUT} seconds")
                self.in_recovery_maneuver = False
                return True
                
            # Still in recovery, wait for outcome
            return False
        
        # Initialize position tracking if not done yet
        if self.last_position is None:
            self.last_position = current_position
            self.last_position_time = current_time
            self.stopped_time = None
            return False
        
        # Calculate distance moved since last check
        distance_moved = np.linalg.norm(current_position - self.last_position)
        
        # If robot has moved enough, reset stoppage timer
        if distance_moved > self.MOVEMENT_THRESHOLD:
            self.last_position = current_position
            self.last_position_time = current_time
            self.stopped_time = None
            return False
        
        # If robot hasn't moved much, check how long it's been stopped
        if self.stopped_time is None:
            self.stopped_time = current_time
        
        time_stopped = current_time - self.stopped_time
        
        # If robot has been stopped for long enough, initiate recovery maneuver
        if time_stopped >= self.STOP_CHECK_TIME:
            if not self.in_recovery_maneuver:
                self.get_logger().info(f"Robot detected as stopped for {time_stopped:.1f} seconds - initiating recovery maneuver")
                self.in_recovery_maneuver = True
                self.recovery_stage = 1  # Start with forward movement
                self.recovery_start_time = current_time
                self.recovery_waypoint = self.calculate_recovery_waypoint()
                return False
            
        return False
    
    def calculate_recovery_waypoint(self):
        """
        Calculates a recovery waypoint based on the current recovery stage:
        Stage 1: Turn 60 degrees clockwise
        Stage 2: Turn 60 degrees counter-clockwise
        """
        if self.recovery_stage == 1:
            # Stage 1: Turn 60 degrees clockwise
            self.get_logger().info("Recovery maneuver: Turning 60 degrees clockwise")
            # Stay in same position but rotate 60 degrees clockwise
            waypoint = np.array([self.current_x, self.current_y])
            # Set theta to current orientation minus 60 degrees (clockwise)
            self.target_orientation = self.current_orientation - math.radians(60)
            return waypoint
        elif self.recovery_stage == 2:
            # Stage 2: Turn 60 degrees counter-clockwise
            self.get_logger().info("Recovery maneuver: Turning 60 degrees counter-clockwise")
            # Stay in same position but rotate 60 degrees counter-clockwise
            waypoint = np.array([self.current_x, self.current_y])
            # Set theta to current orientation plus 60 degrees (counter-clockwise)
            self.target_orientation = self.current_orientation + math.radians(60)
            return waypoint
        else:
            self.get_logger().warn(f"Invalid recovery stage: {self.recovery_stage}")
            return None

    def dist_between_points(self, point1, point2):
        """Calculate Euclidean distance between two points in world frame."""
        return np.linalg.norm(point1 - point2)

    def timer_callback(self):
        if not self.is_odom_received or self.goal_position is None:
            return
        
        # First check if goal is reached
        if self.is_goal_reached():
            if self.mode != "goal_reached":
                self.get_logger().info(f"Goal reached! Distance: {self.dist_to_goal():.2f}m. Waiting for new goal.")
                self.mode = "goal_reached"
                self.recovering_from_stuck = False  # Reset recovery flag
                self.hit_point_reset_count = 0     # Reset the counter when goal is reached
            # When goal is reached, don't publish new waypoints, just update visualizations
            self.publish_visualizations()
            return
            
        # Bug1 state machine
        if self.mode == "go_to_goal":
            # Reset stopped detection when entering this mode
            self.last_position = None
            self.stopped_time = None
            
            # Check if we're close to any obstacle edge
            min_edge_distance, closest_point = self.find_closest_edge_distance()
            
            # If close to an obstacle and hit point not set, record hit point
            if min_edge_distance is not None and min_edge_distance < self.OBSTACLE_DETECTION_THRESHOLD and self.hit_point is None:
                # We have detected an edge close by - this is the hit point
                self.hit_point = np.array([self.current_x, self.current_y])
                self.get_logger().info(f"Hit point set at ({self.hit_point[0]:.2f}, {self.hit_point[1]:.2f}), " +
                                      f"distance to edge: {min_edge_distance:.2f}m")
                
                # Initialize leave point search
                self.min_dist_to_goal = float('inf')
                self.leave_point = None
                self.hit_point_found_again = False
                self.completed_full_circuit = False
                
                # Record the start time for boundary following
                self.boundary_following_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                
                self.mode = "follow_boundary_to_hit_point"
                return  # Skip to next iteration to start boundary following
            
            # No nearby obstacle or hit point already set, check line of sight
            if self.check_line_of_sight():
                # Direct path to goal is clear - navigate directly to goal
                waypoint_msg = Pose2D()
                waypoint_camera_init = self.goal_position  # Goal is already in world frame
                waypoint_msg.x = float(waypoint_camera_init[0])
                waypoint_msg.y = float(waypoint_camera_init[1])
                waypoint_msg.theta = self.current_orientation
                self.waypoint_pub.publish(waypoint_msg)
                self.get_logger().debug("Mode: go_to_goal - Direct path clear, navigating to goal")
            else:
                # Line of sight is blocked but we're not close to an obstacle yet
                # Continue moving toward goal but slow down
                waypoint_msg = Pose2D()
                waypoint_camera_init = self.goal_position  # Goal is already in world frame
                waypoint_msg.x = float(waypoint_camera_init[0])
                waypoint_msg.y = float(waypoint_camera_init[1])
                waypoint_msg.theta = self.current_orientation
                self.waypoint_pub.publish(waypoint_msg)
                self.get_logger().debug("Mode: go_to_goal - Line of sight blocked, approaching obstacle")
                
        elif self.mode == "follow_boundary_to_hit_point":
            # Check if robot has stopped moving
            if self.check_robot_stopped():
                self.get_logger().info(f"Robot appears to be permanently stuck. Switching to go_to_goal mode.")
                self.mode = "go_to_goal"
                self.recovering_from_stuck = True  # Set recovery flag
                # Reset stoppage detection
                self.last_position = None
                self.stopped_time = None
                self.in_recovery_maneuver = False
                return
            
            # If in recovery maneuver, use recovery waypoint
            if self.in_recovery_maneuver and self.recovery_waypoint is not None:
                waypoint_msg = Pose2D()
                waypoint_msg.x = float(self.recovery_waypoint[0])
                waypoint_msg.y = float(self.recovery_waypoint[1])
                waypoint_msg.theta = float(self.target_orientation)  # Use the target orientation for turning
                self.waypoint_pub.publish(waypoint_msg)
                
                # Check if we need to move to next recovery stage
                current_time = self.get_clock().now().seconds_nanoseconds()[0]
                time_in_stage = current_time - self.recovery_start_time
                
                # Check if current rotation is complete by comparing orientations
                current_rotation_complete = abs(self.current_orientation - self.target_orientation) < 0.1  # ~5.7 degrees threshold
                
                if current_rotation_complete and time_in_stage > 2.0 and self.recovery_stage == 1:
                    self.recovery_stage = 2
                    self.recovery_waypoint = self.calculate_recovery_waypoint()
                    self.recovery_start_time = current_time
                
                # Check if robot has drifted too far from original hit point
                if self.hit_point is not None and not self.hit_point_found_again:
                    current_pos = np.array([self.current_x, self.current_y])
                    distance_from_hit_point = self.dist_between_points(current_pos, self.hit_point)
                    
                    # If more than 5 meters away and haven't reset too many times, reset hit point
                    if distance_from_hit_point > 5.0 and self.hit_point_reset_count < self.MAX_HIT_POINT_RESETS:
                        self.hit_point_reset_count += 1
                        self.get_logger().info(f"Robot has drifted {distance_from_hit_point:.2f}m from original hit point. " +
                                             f"Resetting hit point (reset {self.hit_point_reset_count} of {self.MAX_HIT_POINT_RESETS}).")
                        self.hit_point = current_pos
                        self.min_dist_to_goal = float('inf')
                        self.leave_point = None
                        self.boundary_following_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    elif distance_from_hit_point > 5.0:
                        self.get_logger().info(f"Robot has drifted {distance_from_hit_point:.2f}m from hit point, " +
                                             f"but maximum resets ({self.MAX_HIT_POINT_RESETS}) reached. Continuing with current hit point.")
                
                return  # Skip regular boundary following while in recovery
            
            # Step 3-4: Follow boundary and search for leave point (min distance to goal)
            waypoint = self.find_boundary_following_waypoint()
            
            if waypoint is not None:
                waypoint_msg = Pose2D()
                waypoint_camera_init = self.transform_to_camera_init(waypoint)
                waypoint_msg.x = float(waypoint_camera_init[0])
                waypoint_msg.y = float(waypoint_camera_init[1])
                waypoint_msg.theta = self.current_orientation
                self.waypoint_pub.publish(waypoint_msg)
                
                # Check if robot has drifted too far from original hit point
                if self.hit_point is not None and not self.hit_point_found_again:
                    current_pos = np.array([self.current_x, self.current_y])
                    distance_from_hit_point = self.dist_between_points(current_pos, self.hit_point)
                    
                    # If more than 5 meters away and haven't reset too many times, reset hit point
                    if distance_from_hit_point > 7.0 and self.hit_point_reset_count < self.MAX_HIT_POINT_RESETS:
                        self.hit_point_reset_count += 1
                        self.get_logger().info(f"Robot has drifted {distance_from_hit_point:.2f}m from original hit point. " +
                                             f"Resetting hit point (reset {self.hit_point_reset_count} of {self.MAX_HIT_POINT_RESETS}).")
                        self.hit_point = current_pos
                        self.min_dist_to_goal = float('inf')
                        self.leave_point = None
                        self.boundary_following_start_time = self.get_clock().now().seconds_nanoseconds()[0]
                    elif distance_from_hit_point > 5.0:
                        self.get_logger().info(f"Robot has drifted {distance_from_hit_point:.2f}m from hit point, " +
                                             f"but maximum resets ({self.MAX_HIT_POINT_RESETS}) reached. Continuing with current hit point.")
                
                # Step 5: Check if we've returned to the hit point
                if self.hit_point is not None and not self.hit_point_found_again:
                    # Check if enough time has passed
                    current_time = self.get_clock().now().seconds_nanoseconds()[0]
                    time_elapsed = current_time - self.boundary_following_start_time
                    
                    if time_elapsed >= self.MIN_BOUNDARY_FOLLOWING_TIME and self.is_near_hit_point():
                        # Store the current leave point and min_dist_to_goal
                        current_leave_point = None if self.leave_point is None else self.leave_point.copy()
                        current_min_dist = self.min_dist_to_goal
                        
                        self.hit_point_found_again = True
                        
                        # Restore the leave point and min_dist_to_goal
                        if current_leave_point is not None:
                            self.leave_point = current_leave_point
                            self.min_dist_to_goal = current_min_dist
                            
                            # Now that we've completed the circuit, finalize the leave point
                            self.get_logger().info(f"Returned to hit point after full boundary circuit ({time_elapsed:.1f}s). " + 
                                                  f"Leave point found at distance {self.min_dist_to_goal:.2f}m to goal. " + 
                                                  f"Continuing boundary following to reach leave point.")
                            self.mode = "follow_boundary_to_leave_point"
            
        elif self.mode == "follow_boundary_to_leave_point":
            # Check if robot has stopped moving
            if self.check_robot_stopped():
                self.get_logger().info(f"Robot appears to be permanently stuck. Switching to go_to_goal mode.")
                self.mode = "go_to_goal"
                self.recovering_from_stuck = True  # Set recovery flag
                # Reset stoppage detection
                self.last_position = None
                self.stopped_time = None
                self.in_recovery_maneuver = False
                return
                
            # Step 6: Continue boundary following to reach the leave point
            waypoint = self.find_boundary_following_waypoint()
            
            if waypoint is not None:
                waypoint_msg = Pose2D()
                waypoint_camera_init = self.transform_to_camera_init(waypoint)
                waypoint_msg.x = float(waypoint_camera_init[0])
                waypoint_msg.y = float(waypoint_camera_init[1])
                waypoint_msg.theta = self.current_orientation
                self.waypoint_pub.publish(waypoint_msg)
                
                # Check if we have reached the leave point
                if self.leave_point is not None:
                    if self.is_near_leave_point():
                        self.get_logger().info("Leave point reached, switching to go_to_goal")
                        self.mode = "go_to_goal"
                        # Don't reset hit_point and leave_point yet in case we encounter the same obstacle again
                        self.completed_full_circuit = True
                
        # Publish visualization markers
        self.publish_visualizations()
    
    def publish_visualizations(self):
        # Publish goal marker
        if self.goal_position is not None:
            goal_marker = Marker()
            goal_marker.header.frame_id = "camera_init"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.id = 0
            goal_marker.pose.position.x = float(self.goal_position[0])
            goal_marker.pose.position.y = float(self.goal_position[1])
            goal_marker.pose.position.z = 0.0
            goal_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
            
            # Change color based on whether goal is reached
            if self.mode == "goal_reached":
                # Green when reached
                goal_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            else:
                # Yellow when not yet reached
                goal_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
                
            self.goal_marker_pub.publish(goal_marker)
            
        # Publish edge markers
        edge_marker = Marker()
        edge_marker.header.frame_id = "livox"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.id = 0
        edge_marker.scale.x = 0.05
        edge_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        
        for start_point, end_point in self.current_edges:
            edge_marker.points.append(Point(x=float(start_point[0]), y=float(start_point[1]), z=0.0))
            edge_marker.points.append(Point(x=float(end_point[0]), y=float(end_point[1]), z=0.0))
            
        self.edge_marker_pub.publish(edge_marker)
        
        # Publish hit point marker (red)
        if self.hit_point is not None:
            hit_marker = Marker()
            hit_marker.header.frame_id = "camera_init"
            hit_marker.header.stamp = self.get_clock().now().to_msg()
            hit_marker.type = Marker.SPHERE
            hit_marker.action = Marker.ADD
            hit_marker.id = 0
            hit_marker.pose.position.x = float(self.hit_point[0])
            hit_marker.pose.position.y = float(self.hit_point[1])
            hit_marker.pose.position.z = 0.0
            hit_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
            hit_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            self.hit_point_marker_pub.publish(hit_marker)
        
        # Publish leave point marker (blue)
        if self.leave_point is not None:
            leave_marker = Marker()
            leave_marker.header.frame_id = "camera_init"
            leave_marker.header.stamp = self.get_clock().now().to_msg()
            leave_marker.type = Marker.SPHERE
            leave_marker.action = Marker.ADD
            leave_marker.id = 0
            leave_marker.pose.position.x = float(self.leave_point[0])
            leave_marker.pose.position.y = float(self.leave_point[1])
            leave_marker.pose.position.z = 0.0
            leave_marker.scale = Vector3(x=0.3, y=0.3, z=0.3)
            leave_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            self.leave_point_marker_pub.publish(leave_marker)
        
        # Publish current waypoint and intermediate points during boundary following
        if (self.mode == "follow_boundary_to_hit_point" or self.mode == "follow_boundary_to_leave_point") and \
           hasattr(self, 'closest_edge_point') and hasattr(self, 'incremented_point'):
            # Closest edge point marker (red)
            edge_point_marker = Marker()
            edge_point_marker.header.frame_id = "livox"
            edge_point_marker.header.stamp = self.get_clock().now().to_msg()
            edge_point_marker.type = Marker.SPHERE
            edge_point_marker.action = Marker.ADD
            edge_point_marker.id = 1
            edge_point_marker.pose.position.x = float(self.closest_edge_point[0])
            edge_point_marker.pose.position.y = float(self.closest_edge_point[1])
            edge_point_marker.pose.position.z = 0.0
            edge_point_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            edge_point_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            self.waypoint_marker_pub.publish(edge_point_marker)
            
            # Incremented point marker (green)
            inc_point_marker = Marker()
            inc_point_marker.header.frame_id = "livox"
            inc_point_marker.header.stamp = self.get_clock().now().to_msg()
            inc_point_marker.type = Marker.SPHERE
            inc_point_marker.action = Marker.ADD
            inc_point_marker.id = 2
            inc_point_marker.pose.position.x = float(self.incremented_point[0])
            inc_point_marker.pose.position.y = float(self.incremented_point[1])
            inc_point_marker.pose.position.z = 0.0
            inc_point_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
            inc_point_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            self.waypoint_marker_pub.publish(inc_point_marker)

def main(args=None):
    rclpy.init(args=args)
    node = Bug1Node()
    
    try:
        # No longer set a hardcoded goal position, will receive via topic
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()