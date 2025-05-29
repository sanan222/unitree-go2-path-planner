#!/usr/bin/env python3
# This script implements a Bug0-style local path planning algorithm for a legged robot using ROS2.

import math  # Provides mathematical functions such as trigonometry and square roots.
import numpy as np  # Used for efficient numerical operations and array handling.
import rclpy  # ROS2 Python client library.
from rclpy.node import Node  # Base class for creating ROS2 nodes.
from nav_msgs.msg import Odometry  # Message type for odometry data.
from geometry_msgs.msg import Pose2D, Point, Vector3  # Message types for 2D poses, points, and vectors.
from visualization_msgs.msg import Marker  # Message type for visual markers in RViz.
from std_msgs.msg import ColorRGBA  # Message type for representing colors with transparency.

class Bug0Node(Node):
    """
    A ROS2 node implementing a Bug0-style local path planning algorithm for a legged robot.

    Key update: Two separate waypoint variables are created. While in obstacle avoidance mode,
    only intermediate waypoints (generated from the edge-following function) are used by the robot,
    whereas the final goal candidate is computed and stored (but not used until after 3 seconds have elapsed
    and the path is clear). Debug messages print the mode switching and timing information.
    """

    def __init__(self):
        super().__init__('bug0')
        # Initialize the ROS2 node with the name 'bug0'
        
        # ------------------------------------
        #         USER-TUNABLE PARAMETERS
        # ------------------------------------
        self.SAFETY_MARGIN = 0.6          # Safety margin in meters to maintain from obstacles.
        self.INCREMENT_DISTANCE = 0.5     # Distance in meters to increment along an edge for waypoint generation.
        self.UPDATE_RATE = 0.1            # Timer update rate in seconds.
        self.GOAL_TOLERANCE = 0.2         # Tolerance in meters for considering the XY position as reached.
        self.WAYPOINT_REACHED_THRESH = 0.05  # Threshold in meters to determine if the waypoint is reached.
        self.WAYPOINT_FILTER_ALPHA = 0.7  # Smoothing factor for waypoint filtering (exponential moving average).
        
        # Parameters for edge smoothing
        self.MIN_EDGE_LENGTH = 0.1        # Minimum edge length to consider during curvature filtering.
        self.CURVATURE_THRESHOLD = 0.05    # Angle threshold (in radians) to keep points with significant curvature.
        self.CHAIKIN_ITERATIONS = 3       # Number of iterations for Chaikin smoothing algorithm.
        
        # New parameter: join nearly collinear edges.
        self.JOIN_ANGLE_THRESHOLD = 0.5   # Angle threshold (in radians) to join nearly collinear edges.
        
        # ------------------------------------
        #          INTERNAL STATES
        # ------------------------------------
        self.is_odom_received = False     # Flag to indicate if odometry data has been received.
        self.current_edges = []           # List to store detected edges after filtering/merging.
        self.current_x = 0.0              # Current X coordinate of the robot.
        self.current_y = 0.0              # Current Y coordinate of the robot.
        self.current_orientation = 0.0    # Current orientation (yaw) of the robot.
        
        # For visualization purposes
        self.closest_edge_point = None    # Closest point on an edge to the robot.
        self.incremented_point = None     # Point after incrementing along an edge.
        
        # Goal info
        self.goal_x = 0.0                 # X coordinate of the goal.
        self.goal_y = 0.0                 # Y coordinate of the goal.
        self.goal_theta = 0.0             # Desired orientation (theta) at the goal.
        self.goal_received = False        # Flag to indicate if a goal has been received.
        self.goal_reached = False         # Flag to indicate if the final goal has been reached.
        
        # State machine: "GO_TO_GOAL" or "AVOID_OBSTACLE"
        self.state = "GO_TO_GOAL"         # Initial state: attempt to go directly to the goal.
        self.active_waypoint = None       # Currently active waypoint for navigation.
        
        # New: Separate waypoint generation variables
        self.intermediate_wp = None  # Waypoint used during obstacle avoidance.
        self.final_goal_wp = None    # Final goal candidate waypoint (computed continuously but used later).
        
        # Hysteresis control variables for obstacle avoidance timing
        self.avoidance_start_time = None  # Timestamp when obstacle avoidance started.
        self.avoidance_elapsed_time = 0.0   # Elapsed time in obstacle avoidance mode.
        self.last_hysteresis_update = 0.0   # Timestamp for the last hysteresis update.
        
        # ------------------------------------
        #           ROS PUBLISHERS
        # ------------------------------------
        # Publisher for sending waypoints (Pose2D messages)
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        # Publisher for visualizing the current waypoint in RViz
        self.waypoint_marker_pub = self.create_publisher(Marker, 'current_waypoint', 10)
        # Publisher for visualizing detected edges
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)
        
        # ------------------------------------
        #           ROS SUBSCRIPTIONS
        # ------------------------------------
        # Subscribe to odometry data
        self.odom_sub = self.create_subscription(Odometry, 'Odometry', self.odom_callback, 10)
        # Subscribe to local map lines (edges) for obstacle detection
        self.line_sub = self.create_subscription(Marker, 'local_map_lines', self.line_callback, 10)
        # Subscribe to goal commands
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_callback, 10)
        
        # ------------------------------------
        #             TIMER
        # ------------------------------------
        # Create a timer that calls timer_callback at the defined update rate.
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)
        
        self.get_logger().info('Bug0 node initialized.')

    # --------------------------------------------------
    #               CALLBACK METHODS
    # --------------------------------------------------
    def odom_callback(self, msg):
        # Callback function to process incoming odometry data.
        self.is_odom_received = True  # Set flag indicating odometry data has been received.
        self.current_x = msg.pose.pose.position.x  # Update current X position.
        self.current_y = msg.pose.pose.position.y  # Update current Y position.
        q = msg.pose.pose.orientation  # Get the quaternion representing orientation.
        # Compute sine and cosine components for yaw extraction.
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)  # Compute yaw angle.

    def goal_callback(self, msg):
        # Callback function to process a new goal command.
        # Always update the goal command even if currently avoiding obstacles.
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_theta = msg.theta
        self.goal_received = True  # Indicate that a goal has been received.
        self.goal_reached = False  # Reset goal reached flag.
        # If not already in obstacle avoidance mode, switch state to GO_TO_GOAL.
        if self.state != "AVOID_OBSTACLE":
            self.state = "GO_TO_GOAL"
        self.active_waypoint = None  # Reset the active waypoint.
        self.get_logger().info(
            f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}, theta={self.goal_theta:.2f}"
        )

    def smooth_edges(self, points):
        # Function to smooth a set of edge points.
        if len(points) < 3:
            return points  # Not enough points to smooth.
        filtered_points = self.curvature_filter(points)  # Filter out points based on curvature.
        smoothed_points = self.chaikin_smooth(filtered_points, self.CHAIKIN_ITERATIONS)  # Apply Chaikin smoothing.
        return smoothed_points

    def curvature_filter(self, points):
        # Filters points based on curvature; removes points on nearly straight segments.
        if len(points) < 3:
            return points  # Not enough points to filter.
        filtered = [points[0]]  # Always include the first point.
        for i in range(1, len(points) - 1):
            prev_vec = points[i] - points[i - 1]  # Vector from previous point to current.
            next_vec = points[i + 1] - points[i]    # Vector from current point to next.
            prev_len = np.linalg.norm(prev_vec)
            next_len = np.linalg.norm(next_vec)
            # Skip if segment lengths are too short.
            if prev_len < self.MIN_EDGE_LENGTH or next_len < self.MIN_EDGE_LENGTH:
                continue
            prev_dir = prev_vec / prev_len  # Normalize previous vector.
            next_dir = next_vec / next_len    # Normalize next vector.
            dot_product = np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0)  # Dot product between normalized vectors.
            angle = np.arccos(dot_product)  # Compute angle between segments.
            # Keep the point if the curvature (angle) exceeds the threshold.
            if angle > self.CURVATURE_THRESHOLD:
                filtered.append(points[i])
        filtered.append(points[-1])  # Always include the last point.
        return filtered

    def chaikin_smooth(self, points, iterations=1):
        # Applies Chaikin's corner cutting algorithm to smooth the points.
        if len(points) < 3:
            return points  # Not enough points to smooth.
        result = np.array(points)
        for _ in range(iterations):
            new_points = [result[0]]  # Start with the first point.
            # Process each pair of points.
            for i in range(len(result) - 1):
                p0 = result[i]
                p1 = result[i + 1]
                q = p0 + 0.25 * (p1 - p0)  # Generate a new point 25% along the segment.
                r = p0 + 0.75 * (p1 - p0)  # Generate a new point 75% along the segment.
                new_points.extend([q, r])  # Add the two new points.
            new_points.append(result[-1])  # Append the last point.
            result = np.array(new_points)
        return result

    def line_callback(self, msg):
        # Callback function to process detected line segments from the sensor.
        if len(msg.points) < 2:
            return  # Not enough points to form a line.
        # Convert message points to numpy arrays for processing.
        points = [np.array([p.x, p.y]) for p in msg.points]
        # Smooth the detected edges.
        smoothed_points = self.smooth_edges(points)
        new_edges = []
        # Create edge segments from consecutive smoothed points.
        for i in range(len(smoothed_points) - 1):
            new_edges.append((smoothed_points[i], smoothed_points[i + 1]))
        
        # Merge collinear edges based on the JOIN_ANGLE_THRESHOLD.
        merged_edges = []
        if new_edges:
            current_start, current_end = new_edges[0]
            for edge in new_edges[1:]:
                next_start, next_end = edge
                curr_vec = current_end - current_start
                next_vec = next_end - next_start
                curr_norm = np.linalg.norm(curr_vec)
                next_norm = np.linalg.norm(next_vec)
                # Skip if any segment length is nearly zero.
                if curr_norm < 1e-6 or next_norm < 1e-6:
                    continue
                curr_dir = curr_vec / curr_norm  # Normalize current edge vector.
                next_dir = next_vec / next_norm      # Normalize next edge vector.
                dot = np.clip(np.dot(curr_dir, next_dir), -1.0, 1.0)  # Compute dot product.
                angle = np.arccos(dot)  # Determine the angle between edges.
                # If the edges are nearly collinear and connected, merge them.
                if angle < self.JOIN_ANGLE_THRESHOLD and np.linalg.norm(current_end - next_start) < 1e-6:
                    current_end = next_end  # Extend the current edge.
                else:
                    merged_edges.append((current_start, current_end))  # Save the current edge.
                    current_start, current_end = edge  # Start a new edge.
            merged_edges.append((current_start, current_end))  # Append the final edge.
        else:
            merged_edges = new_edges
        
        # Filter out edges that are too far from the robot.
        filtered_edges = []
        robot_origin = np.array([0.0, 0.0])
        for edge in merged_edges:
            start, end = edge
            dist = self.point_to_segment_distance(robot_origin, start, end)
            if dist <= 1.5:  # Consider only edges within 2.5 meters.
                filtered_edges.append(edge)
        self.current_edges = filtered_edges  # Update current edges for use in planning.

    # --------------------------------------------------
    #            FRAME TRANSFORM UTILITIES
    # --------------------------------------------------
    def transform_to_camera_init(self, point_livox):
        # Transform a point from the livox (sensor) frame to the world (camera initialization) frame.
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        x_world = point_livox[0] * c - point_livox[1] * s + self.current_x
        y_world = point_livox[0] * s + point_livox[1] * c + self.current_y
        return np.array([x_world, y_world])

    def transform_to_base_link(self, point_world):
        # Transform a point from the world frame to the robot's base_link frame.
        dx = point_world[0] - self.current_x
        dy = point_world[1] - self.current_y
        c = math.cos(-self.current_orientation)
        s = math.sin(-self.current_orientation)
        x_livox = dx * c - dy * s
        y_livox = dx * s + dy * c
        return np.array([x_livox, y_livox])

    # --------------------------------------------------
    #         HELPER: Check if an edge is in the path corridor
    # --------------------------------------------------
    def is_edge_in_path(self, p3, p4, p1, p2):
        # Determines if an edge (p3, p4) lies within the corridor defined by the segment (p1, p2).
        v = p2 - p1
        d = np.linalg.norm(v)
        if d < 1e-6:
            return False  # Avoid division by zero for very short segments.
        v_unit = v / d  # Unit vector along the corridor.
        t3 = np.dot(p3, v_unit)  # Projection of p3 onto the corridor.
        t4 = np.dot(p4, v_unit)  # Projection of p4 onto the corridor.
        # Check if either point lies within the corridor, or the edge spans the corridor.
        if (0 <= t3 <= d) or (0 <= t4 <= d) or (t3 < 0 and t4 > d) or (t4 < 0 and t3 > d):
            return True
        return False

    # --------------------------------------------------
    #    CHECK IF PATH TO LONG-TERM GOAL IS BLOCKED
    # --------------------------------------------------
    def is_path_to_goal_blocked(self):
        # Checks if any detected edge blocks the straight-line path to the goal.
        if not self.goal_received or not self.current_edges:
            return False  # If no goal or no obstacles, path is clear.
        # Transform goal coordinates to the robot's base_link frame.
        goal_livox = self.transform_to_base_link(np.array([self.goal_x, self.goal_y]))
        p1 = np.array([0.0, 0.0])  # Robot's position in base_link frame.
        p2 = goal_livox  # Goal position in base_link frame.
        # Check each edge for interference with the path.
        for (start_point, end_point) in self.current_edges:
            if not self.is_edge_in_path(start_point, end_point, p1, p2):
                continue  # Skip edges that are not in the path corridor.
            dist = self.line_segment_to_line_segment_distance(p1, p2, start_point, end_point)
            if dist < self.SAFETY_MARGIN:
                return True  # Path is blocked if an edge is within the safety margin.
        return False

    def line_segment_to_line_segment_distance(self, p1, p2, p3, p4):
        # Computes the minimum distance between two line segments.
        if self.segments_intersect(p1, p2, p3, p4):
            return 0.0  # If segments intersect, distance is zero.
        d1 = self.point_to_segment_distance(p1, p3, p4)
        d2 = self.point_to_segment_distance(p2, p3, p4)
        d3 = self.point_to_segment_distance(p3, p1, p2)
        d4 = self.point_to_segment_distance(p4, p1, p2)
        return min(d1, d2, d3, d4)

    def point_to_segment_distance(self, pt, seg_a, seg_b):
        # Calculates the shortest distance from a point 'pt' to a segment defined by seg_a and seg_b.
        seg_v = seg_b - seg_a
        pt_v = pt - seg_a
        seg_len = np.linalg.norm(seg_v)
        if seg_len < 1e-6:
            return np.linalg.norm(pt - seg_a)  # If segment is degenerate, return distance to seg_a.
        seg_unit = seg_v / seg_len  # Unit vector along the segment.
        proj = np.dot(pt_v, seg_unit)  # Projection length of pt_v on the segment.
        proj = max(0, min(seg_len, proj))  # Clamp projection to be within the segment.
        proj_point = seg_a + proj * seg_unit  # Closest point on the segment.
        return np.linalg.norm(pt - proj_point)

    def segments_intersect(self, p1, p2, p3, p4):
        # Checks if two segments (p1, p2) and (p3, p4) intersect.
        def orientation(a, b, c):
            # Helper function to determine orientation of triplet (a, b, c)
            val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
            if abs(val) < 1e-9:
                return 0  # Collinear
            return 1 if val > 0 else 2  # Clockwise or Counterclockwise

        def on_segment(a, b, c):
            # Checks if point b lies on segment (a, c)
            return (min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and
                    min(a[1], c[1]) <= b[1] <= max(a[1], c[1]))

        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)
        # General case: if orientations differ, segments intersect.
        if o1 != o2 and o3 != o4:
            return True
        # Check for special cases where points are collinear.
        if o1 == 0 and on_segment(p1, p3, p2):
            return True
        if o2 == 0 and on_segment(p1, p4, p2):
            return True
        if o3 == 0 and on_segment(p3, p1, p4):
            return True
        if o4 == 0 and on_segment(p3, p2, p4):
            return True
        return False

    # --------------------------------------------------
    #        OBSTACLE AVOIDANCE (ANTICLOCKWISE)
    # --------------------------------------------------
    def follow_obstacle_anticlockwise(self):
        # Determines a new waypoint by following the obstacle boundary in an anticlockwise direction.
        if not self.current_edges:
            return None  # No edges to follow.
        
        robot_pos = np.array([0.0, 0.0])  # Robot's position in its own base_link frame.
        closest_edge = None  # To store the closest edge segment.
        closest_point = None  # To store the closest point on that edge.
        closest_edge_index = 0  # Index of the closest edge in the list.
        min_distance = float('inf')  # Initialize with a large number.
        # Iterate over all detected edges to find the one closest to the robot.
        for i, (start_point, end_point) in enumerate(self.current_edges):
            edge_vec = end_point - start_point
            edge_len = np.linalg.norm(edge_vec)
            if edge_len < 1e-6:
                continue  # Skip degenerate edges.
            edge_dir = edge_vec / edge_len  # Compute the direction of the edge.
            to_robot = robot_pos - start_point  # Vector from the edge start to the robot.
            proj = np.dot(to_robot, edge_dir)  # Project robot position onto the edge.
            proj = max(0, min(edge_len, proj))  # Clamp the projection to the edge length.
            point_on_edge = start_point + proj * edge_dir  # Determine the closest point on the edge.
            dist = np.linalg.norm(point_on_edge - robot_pos)  # Compute distance to the robot.
            if dist < min_distance:
                min_distance = dist
                closest_edge = (start_point, end_point)
                closest_point = point_on_edge
                closest_edge_index = i

        if closest_edge is None:
            return None  # If no valid edge found, return None.

        self.closest_edge_point = closest_point  # Save for visualization.

        start_pt, end_pt = closest_edge
        edge_vec = end_pt - start_pt
        edge_len = np.linalg.norm(edge_vec)
        if edge_len < 1e-6:
            return None  # Avoid processing a degenerate edge.

        current_point = closest_point  # Start following from the closest point.
        current_index = closest_edge_index  # Starting edge index.
        increment_left = self.INCREMENT_DISTANCE  # Distance to move along the edge.
        # Follow the edge segments until the increment distance is exhausted.
        while increment_left > 0 and current_index < len(self.current_edges):
            seg_start, seg_end = self.current_edges[current_index]
            seg_vec = seg_end - seg_start
            seg_len = np.linalg.norm(seg_vec)
            if seg_len < 1e-6:
                current_index += 1
                continue  # Skip degenerate segment.
            dir_unit = seg_vec / seg_len  # Unit direction of the current segment.
            dist_to_end = np.linalg.norm(seg_end - current_point)
            if increment_left <= dist_to_end:
                current_point = current_point + dir_unit * increment_left  # Increment along the segment.
                increment_left = 0
            else:
                current_point = seg_end  # Move to the end of the segment.
                increment_left -= dist_to_end
                current_index += 1
                # If reached the last edge, compute a forward point for waypoint generation.
                if current_index >= len(self.current_edges):
                    current_index = len(self.current_edges) - 1
                    last_edge = self.current_edges[-1]
                    start_last, end_last = last_edge
                    edge_vector = end_last - start_last
                    if np.linalg.norm(edge_vector) < 1e-6:
                        break
                    edge_direction = edge_vector / np.linalg.norm(edge_vector)
                    forward_point = end_last + edge_direction * 0.5  # 50cm forward extension.
                    perp = np.array([-edge_direction[1], edge_direction[0]])  # Perpendicular vector.
                    if np.dot(robot_pos - end_last, perp) < 0:
                        perp = -perp  # Adjust the direction to ensure proper offset.
                    self.incremented_point = forward_point
                    waypoint_livox = forward_point + perp * self.SAFETY_MARGIN  # Final waypoint in sensor frame.
                    return waypoint_livox

        self.incremented_point = current_point  # Save the incremented point for visualization.
        
        seg_start, seg_end = self.current_edges[current_index]
        seg_vec = seg_end - seg_start
        seg_len = np.linalg.norm(seg_vec)
        if seg_len < 1e-6:
            return current_point  # Return current point if segment is degenerate.
        edge_dir = seg_vec / seg_len  # Direction of the current edge segment.
        perp = np.array([-edge_dir[1], edge_dir[0]])  # Compute a perpendicular direction.
        waypoint_livox = current_point + perp * self.SAFETY_MARGIN  # Offset the point by the safety margin.
        return waypoint_livox

    # --------------------------------------------------
    #        WAYPOINT HANDLING & STATE MACHINE
    # --------------------------------------------------
    def has_reached_active_waypoint(self):
        # Checks if the robot has reached the active waypoint.
        if self.active_waypoint is None:
            return False
        ax, ay, _ = self.active_waypoint
        dx = ax - self.current_x
        dy = ay - self.current_y
        return math.hypot(dx, dy) < self.WAYPOINT_REACHED_THRESH  # Compare distance to threshold.

    def publish_current_waypoint(self):
        # Publishes the active waypoint to the ROS topic.
        if self.active_waypoint is None:
            return
        ax, ay, _ = self.active_waypoint
        # Calculate orientation for the waypoint based on current robot position.
        theta = math.atan2(ay - self.current_y, ax - self.current_x)
        wp_msg = Pose2D(x=ax, y=ay, theta=theta)
        self.waypoint_pub.publish(wp_msg)

    def timer_callback(self):
        # Main periodic callback for processing planning logic.
        if not self.is_odom_received or not self.goal_received or self.goal_reached:
            return  # Do nothing if prerequisites are not met.
        
        # --- Final Goal Check ---
        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist_to_goal = math.hypot(dx, dy)
        # Check if the robot is close enough to the goal position.
        if dist_to_goal < self.GOAL_TOLERANCE:
            # Additionally, check if the orientation error is small.
            dtheta = abs(math.atan2(math.sin(self.goal_theta - self.current_orientation),
                                     math.cos(self.goal_theta - self.current_orientation)))
            if dtheta < 0.1:
                self.goal_reached = True  # Goal reached when both position and orientation are acceptable.
                self.get_logger().info("Final goal reached: position and orientation achieved.")
                final_pose = Pose2D(x=self.goal_x, y=self.goal_y, theta=self.goal_theta)
                self.waypoint_pub.publish(final_pose)
                return
            else:
                # If only position is reached, adjust orientation.
                self.get_logger().info("XY goal reached; adjusting orientation.")
                final_pose = Pose2D(x=self.goal_x, y=self.goal_y, theta=self.goal_theta)
                self.waypoint_pub.publish(final_pose)
                return
        # --- End Final Goal Check ---

        # If an active waypoint exists and hasn't been reached yet, keep publishing it.
        if self.active_waypoint is not None:
            if self.has_reached_active_waypoint():
                self.active_waypoint = None  # Clear active waypoint if reached.
            else:
                self.publish_current_waypoint()
                return

        # Compute the final goal candidate in the base_link frame (for path clearance checking).
        final_candidate_livox = self.transform_to_base_link(
            np.array([self.goal_x, self.goal_y])
        )
        self.final_goal_wp = final_candidate_livox

        current_time = self.get_clock().now().nanoseconds / 1e9  # Get current time in seconds.

        # State-based waypoint selection based on current mode.
        if self.state == "GO_TO_GOAL":
            # Check if direct path to goal is blocked.
            if self.is_path_to_goal_blocked():
                self.state = "AVOID_OBSTACLE"  # Switch to obstacle avoidance mode.
                self.avoidance_start_time = current_time  # Record the time of mode switch.
                self.get_logger().info(
                    f"DEBUG: Switching to AVOID_OBSTACLE mode at time {self.avoidance_start_time:.2f}"
                )
                candidate_wp = self.follow_obstacle_anticlockwise()  # Compute intermediate waypoint.
                self.intermediate_wp = candidate_wp
            else:
                candidate_wp = final_candidate_livox  # Directly head towards the goal.
        else:  # AVOID_OBSTACLE mode
            candidate_wp = self.follow_obstacle_anticlockwise()  # Continue obstacle avoidance.
            self.intermediate_wp = candidate_wp
            elapsed = current_time - (self.avoidance_start_time if self.avoidance_start_time else current_time)
            self.get_logger().info(
                f"DEBUG: AVOID_OBSTACLE mode: elapsed={elapsed:.2f} sec, Final candidate: {self.final_goal_wp}"
            )
            # After 3 seconds, if the path is clear, switch back to GO_TO_GOAL mode.
            if elapsed >= 3.0:
                if not self.is_path_to_goal_blocked():
                    self.get_logger().info("DEBUG: Switching to GO_TO_GOAL mode after 3 seconds; path is clear.")
                    self.state = "GO_TO_GOAL"
                    self.avoidance_start_time = None
                    candidate_wp = self.final_goal_wp
                else:
                    self.get_logger().info("DEBUG: Remaining in AVOID_OBSTACLE mode; path still blocked.")
        
        self.get_logger().info(f"DEBUG: Current state: {self.state}, Selected waypoint: {candidate_wp}")

        if candidate_wp is None:
            return  # If no candidate waypoint was computed, exit the callback.

        # Transform the candidate waypoint from the base_link frame to the world (camera init) frame.
        waypoint_world = self.transform_to_camera_init(candidate_wp)
        dx_wp = waypoint_world[0] - self.current_x
        dy_wp = waypoint_world[1] - self.current_y
        theta_wp = math.atan2(dy_wp, dx_wp)  # Compute the orientation towards the waypoint.
        candidate = (float(waypoint_world[0]), float(waypoint_world[1]), theta_wp)
        
        # Filter the waypoint using an exponential moving average if an active waypoint already exists.
        if self.active_waypoint is None:
            self.active_waypoint = candidate
        else:
            old_wp = self.active_waypoint
            new_x = self.WAYPOINT_FILTER_ALPHA * candidate[0] + (1 - self.WAYPOINT_FILTER_ALPHA) * old_wp[0]
            new_y = self.WAYPOINT_FILTER_ALPHA * candidate[1] + (1 - self.WAYPOINT_FILTER_ALPHA) * old_wp[1]
            new_theta = self.WAYPOINT_FILTER_ALPHA * candidate[2] + (1 - self.WAYPOINT_FILTER_ALPHA) * old_wp[2]
            self.active_waypoint = (new_x, new_y, new_theta)
        
        self.publish_current_waypoint()  # Publish the active waypoint.
        self.publish_visualizations(candidate_wp)  # Publish visual markers for debugging.

    def publish_visualizations(self, current_waypoint):
        # Publishes visualization markers for the current waypoint, closest edge point, and incremented point.
        if current_waypoint is not None:
            # Create a marker for the current waypoint.
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = "livox"
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.type = Marker.SPHERE
            waypoint_marker.action = Marker.ADD
            waypoint_marker.id = 0
            waypoint_marker.pose.position.x = float(current_waypoint[0])
            waypoint_marker.pose.position.y = float(current_waypoint[1])
            waypoint_marker.pose.position.z = 0.0
            waypoint_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            waypoint_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            self.waypoint_marker_pub.publish(waypoint_marker)

            # Create a marker for the closest point on an edge (if available).
            if self.closest_edge_point is not None:
                c_marker = Marker()
                c_marker.header.frame_id = "livox"
                c_marker.header.stamp = self.get_clock().now().to_msg()
                c_marker.type = Marker.SPHERE
                c_marker.action = Marker.ADD
                c_marker.id = 1
                c_marker.pose.position.x = float(self.closest_edge_point[0])
                c_marker.pose.position.y = float(self.closest_edge_point[1])
                c_marker.pose.position.z = 0.0
                c_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
                c_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                self.waypoint_marker_pub.publish(c_marker)

            # Create a marker for the incremented point (if available).
            if self.incremented_point is not None:
                i_marker = Marker()
                i_marker.header.frame_id = "livox"
                i_marker.header.stamp = self.get_clock().now().to_msg()
                i_marker.type = Marker.SPHERE
                i_marker.action = Marker.ADD
                i_marker.id = 2
                i_marker.pose.position.x = float(self.incremented_point[0])
                i_marker.pose.position.y = float(self.incremented_point[1])
                i_marker.pose.position.z = 0.0
                i_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
                i_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                self.waypoint_marker_pub.publish(i_marker)

        # Create a marker for all detected edges.
        edge_marker = Marker()
        edge_marker.header.frame_id = "livox"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.id = 3
        edge_marker.scale.x = 0.05
        edge_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        # Append each edge's start and end points to the marker.
        for start_point, end_point in self.current_edges:
            edge_marker.points.append(Point(x=float(start_point[0]), y=float(start_point[1]), z=0.0))
            edge_marker.points.append(Point(x=float(end_point[0]), y=float(end_point[1]), z=0.0))
        self.edge_marker_pub.publish(edge_marker)

def main(args=None):
    # Initialize the ROS2 Python client library.
    rclpy.init(args=args)
    # Create an instance of the Bug0Node.
    node = Bug0Node()
    try:
        # Spin the node to process callbacks.
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Clean up the node and shutdown ROS2.
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
