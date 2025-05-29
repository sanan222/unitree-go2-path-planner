#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class Bug0Node(Node):
    """
    A ROS2 node implementing a Bug0-style local path planning algorithm for a legged robot.
    
    Key update: We add a "long term goal" check that, at every cycle, tests whether
    the direct path from the robot to the final goal is free of obstacles (i.e. no observable
    edges within the safety margin along the corridor between the robot and the goal). If
    the path is clear, the robot will generate a waypoint directly toward the goal,
    ignoring any edge data. If there is no clear path, the edge following and obstacle
    avoidance algorithm is applied.
    
    We also keep the logic that we only generate a new waypoint after the current 
    one is reached, to avoid excessive updates that cause the robot to turn in place.
    """

    def __init__(self):
        super().__init__('bug0')
        
        # ------------------------------------
        #         USER-TUNABLE PARAMETERS
        # ------------------------------------
        self.SAFETY_MARGIN = 0.7  # meters; minimum distance to keep from any obstacle.
        self.INCREMENT_DISTANCE = 0.35  # meters; how far we move along an obstacle each step.
        self.UPDATE_RATE = 0.1  # seconds; loop rate.
        self.GOAL_TOLERANCE = 0.2  # meters; threshold for considering final goal reached.
        self.WAYPOINT_REACHED_THRESH = 0.05  # meters; threshold for intermediate waypoint reached.
        self.FILTER_ALPHA = 0.7  # low-pass filter for smoothing edge positions.
        # ADDED: Filter coefficient for smoothing the computed waypoint.
        self.WAYPOINT_FILTER_ALPHA = 0.5  # 0.0 => very smooth (slow update), 1.0 => no filtering
        
        # ------------------------------------
        #          INTERNAL STATES
        # ------------------------------------
        self.is_odom_received = False
        self.current_edges = []  # will hold the filtered edges (list of (start_point, end_point) in 'livox' frame)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0

        # For visualization
        self.closest_edge_point = None
        self.incremented_point = None

        # Goal info (long term goal)
        self.goal_x = 0.0
        self.goal_y = 0.0
        self.goal_received = False
        self.goal_reached = False

        # Bug0 states: GO_TO_GOAL or AVOID_OBSTACLE
        self.state = "GO_TO_GOAL"

        # The active waypoint the robot is currently heading towards (x, y, theta) in world frame.
        self.active_waypoint = None

        # ADDED: To store the filtered version of the detected edges.
        self.filtered_edges = []  # list of (start_point, end_point)

        # ------------------------------------
        #           ROS PUBLISHERS
        # ------------------------------------
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, 'current_waypoint', 10)
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)

        # ------------------------------------
        #           ROS SUBSCRIPTIONS
        # ------------------------------------
        self.odom_sub = self.create_subscription(Odometry, 'Odometry', self.odom_callback, 10)
        self.line_sub = self.create_subscription(Marker, 'local_map_lines', self.line_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_callback, 10)

        # ------------------------------------
        #             TIMER
        # ------------------------------------
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)

        self.get_logger().info('Bug0 node initialized.')

    # --------------------------------------------------
    #               CALLBACK METHODS
    # --------------------------------------------------
    def odom_callback(self, msg):
        """Update current robot pose (x, y, theta) from odometry (in odom frame)."""
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def line_callback(self, msg):
        """
        Process incoming line segments (obstacles) and apply a low-pass filter to smooth edge positions.
        The filtered edges are stored in self.filtered_edges and used for further processing and visualization.
        """
        if len(msg.points) < 2:
            return

        # Convert raw points to numpy arrays.
        points = [np.array([p.x, p.y]) for p in msg.points]
        # Build new edge list from consecutive points.
        new_edges = []
        for i in range(len(points) - 1):
            new_edges.append((points[i], points[i+1]))

        # Update filtered_edges using an exponential moving average.
        if not self.filtered_edges or len(self.filtered_edges) != len(new_edges):
            self.filtered_edges = new_edges
        else:
            for i in range(len(new_edges)):
                old_start, old_end = self.filtered_edges[i]
                new_start, new_end = new_edges[i]
                filtered_start = self.FILTER_ALPHA * new_start + (1 - self.FILTER_ALPHA) * old_start
                filtered_end   = self.FILTER_ALPHA * new_end   + (1 - self.FILTER_ALPHA) * old_end
                self.filtered_edges[i] = (filtered_start, filtered_end)
        # Use filtered_edges for processing and visualization.
        self.current_edges = self.filtered_edges

    def goal_callback(self, msg):
        """Callback for receiving a new goal (Pose2D) in world coordinates."""
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.goal_received = True
        self.goal_reached = False
        self.state = "GO_TO_GOAL"
        self.active_waypoint = None  # Reset so a fresh waypoint is generated.
        self.get_logger().info(f"New goal received: x={self.goal_x:.2f}, y={self.goal_y:.2f}")

    # --------------------------------------------------
    #            FRAME TRANSFORM UTILITIES
    # --------------------------------------------------
    def transform_to_camera_init(self, point_livox):
        """Transform a 2D point from the robot's local 'livox' frame into the world frame 'camera_init'."""
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        x_world = point_livox[0] * c - point_livox[1] * s + self.current_x
        y_world = point_livox[0] * s + point_livox[1] * c + self.current_y
        return np.array([x_world, y_world])

    def transform_to_base_link(self, point_world):
        """Transform a 2D point from the world frame 'camera_init' into the robot's local 'livox' frame."""
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
        """
        Given an edge with endpoints p3 and p4 and the path segment p1->p2 (both in robot's frame),
        return True if at least part of the edge lies within the corridor between p1 and p2.
        """
        v = p2 - p1
        d = np.linalg.norm(v)
        if d < 1e-6:
            return False
        v_unit = v / d
        t3 = np.dot(p3, v_unit)
        t4 = np.dot(p4, v_unit)
        if (0 <= t3 <= d) or (0 <= t4 <= d) or (t3 < 0 and t4 > d) or (t4 < 0 and t3 > d):
            return True
        return False

    # --------------------------------------------------
    #    CHECK IF PATH TO LONG-TERM GOAL IS BLOCKED
    # --------------------------------------------------
    def is_path_to_goal_blocked(self):
        """
        Returns True if any edge lying within the corridor from the robot (p1) to the goal (p2)
        comes closer than self.SAFETY_MARGIN to that direct path.
        """
        if not self.goal_received or not self.current_edges:
            return False

        goal_livox = self.transform_to_base_link(np.array([self.goal_x, self.goal_y]))
        p1 = np.array([0.0, 0.0])
        p2 = goal_livox

        for (start_point, end_point) in self.current_edges:
            if not self.is_edge_in_path(start_point, end_point, p1, p2):
                continue
            dist = self.line_segment_to_line_segment_distance(p1, p2, start_point, end_point)
            if dist < self.SAFETY_MARGIN:
                return True
        return False

    def line_segment_to_line_segment_distance(self, p1, p2, p3, p4):
        """
        Returns the minimum distance between two line segments p1->p2 and p3->p4 in 2D.
        If they intersect, the distance is 0.
        """
        if self.segments_intersect(p1, p2, p3, p4):
            return 0.0
        d1 = self.point_to_segment_distance(p1, p3, p4)
        d2 = self.point_to_segment_distance(p2, p3, p4)
        d3 = self.point_to_segment_distance(p3, p1, p2)
        d4 = self.point_to_segment_distance(p4, p1, p2)
        return min(d1, d2, d3, d4)

    def point_to_segment_distance(self, pt, seg_a, seg_b):
        """
        Returns the minimum distance from point pt to the line segment seg_a->seg_b.
        """
        seg_v = seg_b - seg_a
        pt_v = pt - seg_a
        seg_len = np.linalg.norm(seg_v)
        if seg_len < 1e-6:
            return np.linalg.norm(pt - seg_a)
        seg_unit = seg_v / seg_len
        proj = np.dot(pt_v, seg_unit)
        proj = max(0, min(seg_len, proj))
        proj_point = seg_a + proj * seg_unit
        return np.linalg.norm(pt - proj_point)

    def segments_intersect(self, p1, p2, p3, p4):
        """
        Returns True if line segments p1->p2 and p3->p4 intersect.
        """
        def orientation(a, b, c):
            val = (b[1] - a[1]) * (c[0] - b[0]) - (b[0] - a[0]) * (c[1] - b[1])
            if abs(val) < 1e-9:
                return 0
            return 1 if val > 0 else 2

        def on_segment(a, b, c):
            return (min(a[0], c[0]) <= b[0] <= max(a[0], c[0]) and
                    min(a[1], c[1]) <= b[1] <= max(a[1], c[1]))

        o1 = orientation(p1, p2, p3)
        o2 = orientation(p1, p2, p4)
        o3 = orientation(p3, p4, p1)
        o4 = orientation(p3, p4, p2)

        if o1 != o2 and o3 != o4:
            return True
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
        """
        Generate a local waypoint in the robot's 'livox' frame that goes around
        the nearest obstacle in small increments anticlockwise. This method is
        used only when the direct path to the goal is blocked.
        """
        if not self.current_edges:
            return None
        
        robot_pos = np.array([0.0, 0.0])
        closest_edge = None
        closest_point = None
        closest_edge_index = 0
        min_distance = float('inf')

        for i, (start_point, end_point) in enumerate(self.current_edges):
            edge_vec = end_point - start_point
            edge_len = np.linalg.norm(edge_vec)
            if edge_len < 1e-6:
                continue
            edge_dir = edge_vec / edge_len
            to_robot = robot_pos - start_point
            proj = np.dot(to_robot, edge_dir)
            proj = max(0, min(edge_len, proj))
            point_on_edge = start_point + proj * edge_dir
            dist = np.linalg.norm(point_on_edge - robot_pos)
            if dist < min_distance:
                min_distance = dist
                closest_edge = (start_point, end_point)
                closest_point = point_on_edge
                closest_edge_index = i

        if closest_edge is None:
            return None

        self.closest_edge_point = closest_point

        start_pt, end_pt = closest_edge
        edge_vec = end_pt - start_pt
        edge_len = np.linalg.norm(edge_vec)
        if edge_len < 1e-6:
            return None

        current_point = closest_point
        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE

        while increment_left > 0 and current_index < len(self.current_edges):
            seg_start, seg_end = self.current_edges[current_index]
            seg_vec = seg_end - seg_start
            seg_len = np.linalg.norm(seg_vec)
            if seg_len < 1e-6:
                current_index += 1
                continue

            dir_unit = seg_vec / seg_len
            dist_to_end = np.linalg.norm(seg_end - current_point)

            if increment_left <= dist_to_end:
                current_point = current_point + dir_unit * increment_left
                increment_left = 0
            else:
                current_point = seg_end
                increment_left -= dist_to_end
                current_index += 1
                if current_index >= len(self.current_edges):
                    current_index = len(self.current_edges) - 1
                    break

        self.incremented_point = current_point

        seg_start, seg_end = self.current_edges[current_index]
        seg_vec = seg_end - seg_start
        seg_len = np.linalg.norm(seg_vec)
        if seg_len < 1e-6:
            return current_point

        edge_dir = seg_vec / seg_len
        perp = np.array([-edge_dir[1], edge_dir[0]])
        waypoint_livox = current_point + perp * self.SAFETY_MARGIN
        return waypoint_livox

    # --------------------------------------------------
    #        WAYPOINT HANDLING & STATE MACHINE
    # --------------------------------------------------
    def has_reached_active_waypoint(self):
        """Return True if the robot is close enough to the active waypoint."""
        if self.active_waypoint is None:
            return False
        ax, ay, _ = self.active_waypoint
        dx = ax - self.current_x
        dy = ay - self.current_y
        return math.hypot(dx, dy) < self.WAYPOINT_REACHED_THRESH

    def publish_current_waypoint(self):
        """Publish the active waypoint (Pose2D) to drive the robot."""
        if self.active_waypoint is None:
            return
        ax, ay, atheta = self.active_waypoint
        wp_msg = Pose2D(x=ax, y=ay, theta=atheta)
        self.waypoint_pub.publish(wp_msg)

    def timer_callback(self):
        """
        Main control loop (runs at UPDATE_RATE). The robot checks if the direct path to the
        long-term goal is free of obstacles. If so, it generates a waypoint toward the goal,
        ignoring the edge data. Otherwise, it uses the edge-following algorithm.
        """
        if not self.is_odom_received or not self.goal_received or self.goal_reached:
            return

        dx = self.goal_x - self.current_x
        dy = self.goal_y - self.current_y
        dist_to_goal = math.hypot(dx, dy)
        if dist_to_goal < self.GOAL_TOLERANCE:
            self.goal_reached = True
            self.get_logger().info("Goal reached.")
            final_pose = Pose2D(x=self.current_x, y=self.current_y, theta=self.current_orientation)
            self.waypoint_pub.publish(final_pose)
            return

        if self.active_waypoint is not None:
            if self.has_reached_active_waypoint():
                self.active_waypoint = None
            else:
                self.publish_current_waypoint()
                return

        # Check if the direct path is blocked by an edge (in the corridor).
        path_blocked = self.is_path_to_goal_blocked()
        if not path_blocked:
            self.state = "GO_TO_GOAL"
        else:
            self.state = "AVOID_OBSTACLE"

        if self.state == "GO_TO_GOAL":
            goal_livox = self.transform_to_base_link(np.array([self.goal_x, self.goal_y]))
            candidate_wp = goal_livox  # candidate waypoint in the robot's local frame.
        else:
            candidate_wp = self.follow_obstacle_anticlockwise()
            if candidate_wp is None:
                return

        # Convert candidate waypoint to world frame.
        waypoint_world = self.transform_to_camera_init(candidate_wp)
        dx_wp = waypoint_world[0] - self.current_x
        dy_wp = waypoint_world[1] - self.current_y
        theta_wp = math.atan2(dy_wp, dx_wp)
        candidate = (float(waypoint_world[0]), float(waypoint_world[1]), theta_wp)
        
        # ADDED: Smooth waypoint updates to avoid constant changes.
        if self.active_waypoint is None:
            # If no active waypoint exists, initialize it with the candidate.
            self.active_waypoint = candidate
        else:
            # Apply an exponential moving average to update the active waypoint.
            old_wp = self.active_waypoint
            new_x = self.WAYPOINT_FILTER_ALPHA * candidate[0] + (1 - self.WAYPOINT_FILTER_ALPHA) * old_wp[0]
            new_y = self.WAYPOINT_FILTER_ALPHA * candidate[1] + (1 - self.WAYPOINT_FILTER_ALPHA) * old_wp[1]
            new_theta = self.WAYPOINT_FILTER_ALPHA * candidate[2] + (1 - self.WAYPOINT_FILTER_ALPHA) * old_wp[2]
            self.active_waypoint = (new_x, new_y, new_theta)
        
        self.publish_current_waypoint()
        self.publish_visualizations(candidate_wp)

    # --------------------------------------------------
    #        VISUALIZATION MARKERS FOR RVIZ
    # --------------------------------------------------
    def publish_visualizations(self, current_waypoint_livox):
        """Publish RViz markers for edges and waypoints (in 'livox' frame)."""
        if current_waypoint_livox is not None:
            waypoint_marker = Marker()
            waypoint_marker.header.frame_id = "livox"
            waypoint_marker.header.stamp = self.get_clock().now().to_msg()
            waypoint_marker.type = Marker.SPHERE
            waypoint_marker.action = Marker.ADD
            waypoint_marker.id = 0
            waypoint_marker.pose.position.x = float(current_waypoint_livox[0])
            waypoint_marker.pose.position.y = float(current_waypoint_livox[1])
            waypoint_marker.pose.position.z = 0.0
            waypoint_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            waypoint_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            self.waypoint_marker_pub.publish(waypoint_marker)

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

        edge_marker = Marker()
        edge_marker.header.frame_id = "livox"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.id = 3
        edge_marker.scale.x = 0.05
        edge_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        for (start_point, end_point) in self.current_edges:
            edge_marker.points.append(Point(x=float(start_point[0]), y=float(start_point[1]), z=0.0))
            edge_marker.points.append(Point(x=float(end_point[0]), y=float(end_point[1]), z=0.0))

        self.edge_marker_pub.publish(edge_marker)

def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
