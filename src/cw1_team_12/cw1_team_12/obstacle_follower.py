#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import json
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class ObstacleFollowerNode(Node):
    def __init__(self):
        super().__init__('obstacle_follower_node')
        
        # Constants
        self.SAFETY_MARGIN = 0.8  # meters
        self.INCREMENT_DISTANCE = 2.0 # meters
        self.UPDATE_RATE = 0.2  # seconds
        
        # Parameters for edge smoothing
        self.MIN_EDGE_LENGTH = 0.05  # Minimum edge length to consider
        self.CURVATURE_THRESHOLD = 0.3  # Threshold for keeping points with high curvature
        self.CHAIKIN_ITERATIONS = 2  # Number of iterations for Chaikin smoothing
        self.JOIN_ANGLE_THRESHOLD = math.radians(90)  # radians - threshold for joining collinear edges
        self.SMOOTHING_ANGLE_THRESHOLD = math.radians(90)  # Maximum allowed angle change
        
        # Initial target position (unused in current implementation)
        self.initial_x = 0.0
        self.initial_y = 1.0
        self.initial_theta = math.pi
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False
        self.current_edges = []        
        
        # Stuck detection parameters
        self.STUCK_DISTANCE_THRESHOLD = 0.10  # 10 cm
        self.STUCK_TIME_THRESHOLD = 2.0  # 2 seconds

        
        try:
            pkg_share = get_package_share_directory('cw1_team_12')
            self.get_logger().error(f'PACKAGE DIRECTORY: {pkg_share}')
            self.data_file = os.path.join(pkg_share, 'config', 'parameters.json')
            self.get_logger().error(f'PARAMETERS FILE PATH: {self.data_file}')
        except Exception as e:
            self.get_logger().error(f'CRITICAL ERROR FINDING PACKAGE: {str(e)}')
        
        # At the end of __init__
        self.get_logger().warn('NODE INITIALIZATION COMPLETE')
            
        # Create config directory if it doesn't exist
        os.makedirs(os.path.dirname(self.data_file), exist_ok=True)
        
        # Load parameters from JSON file with error handling
        try:
            with open(self.data_file, 'r') as f:
                self.json_data = json.load(f)
            
            # Get position data with defaults
            last_pos = self.json_data.get('last_position')
            if last_pos is not None:
                self.last_position = np.array(last_pos)
            else:
                self.last_position = None
                
            time_pos = self.json_data.get('position_check_time')
            if time_pos is not None:
                self.position_check_time = time_pos
            else:
                self.position_check_time = None

            
            stuck_counter = self.json_data.get('stuck_counter')
            if stuck_counter!=0:
                self.stuck_counter = stuck_counter
            else:
                self.stuck_counter = 0
                
        except (FileNotFoundError, json.JSONDecodeError) as e:
            self.get_logger().error(f"Error loading JSON: {e}")
            # Create default data
            self.json_data = {"last_position": None, "position_check_time": None}
            self.last_position = None
            self.position_check_time = None
            
            # Write the default file
            try:
                with open(self.data_file, 'w') as f:
                    json.dump(self.json_data, f)
                self.get_logger().info("Created default parameters file")
            except Exception as write_err:
                self.get_logger().error(f"Could not write parameters file: {write_err}")
    
        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, 'current_waypoint', 10)
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)
        
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
            
        # Timer
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)
        
        self.get_logger().info('Obstacle Follower node initialized')

    def odom_callback(self, msg):
        """Update current robot pose (x,y,theta) from odometry (in odom frame)"""
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def transform_to_camera_init(self, point):
        """Transform a point from livox to camera_init frame, which is our fixed world frame"""
        # Rotation matrix
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        
        # Transform point
        x = point[0] * c - point[1] * s + self.current_x
        y = point[0] * s + point[1] * c + self.current_y
        
        return np.array([x, y])

    def transform_to_base_link(self, point):
        """Transform a point from camera_init to livox frame"""
        # Translate to origin
        dx = point[0] - self.current_x
        dy = point[1] - self.current_y
        
        # Rotation matrix inverse
        c = math.cos(-self.current_orientation)
        s = math.sin(-self.current_orientation)
        
        # Transform point
        x = dx * c - dy * s
        y = dx * s + dy * c
        
        return np.array([x, y])
    
    def smooth_edges(self, points):
        """
        Improved edge smoothing using curvature-based filtering and Chaikin's algorithm.
        This approach preserves important corners while smoothing noise.
        """
        if len(points) < 3:
            return points
        # First filter points based on curvature
        filtered_points = self.curvature_filter(points)
        # Then apply Chaikin's corner cutting for smoother curves
        smoothed_points = self.chaikin_smooth(filtered_points, self.CHAIKIN_ITERATIONS)
        return smoothed_points

    def curvature_filter(self, points):
        """
        Filter points based on local curvature.
        Keeps points with high curvature (corners) and removes points with low curvature.
        """
        if len(points) < 3:
            return points
        
        # Always keep first point
        filtered = [points[0]]
        
        for i in range(1, len(points)-1):
            prev_vec = points[i] - points[i-1]
            next_vec = points[i+1] - points[i]
            
            # Skip very short edges
            prev_len = np.linalg.norm(prev_vec)
            next_len = np.linalg.norm(next_vec)
            if prev_len < self.MIN_EDGE_LENGTH or next_len < self.MIN_EDGE_LENGTH:
                continue
            
            # Normalize vectors
            prev_dir = prev_vec / prev_len
            next_dir = next_vec / next_len
            
            # Calculate angle between vectors (curvature)
            dot_product = np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0)
            angle = np.arccos(dot_product)
            
            # Keep point if curvature is high (angle is large)
            # if angle > self.CURVATURE_THRESHOLD:
            filtered.append(points[i])
        
        # Always keep last point
        filtered.append(points[-1])
        return filtered

    def chaikin_smooth(self, points, iterations=1):
        """
        Apply Chaikin's corner cutting algorithm to smooth a polyline.
        This creates a smoother curve while preserving the general shape.
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
                # Create two new points that are 1/4 and 3/4 along each segment
                q = p0 + 0.25 * (p1 - p0)
                r = p0 + 0.75 * (p1 - p0)
                new_points.append(q)
                new_points.append(r)
            
            new_points.append(result[-1])  # Keep last point
            result = np.array(new_points)
        return result

    def line_callback(self, msg):
        """Process line segments from the local map"""
        if len(msg.points) < 2:
            return
        points = [np.array([p.x, p.y]) for p in msg.points]
        
        # Use the smooth_edges function
        smoothed_points = self.smooth_edges(points)
        new_edges = []
        for i in range(len(smoothed_points) - 1):
            new_edges.append((smoothed_points[i], smoothed_points[i+1]))
        
        # Merge collinear edges
        merged_edges = []
        if new_edges:
            current_start, current_end = new_edges[0]
            for edge in new_edges[1:]:
                next_start, next_end = edge
                # Compute unit vectors for the current and next segments
                curr_vec = current_end - current_start
                next_vec = next_end - next_start
                curr_norm = np.linalg.norm(curr_vec)
                next_norm = np.linalg.norm(next_vec)
                if curr_norm < 1e-6 or next_norm < 1e-6:
                    continue
                curr_dir = curr_vec / curr_norm
                next_dir = next_vec / next_norm
                # Compute the angle between the two segments
                dot = np.clip(np.dot(curr_dir, next_dir), -1.0, 1.0)
                angle = np.arccos(dot)
                # If nearly collinear and endpoints touch, join them
                if angle < self.JOIN_ANGLE_THRESHOLD and np.linalg.norm(current_end - next_start) < 1e-6:
                    current_end = next_end  # extend current edge
                else:
                    merged_edges.append((current_start, current_end))
                    current_start, current_end = edge
            merged_edges.append((current_start, current_end))
        else:
            merged_edges = new_edges
        
        # Filter out edges further than 1.5 m from the robot
        filtered_edges = []
        robot_origin = np.array([0.0, 0.0])
        for edge in merged_edges:
            start, end = edge
            dist = self.point_to_segment_distance(robot_origin, start, end)
            if dist <= 1.3:
                filtered_edges.append(edge)
        self.current_edges = filtered_edges

    def point_to_segment_distance(self, pt, seg_a, seg_b):
        """Calculate the minimum distance from a point to a line segment"""
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

    def find_next_waypoint(self):
        """Find closest edge and calculate next waypoint"""
        if not self.current_edges:
            return None
            
        # Robot is at origin in livox frame
        robot_pos = np.array([0.0, 0.0])
        
        # World frame robot position to robot frame
        robot_position_robot = self.transform_to_base_link(np.array([self.current_x, self.current_y]))
        
        # Find the closest point on any edge segment
        closest_edge = None
        closest_point = None
        min_distance = float('inf')
        closest_edge_index = 0
        all_edges_too_far = True
        
        # Find the closest left-side edge
        for i, edge in enumerate(self.current_edges):
            start_point, end_point = edge
            
            # Calculate edge vector
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            
            if edge_length < 0.01:  # Skip very short edges
                continue
            
            # Check if edge is too far
            if np.linalg.norm(start_point - robot_position_robot) >= 1.5:
                self.get_logger().info(f'Edge {i} too far')
                continue

            all_edges_too_far = False  # Found at least one edge within range
            
            # Normalize edge vector
            edge_direction = edge_vector / edge_length
            
            # Vector from start point to robot
            to_robot = robot_pos - start_point
            
            # Project robot position onto edge line
            projection = np.dot(to_robot, edge_direction)
            projection = max(0, min(edge_length, projection))
            point_on_edge = start_point + projection * edge_direction
            
            # Calculate distance to edge
            distance = np.linalg.norm(point_on_edge - robot_pos)
            
            # For left-side following, we want the closest edge
            if distance < min_distance:
                min_distance = distance
                closest_edge = edge
                closest_point = point_on_edge
                closest_edge_index = i
        
        # Check if all edges were too far before proceeding
        if all_edges_too_far:
            self.get_logger().info('All edges too far, activating stuck behavior')
            return None
            
        if closest_edge is None:
            self.get_logger().info('No left-side edge found, activating stuck behavior')
            return None

        # Store the closest point (red dot)
        self.closest_edge_point = closest_point

        # Now move along the continuous edge
        start_point, end_point = closest_edge
        edge_vector = end_point - start_point
        edge_direction = edge_vector / np.linalg.norm(edge_vector)
        
        # For left-side following, we typically want to move backward along the edge
        # This creates a counterclockwise motion around obstacles
        moving_forward = False
        
        # Move along edges to find increment point
        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE
        current_point = closest_point
        
        # Determine the current_point based on edge following
        if moving_forward:
            while increment_left > 0 and current_index < len(self.current_edges):
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(end - current_point)
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point + edge_direction * increment_left
                    break
                else:
                    # Move to next edge
                    increment_left -= remaining_distance
                    current_index += 1
                    if current_index >= len(self.current_edges):
                        # Check if we're at the end of the edge
                        if current_index >= len(self.current_edges) - 1:  # Last edge
                            # Move forward in the direction of the last edge
                            last_edge = self.current_edges[-1]
                            start, end = last_edge
                            edge_vector = end - start
                            edge_direction = edge_vector / np.linalg.norm(edge_vector)
                            
                            # 25cm forward from the last point
                            current_point = end + edge_direction * 0.25
                            break
                        else:
                            # Move to next edge
                            current_index = len(self.current_edges) - 1
                            current_point = self.current_edges[current_index][1]
                            break
        else:
            while increment_left > 0 and current_index >= 0:
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                
                remaining_distance = np.linalg.norm(current_point - start)

                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point - edge_direction * increment_left
                    break
                else:
                    # Move to previous edge
                    increment_left -= remaining_distance
                    current_index -= 1
                    
                    if current_index < 0:
                        # If at start of first edge
                        current_index = 0
                        first_edge = self.current_edges[0]
                        start, end = first_edge
                        edge_vector = end - start
                        edge_direction = edge_vector / np.linalg.norm(edge_vector)
                        current_point = start - edge_direction * 0.50  # Move 50cm back
                        break
                    else:
                        current_point = self.current_edges[current_index][0]
                        break

        # Store the current point for visualization
        self.incremented_point = current_point
        
        # Calculate perpendicular vector
        perpendicular = np.array([-edge_direction[1], edge_direction[0]])
        
        # Check which side the robot is on using cross product
        to_robot = robot_pos - current_point
        cross_product = edge_direction[0] * to_robot[1] - edge_direction[1] * to_robot[0]
        
        # If cross product is negative, robot is on the right side, so flip perpendicular
        if cross_product < 0:
            perpendicular = -perpendicular
            
        # Calculate final waypoint
        waypoint = current_point + perpendicular * self.SAFETY_MARGIN
        
        return waypoint

    def publish_visualizations(self, current_waypoint):
        """Publish visualization markers"""
        # Current waypoint
        if current_waypoint is not None:
            point_marker = Marker()
            point_marker.header.frame_id = "livox"
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.type = Marker.SPHERE
            point_marker.action = Marker.ADD
            point_marker.id = 0
            
            point_marker.pose.position.x = float(current_waypoint[0])
            point_marker.pose.position.y = float(current_waypoint[1])
            point_marker.pose.position.z = 0.0
            
            point_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            point_marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=1.0)
            self.waypoint_marker_pub.publish(point_marker)
            
            # Closest point on edge (red)
            if hasattr(self, 'closest_edge_point'):
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
                
            # Incremented point on edge (green)
            if hasattr(self, 'incremented_point'):
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
                
        # Detected edges
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

    def stuck_forced(self):
        """Execute the stuck rotation behavior"""
        self.get_logger().info('Performing stuck rotation')
        forward_vector = np.array([
            math.cos(self.current_orientation),
            math.sin(self.current_orientation)
        ])

        # Move forward 50cm
        new_x = self.current_x + forward_vector[0] * 0.25
        new_y = self.current_y + forward_vector[1] * 0.25
        
        waypoint_msg = Pose2D()
        waypoint_msg.x = new_x
        waypoint_msg.y = new_y
        waypoint_msg.theta = self.current_orientation + math.radians(22)
        self.waypoint_pub.publish(waypoint_msg)

    def stuck_rotation(self):
        """Execute the stuck rotation behavior"""
        self.get_logger().info('Performing stuck rotation')
        forward_vector = np.array([
            math.cos(self.current_orientation),
            math.sin(self.current_orientation)
        ])
        
        # Move forward 50cm
        new_x = self.current_x + forward_vector[0] * 0.35
        new_y = self.current_y + forward_vector[1] * 0.35
        
        waypoint_msg = Pose2D()
        waypoint_msg.x = new_x
        waypoint_msg.y = new_y
        # waypoint_msg.theta = self.current_orientation + math.radians(30)
        waypoint_msg.theta = self.current_orientation + math.radians(30)
        self.waypoint_pub.publish(waypoint_msg)
        
    def timer_callback(self):
        """Main control loop"""
        if not self.is_odom_received:
            return
        
        # Find next waypoint and handle edge following
        next_waypoint = self.find_next_waypoint()
        self.get_logger().info(f'Next waypoint: {next_waypoint}')

        current_time = self.get_clock().now().nanoseconds / 1e9
        current_position = np.array([self.current_x, self.current_y])

        if next_waypoint is None:
            self.stuck_counter += 1
            self.get_logger().info(f'Stuck counter: {self.stuck_counter}')
            if self.stuck_counter >= 20:
                self.stuck_rotation()
                self.stuck_counter = 0
                return
        
        if next_waypoint is not None:
            # Initialize position tracking
            self.stuck_counter = 0
            self.get_logger().info(f'Last position: {self.last_position} {self.position_check_time}')
            if self.last_position is None:
                self.last_position = current_position
                self.position_check_time = current_time
            else:
                # Check if robot hasn't moved enough
                distance_moved = np.linalg.norm(current_position - self.last_position)
                time_diff = current_time - self.position_check_time
                self.get_logger().info(f'Last position was not None')

                if distance_moved < self.STUCK_DISTANCE_THRESHOLD:
                    self.get_logger().info(f'Robot seems stuck')
                    if time_diff >= self.STUCK_TIME_THRESHOLD:
                        self.get_logger().info(f"Robot Stuck, Time diff: {time_diff}, Distance moved: {distance_moved}")
                        self.stuck_rotation()
                        # Only reset position and time after stuck is confirmed
                        self.last_position = current_position
                        self.position_check_time = current_time
                        return
                    else:
                        self.get_logger().info(f"Robot is not stuck, Time diff: {time_diff}, Distance moved: {distance_moved}")
                else:
                    # Only reset tracking if robot has moved enough
                    self.last_position = current_position
                    self.position_check_time = current_time

            # Normal edge following
            waypoint_msg = Pose2D()
            waypoint_camera_init = self.transform_to_camera_init(next_waypoint)
            waypoint_msg.x = float(waypoint_camera_init[0])
            waypoint_msg.y = float(waypoint_camera_init[1])
            waypoint_msg.theta = self.current_orientation
            self.get_logger().info(f'Publishing waypoint: x={waypoint_msg.x}, y={waypoint_msg.y}, theta={waypoint_msg.theta}')
            self.waypoint_pub.publish(waypoint_msg)

            # Save position data to JSON
            if self.last_position is not None:
                self.json_data['last_position'] = self.last_position.tolist()
            else:
                self.json_data['last_position'] = None
                
            # Store time directly (it's already a float)
            self.json_data['position_check_time'] = self.position_check_time
            self.json_data['stuck_counter'] = self.stuck_counter
            with open(self.data_file, 'w') as f:
                json.dump(self.json_data, f)
        
            self.publish_visualizations(next_waypoint)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFollowerNode()
    
    # Get package directory and construct parameters.json path
    package_dir = get_package_share_directory('cw1_team_12')
    data_file = os.path.join(package_dir, 'config', 'parameters.json')
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        # Reset JSON file to initial state
        try:
            # Reset JSON data to initial state
            json_data = {"last_position": None, "position_check_time": None, "stuck_counter": 0}
            
            # Write reset data to file
            with open(data_file, 'w') as f:
                json.dump(json_data, f)
            node.get_logger().info('Parameters file reset successfully')
        except Exception as e:
            node.get_logger().error(f'Failed to reset parameters file: {e}')
            
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()