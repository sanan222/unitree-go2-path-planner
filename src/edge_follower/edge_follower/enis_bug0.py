#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
import signal
import sys
import subprocess

class ObstacleFollowerNode(Node):
    def __init__(self):
        super().__init__('obstacle_follower_node')
        
        # Constants
        self.SAFETY_MARGIN = 0.8        # meters; if an edge is within (or equal to) this distance, switch to obstacle following
        self.INCREMENT_DISTANCE = 1.5   # meters
        self.UPDATE_RATE = 0.5          # seconds
        self.POSITION_TOLERANCE = 0.2   # tolerance to goal position
        self.ANGLE_TOLERANCE = 0.2      # tolerance to goal orientation
        
        # Edge smoothing parameters
        self.SMOOTHING_ANGLE_THRESHOLD = 0.3  # radians (~17°)
        self.MIN_EDGE_LENGTH = 0.1            # meters
        
        # Use the initial target as the final goal.
        self.initial_x = 0.0
        self.initial_y = 5.0
        self.initial_theta = math.pi  # desired final orientation
        
        # State machine
        self.GOING_TO_START = 0
        self.FOLLOWING_OBSTACLE = 1
        self.GOING_TO_GOAL = 2
        self.state = self.GOING_TO_START
        
        # Debug için state isimlerini tanımla
        self.state_names = {
            self.GOING_TO_START: "GOING_TO_START",
            self.FOLLOWING_OBSTACLE: "FOLLOWING_OBSTACLE",
            self.GOING_TO_GOAL: "GOING_TO_GOAL"
        }
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False
        self.current_edges = []
        self.last_no_edge_time = None  # used when no edge is detected
        
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
        """Update robot pose (x, y, theta) from odometry (odom frame)."""
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def transform_to_camera_init(self, point):
        """Transform a point from livox to camera_init (world) frame."""
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        x = point[0] * c - point[1] * s + self.current_x
        y = point[0] * s + point[1] * c + self.current_y
        return np.array([x, y])

    def transform_to_base_link(self, point):
        """Transform a point from camera_init to livox frame."""
        dx = point[0] - self.current_x
        dy = point[1] - self.current_y
        c = math.cos(-self.current_orientation)
        s = math.sin(-self.current_orientation)
        x = dx * c - dy * s
        y = dx * s + dy * c
        return np.array([x, y])
    
    def smooth_edges(self, points):
        """Smooth edges by removing points that are too close together."""
        if len(points) < 3:
            return points
        smoothed = [points[0]]
        for i in range(1, len(points)-1):
            prev_vector = points[i] - points[i-1]
            next_vector = points[i+1] - points[i]
            if np.linalg.norm(prev_vector) < self.MIN_EDGE_LENGTH or np.linalg.norm(next_vector) < self.MIN_EDGE_LENGTH:
                continue
            prev_dir = prev_vector / np.linalg.norm(prev_vector)
            next_dir = next_vector / np.linalg.norm(next_vector)
            angle = np.arccos(np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0))
            if angle > self.SMOOTHING_ANGLE_THRESHOLD:
                smoothed.append(points[i])
        smoothed.append(points[-1])
        return smoothed

    def line_callback(self, msg):
        """Process incoming line segments (obstacle edges)."""
        if len(msg.points) < 2:
            return
        points = [np.array([point.x, point.y]) for point in msg.points]
        smoothed_points = self.smooth_edges(points)
        self.current_edges = []
        for i in range(len(smoothed_points)-1):
            self.current_edges.append((smoothed_points[i], smoothed_points[i+1]))
        self.get_logger().info(f'Orijinal: {len(points)} nokta, Yumuşatılmış: {len(smoothed_points)} nokta')

    def find_next_waypoint(self):
        """Find the closest edge and calculate the next waypoint for obstacle following."""
        if not self.current_edges:
            return None
        # In the sensor (livox) frame, the robot is at the origin.
        robot_pos = np.array([0.0, 0.0])
        robot_position_robot = self.transform_to_base_link(np.array([self.current_x, self.current_y]))
        closest_edge = None
        closest_point = None
        min_distance = float('inf')
        closest_edge_index = 0
        for i, edge in enumerate(self.current_edges):
            start_point, end_point = edge
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            if edge_length < 0.01:
                continue
            if np.linalg.norm(start_point - robot_position_robot) > 1.3:
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

        # Save the closest point for visualization (red marker).
        self.closest_edge_point = closest_point

        # Move along the edge by INCREMENT_DISTANCE.
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
                        if current_index >= len(self.current_edges) - 1:
                            last_edge = self.current_edges[-1]
                            start, end = last_edge
                            edge_vector = end - start
                            edge_direction = edge_vector / np.linalg.norm(edge_vector)
                            forward_point = end + edge_direction * 0.25
                            perpendicular = np.array([-edge_direction[1], edge_direction[0]])
                            if np.dot(to_robot, perpendicular) < 0:
                                perpendicular = -perpendicular
                            waypoint = forward_point + perpendicular * self.SAFETY_MARGIN
                            waypoint_msg = Pose2D()
                            waypoint_world = self.transform_to_camera_init(waypoint)
                            waypoint_msg.x = float(waypoint_world[0])
                            waypoint_msg.y = float(waypoint_world[1])
                            waypoint_msg.theta = self.current_orientation - math.radians(22)
                            self.waypoint_pub.publish(waypoint_msg)
                            return waypoint
                        else:
                            current_index = len(self.current_edges) - 1
                            current_point = self.current_edges[current_index][1]
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
                        current_point = self.current_edges[current_index][0]
                        break

        self.incremented_point = current_point
        
        current_edge = self.current_edges[current_index]
        start, end = current_edge
        edge_direction = (end - start) / np.linalg.norm(end - start)
        perpendicular = np.array([-edge_direction[1], edge_direction[0]])
        to_robot = robot_pos - current_point
        if np.dot(perpendicular, to_robot) < 0:
            perpendicular = -perpendicular
        waypoint = current_point + perpendicular * self.SAFETY_MARGIN
        return waypoint

    def check_line_of_sight_to_goal(self):
        """Check if there is a clear path to the goal"""
        if not self.current_edges:
            return True
            
        # Robot ve hedef pozisyonları (world frame)
        p1 = np.array([self.current_x, self.current_y])
        p2 = np.array([self.initial_x, self.initial_y])
        
        # Debug
        self.get_logger().info(f'Checking line of sight from {p1} to {p2}')
        
        for edge in self.current_edges:
            start, end = edge
            # Edge'leri world frame'e dönüştür
            start_world = self.transform_to_camera_init(start)
            end_world = self.transform_to_camera_init(end)
            
            # Edge vektörü
            edge_vector = end_world - start_world
            to_goal = p2 - p1
            
            # Kesişim kontrolü
            denominator = edge_vector[0] * to_goal[1] - edge_vector[1] * to_goal[0]
            if abs(denominator) < 1e-6:  # Paralel
                continue
                
            t = ((start_world[0] - p1[0]) * to_goal[1] - 
                 (start_world[1] - p1[1]) * to_goal[0]) / denominator
            u = ((start_world[0] - p1[0]) * edge_vector[1] - 
                 (start_world[1] - p1[1]) * edge_vector[0]) / denominator
            
            if 0 <= t <= 1 and 0 <= u <= 1:
                self.get_logger().info(f'Path blocked by edge from {start_world} to {end_world}')
                return False
        
        self.get_logger().info('Clear path to goal')
        return True

    def goal_in_front_if_turn_left_90(self):
        """Check if turning 90 degrees left would point towards the goal"""
        # Hedefe olan vektör (world frame)
        to_goal = np.array([
            self.initial_x - self.current_x,
            self.initial_y - self.current_y
        ])
        
        # 90 derece sola dönmüş yön vektörü
        left_90_direction = np.array([
            -math.sin(self.current_orientation),
            math.cos(self.current_orientation)
        ])
        
        # İki vektör arasındaki açı
        dot_product = np.dot(to_goal, left_90_direction)
        angle = math.acos(dot_product / (np.linalg.norm(to_goal) * np.linalg.norm(left_90_direction)))
        
        # Debug
        self.get_logger().info(f'Angle to goal after 90 left turn: {math.degrees(angle)} degrees')
        
        return abs(angle) < math.radians(20)

    def edge_within_safety_margin(self):
        """
        Returns True if any detected edge is closer to the robot's current position
        than or equal to the defined safety margin.
        """
        P = np.array([self.current_x, self.current_y])
        for edge in self.current_edges:
            A, B = edge
            AB = B - A
            t = np.dot(P - A, AB) / (np.dot(AB, AB) + 1e-6)
            t = max(0.0, min(1.0, t))
            projection = A + t * AB
            distance = np.linalg.norm(P - projection)
            if distance <= self.SAFETY_MARGIN:  # <= now, so equal distance triggers obstacle mode
                return True
        return False

    def publish_visualizations(self, current_waypoint):
        """Publish visualization markers (blue for current waypoint, red for closest edge, green for incremented waypoint)."""
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
            # Red marker for closest edge point
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
            # Green marker for the incremented waypoint
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

    def timer_callback(self):
        if not self.is_odom_received:
            return
            
        # Her zaman edge detection'ı yap
        next_waypoint = self.find_next_waypoint()
        
        # Debug için current state'i yazdır
        self.get_logger().info(f'Current state: {self.state_names[self.state]}')
        
        if self.state == self.GOING_TO_START:
            # İlk hedef noktaya git
            waypoint_msg = Pose2D()
            waypoint_msg.x = self.initial_x
            waypoint_msg.y = self.initial_y
            waypoint_msg.theta = self.initial_theta
            self.waypoint_pub.publish(waypoint_msg)
            
            # Edge'e yakınlığı kontrol et
            if next_waypoint is not None:
                # Robot pozisyonundan edge'e olan mesafeyi hesapla
                robot_pos = np.array([self.current_x, self.current_y])
                edge_distance = np.linalg.norm(next_waypoint)

                
                # Edge safety margin mesafesinde mi?
                if edge_distance <= self.SAFETY_MARGIN:
                    self.state = self.FOLLOWING_OBSTACLE
                    self.get_logger().info('Edge detected within safety margin, switching to obstacle following')
                    return
            
            # Hedefe ulaştık mı kontrol et
            dx = self.current_x - self.initial_x
            dy = self.current_y - self.initial_y
            distance = math.sqrt(dx*dx + dy*dy)
            angle_diff = abs(self.current_orientation - self.initial_theta)
            
            if distance < self.POSITION_TOLERANCE and angle_diff < self.ANGLE_TOLERANCE:
                self.state = self.FOLLOWING_OBSTACLE
                self.get_logger().info('Reached initial position, switching to obstacle following')
        
        elif self.state == self.FOLLOWING_OBSTACLE:
            # Hedefe geçiş kontrolü
            can_go_to_goal = self.goal_in_front_if_turn_left_90() and self.check_line_of_sight_to_goal()
            self.get_logger().info(f'Can go to goal: {can_go_to_goal}')
            
            if can_go_to_goal:
                self.state = self.GOING_TO_GOAL
                self.get_logger().info('Switching to GOING_TO_GOAL state')
                return
                
            if next_waypoint is None:
                current_time = self.get_clock().now()
                
                if self.last_no_edge_time is None:
                    self.last_no_edge_time = current_time
                    return
                
                time_diff = (current_time - self.last_no_edge_time).nanoseconds / 1e9
                if time_diff >= 2.0:  # 2 saniye bekle
                    # Edge yoksa ileri git ve sağa dön
                    forward_vector = np.array([
                        math.cos(self.current_orientation),
                        math.sin(self.current_orientation)
                    ])
                    
                    # Mevcut konumdan 30cm ileri
                    new_x = self.current_x + forward_vector[0] * 0.3
                    new_y = self.current_y + forward_vector[1] * 0.3
                    
                    waypoint_msg = Pose2D()
                    waypoint_msg.x = new_x
                    waypoint_msg.y = new_y
                    waypoint_msg.theta = self.current_orientation - math.radians(15)  # 15 derece sola
                    self.waypoint_pub.publish(waypoint_msg)
                    self.get_logger().info('No edges found for 2 seconds, moving forward 30cm and turning left 15 degrees')
                    
                    self.last_no_edge_time = None
            else:
                # Edge bulundu, timer'ı sıfırla
                self.last_no_edge_time = None
                
                # Normal edge following
                waypoint_msg = Pose2D()
                waypoint_camera_init = self.transform_to_camera_init(next_waypoint)
                waypoint_msg.x = float(waypoint_camera_init[0])
                waypoint_msg.y = float(waypoint_camera_init[1])
                waypoint_msg.theta = self.current_orientation
                self.waypoint_pub.publish(waypoint_msg)
            
            self.publish_visualizations(next_waypoint)
        
        elif self.state == self.GOING_TO_GOAL:
            # Edge kontrolü
            if next_waypoint is not None:
                edge_distance = np.linalg.norm(self.transform_to_base_link(next_waypoint))
                if edge_distance <= self.SAFETY_MARGIN:
                    self.state = self.FOLLOWING_OBSTACLE
                    self.get_logger().info('Edge detected, switching back to FOLLOWING_OBSTACLE')
                    return
            
            # Hedefe ulaştık mı kontrol et
            dx = self.current_x - self.initial_x
            dy = self.current_y - self.initial_y
            distance_to_goal = math.sqrt(dx*dx + dy*dy)
            angle_diff = abs(self.current_orientation - self.initial_theta)

            if distance_to_goal < self.POSITION_TOLERANCE:
                if angle_diff > self.ANGLE_TOLERANCE:
                    # Pozisyonda dur ve orientation'ı düzelt
                    waypoint_msg = Pose2D()
                    waypoint_msg.x = self.current_x
                    waypoint_msg.y = self.current_y
                    waypoint_msg.theta = self.initial_theta
                    self.waypoint_pub.publish(waypoint_msg)
                    return
                else:
                    # Hem pozisyon hem orientation doğru, dur
                    stop_msg = Pose2D()
                    stop_msg.x = self.current_x
                    stop_msg.y = self.current_y
                    stop_msg.theta = self.initial_theta
                    self.waypoint_pub.publish(stop_msg)
                    
                    self.get_logger().info('Goal reached! Robot stopped.')
                    self.destroy_node()
                    rclpy.shutdown()
                    return
            
            # Hedefe doğru git
            waypoint_msg = Pose2D()
            waypoint_msg.x = self.initial_x
            waypoint_msg.y = self.initial_y
            waypoint_msg.theta = math.atan2(self.initial_y - self.current_y, 
                                          self.initial_x - self.current_x)
            self.waypoint_pub.publish(waypoint_msg)
    
    def shutdown(self):
        stop_msg = Pose2D()
        stop_msg.x = self.current_x
        stop_msg.y = self.current_y
        stop_msg.theta = self.current_orientation
        self.waypoint_pub.publish(stop_msg)
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFollowerNode()
    
    def signal_handler(sig, frame):
        node.get_logger().info('Shutdown signal received.')
        try:
            subprocess.run(['pkill', '-f', 'ros2'])
            subprocess.run(['pkill', '-f', 'gazebo'])
            subprocess.run(['pkill', '-f', 'gzclient'])
            subprocess.run(['pkill', '-f', 'gzserver'])
            node.shutdown()
        except Exception as e:
            node.get_logger().error(f'Error during shutdown: {e}')
        finally:
            sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)
    except Exception as e:
        node.get_logger().error(f'Error: {e}')
        signal_handler(signal.SIGINT, None)

if __name__ == '__main__':
    main()
