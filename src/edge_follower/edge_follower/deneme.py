#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class ObstacleFollowerNode(Node):
    def __init__(self):
        super().__init__('obstacle_follower_node')
        
        # Constants
        self.SAFETY_MARGIN = 0.8  # meters
        self.INCREMENT_DISTANCE = 1.5 # meters
        self.UPDATE_RATE = 0.5  # seconds
        self.POSITION_TOLERANCE = 0.2  # Hedef pozisyona yakınlık toleransı
        self.ANGLE_TOLERANCE = 0.2     # Hedef açı toleransı
        
        # Edge smoothing parameters
        self.SMOOTHING_ANGLE_THRESHOLD = 0.3  # radians (~17 degrees)
        self.MIN_EDGE_LENGTH = 0.1  # meters
        
        # Initial target position
        self.initial_x = 0.0
        self.initial_y = 1.0
        self.initial_theta = math.pi  # 3.14
        
        # State machine
        self.GOING_TO_START = 0
        self.FOLLOWING_OBSTACLE = 1
        self.state = self.GOING_TO_START
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False
        self.current_edges = []
        self.last_no_edge_time = None  # Edge bulunamadığı zamanı tutmak için
        
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
        """Smooth edges by removing points that are too close together"""
        if len(points) < 3:
            return points
            
        smoothed = [points[0]]  # İlk noktayı ekle
        
        for i in range(1, len(points)-1):
            # Önceki ve sonraki noktalar arasındaki vektörler
            prev_vector = points[i] - points[i-1]
            next_vector = points[i+1] - points[i]
            
            # Çok kısa edge'leri atla
            if np.linalg.norm(prev_vector) < self.MIN_EDGE_LENGTH or np.linalg.norm(next_vector) < self.MIN_EDGE_LENGTH:
                continue
            
            # Vektörlerin normalizasyonu
            prev_dir = prev_vector / np.linalg.norm(prev_vector)
            next_dir = next_vector / np.linalg.norm(next_vector)
            
            # Yön değişimi kontrolü
            angle = np.arccos(np.clip(np.dot(prev_dir, next_dir), -1.0, 1.0))
            
            # Eğer açı değişimi threshold'dan büyükse noktayı ekle
            if angle > self.SMOOTHING_ANGLE_THRESHOLD:
                smoothed.append(points[i])
                
        smoothed.append(points[-1])  # Son noktayı ekle
        return smoothed

    def line_callback(self, msg):
        """Process incoming line segments"""
        if len(msg.points) < 2:
            return

        # Convert line points to numpy arrays
        points = [np.array([point.x, point.y]) for point in msg.points]
        
        # Smooth the edges
        smoothed_points = self.smooth_edges(points)
        
        # Create edges by connecting adjacent points
        self.current_edges = []
        for i in range(len(smoothed_points) - 1):
            self.current_edges.append((smoothed_points[i], smoothed_points[i+1]))
        
        # Debug log
        self.get_logger().info(f'Original points: {len(points)}, Smoothed points: {len(smoothed_points)}')

    def find_next_waypoint(self):
        """Find closest edge and calculate next waypoint"""
        if not self.current_edges:
            return None  # Edge yoksa None döndür
            
        # Robot is at origin in livox frame
        robot_pos = np.array([0.0, 0.0])
        
        # World frame'deki robot pozisyonunu robot frame'e dönüştür
        robot_position_robot = self.transform_to_base_link(np.array([self.current_x, self.current_y]))
        
        # First find the closest point on any edge segment
        closest_edge = None
        closest_point = None
        min_distance = float('inf')
        closest_edge_index = 0
        
        for i, edge in enumerate(self.current_edges):
            start_point, end_point = edge
            
            # Calculate edge vector
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            
            if edge_length < 0.01:  # Skip very short edges
                continue
            
            # Şimdi ikisi de robot frame'inde, karşılaştır
            if np.linalg.norm(start_point - robot_position_robot) > 1.3:
                continue
            
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
            
            if distance < min_distance:
                min_distance = distance
                closest_edge = edge
                closest_point = point_on_edge
                closest_edge_index = i
        
        if closest_edge is None:
            return None

        # Store the closest point (red dot)
        self.closest_edge_point = closest_point

        # Now move clockwise along the continuous edge by INCREMENT_DISTANCE
        start_point, end_point = closest_edge
        edge_vector = end_point - start_point
        edge_direction = edge_vector / np.linalg.norm(edge_vector)
        
        # Vector from closest point to robot
        to_robot = robot_pos - closest_point
        
        # Determine cw direction using cross product
        cross_z = edge_direction[0] * to_robot[1] - edge_direction[1] * to_robot[0]
        moving_forward = cross_z > 0
        
        # Move along edges to find increment point
        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE
        current_point = closest_point
        
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
                        # Edge sonuna gelindi mi kontrol et
                        if current_index >= len(self.current_edges) - 1:  # Son edge'deyiz
                            # Son edge'in yönünde ileri git
                            last_edge = self.current_edges[-1]
                            start, end = last_edge
                            edge_vector = end - start
                            edge_direction = edge_vector / np.linalg.norm(edge_vector)
                            
                            # Son noktadan 25cm ileri
                            forward_point = end + edge_direction * 0.25  # 35cm -> 25cm
                            
                            # Güvenli mesafe ekle
                            perpendicular = np.array([-edge_direction[1], edge_direction[0]])
                            if np.dot(to_robot, perpendicular) < 0:
                                perpendicular = -perpendicular
                            
                            # Waypoint'i oluştur ve sağa dön
                            waypoint = forward_point + perpendicular * self.SAFETY_MARGIN
                            waypoint_msg = Pose2D()
                            waypoint_world = self.transform_to_camera_init(waypoint)
                            waypoint_msg.x = float(waypoint_world[0])
                            waypoint_msg.y = float(waypoint_world[1])
                            waypoint_msg.theta = self.current_orientation - math.radians(22)  # 20 -> 30 derece sağa
                            self.waypoint_pub.publish(waypoint_msg)
                            
                            return waypoint
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
                        # If we reach the start, stay at the first point
                        current_index = 0
                        current_point = self.current_edges[current_index][0]
                        break

        # Store the incremented point (green dot)
        self.incremented_point = current_point
        
        # Get the edge direction at the incremented point
        current_edge = self.current_edges[current_index]
        start, end = current_edge
        edge_direction = (end - start) / np.linalg.norm(end - start)
        
        # Calculate perpendicular vector (rotate edge_direction 90 degrees)
        perpendicular = np.array([-edge_direction[1], edge_direction[0]])
        
        # Check which side the robot is on
        to_robot = robot_pos - current_point
        if np.dot(perpendicular, to_robot) < 0:
            perpendicular = -perpendicular  # Flip if needed to point toward robot's side
            
        # Calculate waypoint by projecting perpendicular to the edge
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

    def timer_callback(self):
        """Main control loop"""
        if not self.is_odom_received:
            return
            
        if self.state == self.GOING_TO_START:
            # İlk hedef noktaya git
            waypoint_msg = Pose2D()
            waypoint_msg.x = self.initial_x
            waypoint_msg.y = self.initial_y
            waypoint_msg.theta = self.initial_theta
            self.waypoint_pub.publish(waypoint_msg)
            
            # Hedefe ulaştık mı kontrol et
            dx = self.current_x - self.initial_x
            dy = self.current_y - self.initial_y
            distance = math.sqrt(dx*dx + dy*dy)
            angle_diff = abs(self.current_orientation - self.initial_theta)
            
            if distance < self.POSITION_TOLERANCE and angle_diff < self.ANGLE_TOLERANCE:
                self.state = self.FOLLOWING_OBSTACLE
                self.get_logger().info('Reached initial position, switching to obstacle following')
        
        else:  # FOLLOWING_OBSTACLE state
            next_waypoint = self.find_next_waypoint()
            
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
                    
                    # Mevcut konumdan 25cm ileri
                    new_x = self.current_x + forward_vector[0] * 0.25  # 35cm -> 25cm
                    new_y = self.current_y + forward_vector[1] * 0.25
                    
                    waypoint_msg = Pose2D()
                    waypoint_msg.x = new_x
                    waypoint_msg.y = new_y
                    waypoint_msg.theta = self.current_orientation - math.radians(22)  # 20 -> 30 derece sağa
                    self.waypoint_pub.publish(waypoint_msg)
                    self.get_logger().info('No edges found for 2 seconds, moving forward 25cm and turning right 30 degrees')
                    
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

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleFollowerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()