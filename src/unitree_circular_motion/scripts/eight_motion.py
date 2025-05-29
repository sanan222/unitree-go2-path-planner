#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class FigureEightMover(Node):
    def __init__(self):
        super().__init__('figure_eight_mover')
        # Create publisher to the command topic (adjust topic name if needed)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare parameters for linear speed, maximum angular speed, and total duration
        # The default values can be tuned to achieve a “slow” figure eight.
        self.declare_parameter('linear_velocity', 0.1) # m/s
        self.declare_parameter('max_angular_velocity', 0.3) # rad/s (peak turning rate)
        self.declare_parameter('total_duration', 40.0) # seconds to complete the figure 8
        
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.max_angular_velocity = self.get_parameter('max_angular_velocity').value
        self.total_duration = self.get_parameter('total_duration').value
        
        self.get_logger().info(
            f"Figure eight motion started: linear_velocity = {self.linear_velocity} m/s, "
            f"max_angular_velocity = {self.max_angular_velocity} rad/s, "
            f"total_duration = {self.total_duration:.1f} s."
        )
        
        # Record the start time and create a timer to publish Twist commands.
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Compute elapsed time in seconds.
        now = self.get_clock().now()
        elapsed = (now - self.start_time).nanoseconds / 1e9
        
        twist = Twist()
        if elapsed < self.total_duration:
            twist.linear.x = self.linear_velocity
            # Compute time-varying angular velocity:
            twist.angular.z = self.max_angular_velocity * math.sin(2 * math.pi * elapsed / self.total_duration)
        else:
            # Stop command: zero velocities.
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info("Figure eight completed. Stopping robot.")
            rclpy.shutdown()
            return

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = FigureEightMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
