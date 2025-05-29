#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMover(Node):
    def __init__(self):
        super().__init__('circle_mover')
        # Create publisher to the command topic (adjust the topic name if necessary)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Declare and get parameters for linear and angular velocities
        self.declare_parameter('linear_velocity', 0.1) # m/s
        self.declare_parameter('angular_velocity', 2.0) # rad/s 
        self.linear_velocity = self.get_parameter('linear_velocity').value
        self.angular_velocity = self.get_parameter('angular_velocity').value
        
        # Calculate total time needed to complete 2 revolutions (4π radians)
        self.total_time = (4 * 3.14159265359) / self.angular_velocity
        
        self.get_logger().info(
            f"Circle motion started: linear_velocity={self.linear_velocity} m/s, "
            f"angular_velocity={self.angular_velocity} rad/s, "
            f"total duration={self.total_time:.2f} seconds."
        )
        
        # Record the start time and create a timer to send commands
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Calculate elapsed time in seconds
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        msg = Twist()
        if elapsed < self.total_time:
            # Continue sending the circle motion command
            msg.linear.x = self.linear_velocity
            msg.angular.z = self.angular_velocity
        else:
            # Time is up: send stop command and shutdown
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            self.get_logger().info("Completed two revolutions. Stopping robot.")
            rclpy.shutdown()
            return
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CircleMover()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


