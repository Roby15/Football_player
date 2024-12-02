#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CircleMotion(Node):
    def __init__(self):
        super().__init__('circle_motion')
        
        # Publisher for the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Timer to publish velocity commands periodically
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.get_logger().info("CircleMotion node has been started.")
    
    def publish_velocity(self):
        # Create a Twist message to send velocities
        twist = Twist()
        
        # Set linear velocity (forward motion)
        twist.linear.x = 0.2  # Adjust this value for speed
        
        # Set angular velocity (rotation)
        twist.angular.z = 0.2  # Adjust this value for the radius of the circle
        
        # Publish the message
        self.publisher_.publish(twist)
        self.get_logger().info("Publishing velocity: linear.x=%.2f, angular.z=%.2f" % (twist.linear.x, twist.angular.z))

def main(args=None):
    rclpy.init(args=args)
    node = CircleMotion()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
