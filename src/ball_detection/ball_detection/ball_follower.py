import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point


class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.subscription = self.create_subscription(
            Point,
            'ball_position',  # Topic from ball_detector
            self.ball_position_callback,
            10
        )
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Ball follower node has started')

        # Parameters to control robot behavior
        self.stop_distance_threshold = 20  # Distance to stop (pixels)
        self.search_speed = 0.3  # Angular speed for searching
        self.approach_speed = 0.3  # Linear speed for approaching the ball
        self.rotate_speed = 0.5  # Angular speed to center the ball

        self.ball_detected = False  # Track if the ball is detected
        self.last_detection_time = self.get_clock().now()  # Last time the ball was seen
        self.lost_ball_duration = 2.0  # Time (seconds) to wait before searching

    def ball_position_callback(self, msg):
        """Callback when ball position is received."""
        ball_x = msg.x
        ball_y = msg.y

        # Create a Twist message for velocity commands
        twist = Twist()

        # If ball is not detected
        if ball_x <= 0 or ball_y <= 0:
            if self.ball_detected:
                # Update last detection time when ball was last seen
                self.last_detection_time = self.get_clock().now()
                self.ball_detected = False
            # If the ball has been missing for a while, start searching
            if (self.get_clock().now() - self.last_detection_time).seconds > self.lost_ball_duration:
                self.get_logger().info("Ball not detected, searching...")
                twist.linear.x = 0.0  # Stop forward motion
                twist.angular.z = self.search_speed  # Rotate in place to search
            else:
                # Continue to stop and wait briefly
                twist.linear.x = 0.0
                twist.angular.z = 0.0
        else:
            # Ball is detected
            self.ball_detected = True
            self.get_logger().info(f"Ball position: x={ball_x}, y={ball_y}")

            # Compute horizontal error
            error_x = ball_x - 320  # Center of the image (assuming 640x480 resolution)

            # If the ball is within the stop distance, stop the robot
            if ball_y > 480 - self.stop_distance_threshold:  # Ball is close enough
                self.get_logger().info('Ball is close. Stopping robot.')
                twist.linear.x = 0.0
                twist.angular.z = 0.0
            else:
                # Move toward the ball
                self.get_logger().info("Moving toward the ball.")
                twist.linear.x = self.approach_speed  # Move forward

                # Rotate to center the ball
                twist.angular.z = -self.rotate_speed * (error_x / 320)  # Scale rotation
                twist.angular.z = max(min(twist.angular.z, 1.0), -1.0)  # Clamp rotation speed

        # Publish the Twist message to control the robot
        self.cmd_vel_publisher.publish(twist)


def main():
    rclpy.init()
    ball_follower = BallFollower()
    rclpy.spin(ball_follower)
    ball_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
