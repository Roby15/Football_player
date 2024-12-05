import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.subscription = self.create_subscription(
            Bool,
            'ball_detected',
            self.ball_detected_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.position_subscription = self.create_subscription(
            Point,
            'ball_position',
            self.ball_position_callback,
            10
        )
        
        self.ball_centered = False
        self.ball_position = Point()

    def ball_detected_callback(self, msg):
        self.ball_detected = msg.data
        if self.ball_detected:
            self.center_on_ball()
        else:
            self.stop_movement()

    def ball_position_callback(self, msg):
        # Update the ball position when it's detected
        self.ball_position = msg

    def center_on_ball(self):
        twist = Twist()

        # Check if the ball is detected and center the robot based on the ball's x position
        if self.ball_position.x != 0:  # If there's a valid ball position
            if self.ball_position.x < 300:  # Ball is to the left
                twist.angular.z = 0.5  # Rotate right
            elif self.ball_position.x > 340:  # Ball is to the right
                twist.angular.z = -0.5  # Rotate left
            else:
                twist.angular.z = 0.0  # Centered on the ball

            # Publish the twist to center the robot on the ball
            self.publisher.publish(twist)

            # Check if the robot is centered
            if 300 <= self.ball_position.x <= 340:  # Ball is centered
                self.ball_centered = True
                self.move_towards_ball()

    def move_towards_ball(self):
        if self.ball_centered:
            twist = Twist()
            twist.linear.x = 0.2  # Move forward towards the ball

            # Stop if the ball is close enough (based on the ball's y-coordinate)
            if self.ball_position.y < 100:  # Threshold for the y position (adjust as necessary)
                twist.linear.x = 0.0  # Stop if we're close to the ball
                self.stop_movement()

            self.publisher.publish(twist)

    def stop_movement(self):
        twist = Twist()  # Stop the robot by publishing zero velocities
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
