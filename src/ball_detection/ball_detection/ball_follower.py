import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist

class BallFollower(Node):
    def __init__(self):
        super().__init__('ball_follower')
        self.subscription = self.create_subscription(
            Point, 
            'ball_position',  # Subscribe to ball position
            self.ball_position_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        
        self.is_centered = False

    def ball_position_callback(self, msg):
        # Get ball position (x, y)
        ball_x = msg.x
        ball_y = msg.y

        # Center the robot on the ball (if necessary)
        if not self.is_centered:
            self.center_on_ball(ball_x, ball_y)

        # Once centered, move towards the ball
        if self.is_centered:
            self.move_towards_ball(ball_x, ball_y)

    def center_on_ball(self, ball_x, ball_y):
        # Create a Twist message to control movement
        twist = Twist()

        # Center the robot based on ball_x and ball_y
        if ball_x < 300:  # Ball is to the left (you may need to adjust these values)
            twist.angular.z = 0.5  # Rotate right
        elif ball_x > 340:  # Ball is to the right
            twist.angular.z = -0.5  # Rotate left
        else:
            twist.angular.z = 0.0  # Stop rotating
            self.is_centered = True  # Once the ball is centered, stop rotating

        # Publish twist message to adjust robot's rotation
        self.publisher.publish(twist)

    def move_towards_ball(self, ball_x, ball_y):
        # Create a Twist message to control movement
        twist = Twist()
        
        # Move forward based on the ball's position
        twist.linear.x = 0.2  # Move towards the ball
        self.publisher.publish(twist)

        # Simulate shooting the ball after a brief moment
        self.shoot_ball()

    def shoot_ball(self):
        # Simulate the kick action by moving the robot forward quickly for a short time
        twist = Twist()
        twist.linear.x = 0.5  # Move quickly forward to simulate a kick
        self.publisher.publish(twist)

        # Stop after the "kick"
        self.stop_movement()

    def stop_movement(self):
        # Stop moving the robot if the ball is not detected
        twist = Twist()
        self.publisher.publish(twist)
        self.is_centered = False

def main(args=None):
    rclpy.init(args=args)
    node = BallFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
