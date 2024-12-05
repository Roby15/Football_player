import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class BallFinder(Node):
    def __init__(self):
        super().__init__('ball_finder')
        self.subscription = self.create_subscription(
            Bool,
            'ball_detected',
            self.ball_detected_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)

    def ball_detected_callback(self, msg):
        if not msg.data:  # Ball not detected
            self.search_for_ball()
        else:
            self.stop_search()

    def search_for_ball(self):
        # Send movement commands to search for the ball (e.g., rotating in place)
        twist = Twist()
        twist.angular.z = 0.5  # Rotate in place
        self.publisher.publish(twist)

    def stop_search(self):
        # Stop moving once the ball is detected
        twist = Twist()
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = BallFinder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
