import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
import time
 
 
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0
        self.last_time = time.time()
 
    def compute(self, error):
        current_time = time.time()
        delta_time = current_time - self.last_time
        if delta_time == 0:
            return 0
 
        self.integral += error * delta_time
        derivative = (error - self.previous_error) / delta_time
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
 
        self.previous_error = error
        self.last_time = current_time
 
        return output
 
 
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
        self.ball_detected = False
        self.ball_position = Point()
 
        # PID controller for angular velocity
        self.pid = PIDController(kp=0.005, ki=0.0, kd=0.002)
 
        # Parameters
        self.image_center_x = 320  # Assume a 640px width image, adjust as necessary
        self.center_tolerance = 30  # Pixels tolerance for considering the ball centered
 
    def ball_detected_callback(self, msg):
        self.ball_detected = msg.data
        if not self.ball_detected:
            self.stop_movement()
 
    def ball_position_callback(self, msg):
        # Update the ball position when it's detected
        self.ball_position = msg
        if self.ball_detected:
            self.center_on_ball()
 
    def center_on_ball(self):
        twist = Twist()
    
        # Compute error (difference between ball's x position and image center)
        error = self.ball_position.x - self.image_center_x
    
        # Deadband to avoid small oscillations near the center
        if abs(error) <= self.center_tolerance:
            twist.angular.z = 0.0  # Stop rotating
            self.ball_centered = True
            self.move_towards_ball()
        else:
            angular_z = self.pid.compute(error)
    
            # Clamp the angular velocity
            angular_z = max(min(angular_z, 0.5), -0.5)  # Limit to ±0.5 rad/s
            twist.angular.z = -angular_z
            twist.linear.x = 0.0  # Don't move forward while centering
    
            self.publisher.publish(twist)
    def move_towards_ball(self):
        if self.ball_centered:
            twist = Twist()
            twist.linear.x = 0.2  # Move forward towards the ball
 
            # Stop if the ball is close enough (based on the ball's y-coordinate)
            #if self.ball_position.y < 100:  # Threshold for the y position (adjust as necessary)
               # twist.linear.x = 0.0  # Stop if we're close to the ball
               # self.stop_movement()
 
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