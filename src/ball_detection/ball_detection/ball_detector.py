import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust this to match your camera topic
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(Bool, 'ball_detected', 10)
        self.position_publisher = self.create_publisher(Point, 'ball_position', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert the ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert the image to grayscale (important for detecting the white and black ball)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Use Hough Circle Transform to detect the football (a circular object)
        circles = cv2.HoughCircles(
            gray, 
            cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
            param1=50, param2=30, minRadius=10, maxRadius=50
        )

        if circles is not None:
            # Convert the circle parameters to integers
            circles = np.uint16(np.around(circles))

            # Get the coordinates of the center of the first detected circle
            center = (circles[0, 0][0], circles[0, 0][1])  # (x, y)
            radius = circles[0, 0][2]

            # Draw a green circle around the detected ball
            cv2.circle(cv_image, center, radius, (0, 255, 0), 3)  # Green circle outline
            cv2.circle(cv_image, center, 3, (0, 0, 255), 3)  # Red center point

            # Publish the coordinates of the ball
            ball_position = Point()
            ball_position.x = float(center[0])  # Convert to float
            ball_position.y = float(center[1])  # Convert to float
            ball_position.z = float(radius)    # Convert to float

            self.publisher.publish(Bool(data=True))  # Ball detected
            self.position_publisher.publish(ball_position)
        else:
            # Publish that no ball is detected
            self.publisher.publish(Bool(data=False))  # No ball detected

        # Display the image with the ball detection circle
        cv2.imshow("Ball Detector", cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
