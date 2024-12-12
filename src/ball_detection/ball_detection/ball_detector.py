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

        # Convert the image to HSV color space
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define the range for orange color in HSV
        lower_orange = np.array([7, 0, 0])  # Slightly darker orange
        upper_orange = np.array([180, 255, 255]) # Adjust if needed

        # Create a mask for the orange color
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        # Reduce noise with morphological operations
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        # Detect contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)

            if radius > 10:  # Minimum size of the ball to detect
                # Draw a circle around the detected ball
                cv2.circle(cv_image, (int(x), int(y)), int(radius), (0, 255, 0), 3)  # Green circle outline
                cv2.circle(cv_image, (int(x), int(y)), 3, (0, 0, 255), 3)  # Red center point

                # Publish the coordinates of the ball
                ball_position = Point()
                ball_position.x = float(x)
                ball_position.y = float(y)
                ball_position.z = float(radius)

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