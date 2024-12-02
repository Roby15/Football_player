import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust this topic to match your camera topic
            self.image_callback,
            10
        )
        self.ball_position_publisher = self.create_publisher(Point, 'ball_position', 10)
        self.get_logger().info('Ball detector node has started')

    def image_callback(self, msg):
        # Convert the ROS Image message to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Convert to grayscale
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Apply a blur to the image and then detect the circles (soccer ball)
        blurred = cv2.GaussianBlur(gray, (15, 15), 0)
        circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1, minDist=30, param1=50, param2=30, minRadius=10, maxRadius=100)

        if circles is not None:
            circles = np.round(circles[0, :]).astype("int")
            for (x, y, r) in circles:
                # Draw the circle in the output image
                cv2.circle(cv_image, (x, y), r, (0, 255, 0), 4)
                cv2.rectangle(cv_image, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                # Publish the ball position as floats
                ball_position = Point()
                ball_position.x = float(x)  # Convert to float
                ball_position.y = float(y)  # Convert to float
                self.ball_position_publisher.publish(ball_position)

        # Optionally display the result
        cv2.imshow("Ball Detection", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    ball_detector = BallDetector()
    rclpy.spin(ball_detector)
    ball_detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
