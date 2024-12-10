import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class HSVRangeTuner(Node):
    def __init__(self):
        super().__init__('hsv_range_tuner')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Adjust this to your camera topic
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

        # Create a window for the HSV tuning trackbars
        cv2.namedWindow('HSV Adjustments')

        # Create trackbars for color range adjustment
        cv2.createTrackbar('Lower H', 'HSV Adjustments', 0, 180, self.nothing)
        cv2.createTrackbar('Lower S', 'HSV Adjustments', 0, 255, self.nothing)
        cv2.createTrackbar('Lower V', 'HSV Adjustments', 0, 255, self.nothing)
        cv2.createTrackbar('Upper H', 'HSV Adjustments', 180, 180, self.nothing)
        cv2.createTrackbar('Upper S', 'HSV Adjustments', 255, 255, self.nothing)
        cv2.createTrackbar('Upper V', 'HSV Adjustments', 255, 255, self.nothing)

    def nothing(self, x):
        pass

    def image_callback(self, msg):
        # Convert ROS image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Convert the image to HSV format
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Get the current values of the trackbars (HSV range)
        lh = cv2.getTrackbarPos('Lower H', 'HSV Adjustments')
        ls = cv2.getTrackbarPos('Lower S', 'HSV Adjustments')
        lv = cv2.getTrackbarPos('Lower V', 'HSV Adjustments')
        uh = cv2.getTrackbarPos('Upper H', 'HSV Adjustments')
        us = cv2.getTrackbarPos('Upper S', 'HSV Adjustments')
        uv = cv2.getTrackbarPos('Upper V', 'HSV Adjustments')

        # Create lower and upper bounds for the HSV filter
        lower_range = np.array([lh, ls, lv])
        upper_range = np.array([uh, us, uv])

        # Apply the mask based on the HSV range
        mask = cv2.inRange(hsv, lower_range, upper_range)

        # Bitwise-AND the original image and the mask to highlight the selected color
        result = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # Display the original image, mask, and filtered result
        cv2.imshow('Original Image', cv_image)
        cv2.imshow('Mask', mask)
        cv2.imshow('Filtered Image', result)

        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = HSVRangeTuner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
