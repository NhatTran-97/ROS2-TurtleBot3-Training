
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge
import numpy as np
import cv2


class ViewImage(Node):
    def __init__(self):
        super().__init__('image_processing_node')
        self.bridge = CvBridge()
        self.image_window = cv2.namedWindow("Camera Output")

        # Subscriptions - Camera Info & Image
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 1)

    # Image Processing - Callback
    def image_callback(self, img_msg):
        self.view_image(img_msg)
        

    def view_image(self, img_msg):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        cv2.imshow("image", cv_image)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = ViewImage()
    rclpy.spin(node)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
