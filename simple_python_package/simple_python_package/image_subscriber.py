import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
# import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_publisher_node')

        self.bridge = CvBridge()

        self.reentrant_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10, callback_group=self.reentrant_group)

    def image_callback(self, msg):
        ret ,frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow("frame", frame)
        cv2.waitKey(1)



def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    executor = MultiThreadedExecutor()
    executor.add_node(image_subscriber)
    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        print("\nKeyboard interrupt detected. Exiting gracefully")
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
