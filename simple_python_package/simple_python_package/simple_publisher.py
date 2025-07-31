import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        
        self.publisher = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello, ROS2! Count: {self.i}'
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)

    publisher_node = PublisherNode()

    rclpy.spin(publisher_node)

    publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
