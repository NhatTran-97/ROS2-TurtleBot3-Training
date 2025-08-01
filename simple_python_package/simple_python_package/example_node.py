import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy


class DemoNode(Node):
    def __init__(self):
        super().__init__('publisher_node')

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  history=QoSHistoryPolicy.KEEP_LAST, depth=10)

        self.reentrant_group = ReentrantCallbackGroup()
        self.exclusive_group = MutuallyExclusiveCallbackGroup()  #callback_group=self.reentrant_group
        self.timer_1 = self.create_timer(0.5, self.timer_callback_1 )
        self.timer_2 = self.create_timer(1.0, self.timer_callback_2)

        self.counter = 0

    def timer_callback_1(self):
        self.counter += 1
        self.get_logger().info(f'Timer_1: "{self.counter}"')


    def timer_callback_2(self):
        self.counter += 1
        self.get_logger().info(f'Timer_2: "{self.counter}"')

def main(args=None):
    rclpy.init(args=args)

    demo_node = DemoNode()
    executor = MultiThreadedExecutor()
    executor.add_node(demo_node)

    rclpy.spin(demo_node)

    demo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
