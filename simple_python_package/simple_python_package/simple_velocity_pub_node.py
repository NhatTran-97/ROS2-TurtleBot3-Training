
import rclpy
from rclpy.node import Node
from rosi_msgs.msg import SimpleVelocity
from geometry_msgs.msg import Twist
class MyPublisher(Node):

    def __init__(self):
        super().__init__('simple_velocity_pub_node')

        self.simple_vel_pub = self.create_publisher(SimpleVelocity, "/simple_vel", 10)
        self.simple_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.listener_callback, 10)
        self.twist_msg = Twist()
        self.timer_ = self.create_timer(1.0, self.timer_callback)

        
    def listener_callback(self, msg):
        self.twist_msg = msg

    def timer_callback(self):
        msg = SimpleVelocity()
        msg.linear_velocity = self.twist_msg.linear.x
        msg.angular_velocity = self.twist_msg.angular.z

        self.simple_vel_pub.publish(msg)


        self.get_logger().info('Publishing linear velocity: "%s" m/s, angular velocity: "%s" rad/s' % (msg.linear_velocity, msg.angular_velocity))

       

def main(args=None):
    rclpy.init(args=args)

    my_publisher = MyPublisher()

    rclpy.spin(my_publisher)

    my_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()