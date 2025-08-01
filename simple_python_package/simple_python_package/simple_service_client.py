import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys 



class SimpleServiceClient(Node):
    def __init__(self):
        super().__init__("simple_service_client")
        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        
     

        while not self.client_.wait_for_service(timeout_sec=1.0):  # wait for service function, 

            self.get_logger().info("Service not available, waiting again...")
        
        self.req_ = AddTwoInts.Request() # message type
        self.req_.a = 3
        self.req_.b = 4

        self.future_ = self.client_.call_async(self.req_) 

        self.future_.add_done_callback(self.responseCallback)
   
    def responseCallback(self, future):
        self.get_logger().info("Service Response %d" % future.result().sum)

def main():
    rclpy.init()
    # use sys lib to access to the parameter of the main. 

    # if the script has been started correctly, so with the correct number of arguments 
    simple_service_client = SimpleServiceClient() # argv[1] only name of the script
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()