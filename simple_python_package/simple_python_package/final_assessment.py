import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseArray, Pose
from perception_msgs.srv import GetObjectLocation
import numpy as np
import time

class TurtleBotAruco(Node):
    def __init__(self):
        super().__init__('final_assessment')
        # Recommended values, change if you want!
        self.distance_threshold = 1.0
        self.max_linear_velocity = 0.1
        self.max_angular_velocity = 1.0
        self.too_close_distance = 0.15
        self.alignment_angle_threshold =  math.radians(20) 
        # self.angular_gain = 2.0   


        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.twist_msg = Twist()

        self.service_client = self.create_client(GetObjectLocation, '/find_objects')
        while not self.service_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')

        self.timer = self.create_timer(0.1, self.send_request) # thay đổi xuống còn 50ms để chạy nhanh hơn 


    def send_request(self):
        self.req = GetObjectLocation.Request()
        self.future_ = self.service_client.call_async(self.req)  
        self.future_.add_done_callback(self.response_callback)  

    def response_callback(self, future):
        try:
            result = future.result()
            # self.get_logger().info("Service response result: %d" % result.result)

            
            pose = result.object_pose
            self.get_logger().info(
                f"Object Pose - x: {pose.position.x:.2f}, y: {pose.position.y:.2f}, z: {pose.position.z:.2f}")

            # self.get_logger().info(
            #     f"Orierươcntation - x: {pose.orientation.x:.2f}, y: {pose.orientation.y:.2f}, z: {pose.orientation.z:.2f}, w: {pose.orientation.w:.2f}")

            self.check_tag(result, pose)
            

        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def check_tag(self, result, pose):

        if result.result is True:
            print("robot can run")

             # Level 1
            # self.move_turtlebot(self.max_linear_velocity, 0.0)

            # Level 2
            current_distance = self.get_tag_distance(pose.position.x, pose.position.y)
            print("current_distance: ", current_distance)

            angle_to_tag = self.get_tag_angle(pose.position.x, pose.position.y)
            # print("angle_to_tag: ", angle_to_tag)
            print("angle_to_tag (degree):", math.degrees(angle_to_tag))

            if abs(angle_to_tag) > self.alignment_angle_threshold:
                angular_vel = angle_to_tag
                if angular_vel > self.max_angular_velocity:
                    angular_vel = self.max_angular_velocity
                self.move_turtlebot(0.0, angular_vel)
                self.cmd_vel_pub.publish(self.twist_msg)
                return 

            if current_distance >= self.distance_threshold:
                print("✅ Reached target → stop")
                self.move_turtlebot(0.0, 0.0)

            elif current_distance <= self.too_close_distance:
                self.move_turtlebot(-self.max_linear_velocity, 0.0)
            
            # elif angle_to_tag
            
            else:
                print("➡️ Moving toward ArUco")
                self.move_turtlebot(self.max_linear_velocity, 0.0)
             
        else:
            print("robot can not run")
            
            self.move_turtlebot(0.0, 0.0)


        self.cmd_vel_pub.publish(self.twist_msg)


    
    def get_tag_pose(self):
        pass
        #------------------------------------------------
        #                    TODO:
        #  Create a function to get the pose of the tag 
        #------------------------------------------------

    def move_turtlebot(self, linear_velocity, angular_velocity):
        pass
        #------------------------------------------------
        #                    TODO:
        #  Create a function to move the robot
        #------------------------------------------------
        self.twist_msg.linear.x = linear_velocity
        self.twist_msg.angular.z = angular_velocity

    def get_tag_distance(self, x, y):
        return math.sqrt((x ** 2) + (y ** 2))

    def get_tag_angle(self, x, y):
        return np.arctan(y/x)
    

    # def get_tag_angle(self, x, y):
    #     return np.arctan2(y, x)


def main(args=None):
    rclpy.init(args=args)
    turtlebot_move = TurtleBotAruco()
    rclpy.spin(turtlebot_move)
    # while(rclpy.ok()):
    #     turtlebot_move.check_tag()
    turtlebot_move.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
