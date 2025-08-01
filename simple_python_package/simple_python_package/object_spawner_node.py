from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose
from rosi_msgs.srv import GetObjectLocation
import rclpy
from rclpy.node import Node

class ObjectSpawnerNode(Node):
    def __init__(self):
        super().__init__('object_spawner_node')
        self.publisher_ = self.create_publisher(Marker, '/pose', 10)
        

        # TODO: Publisher that publishes RViz Markers

        #------------------------------------------------
        #                    TODO:
        #  Create your  Publisher that publishes RViz Markers
        #------------------------------------------------

        #------------------------------------------------
        #                    TODO:
        #  Create your Service Client here! Remember to check
        #  what is the Client type you need.
        #------------------------------------------------
        self.service_client = self.create_client(GetObjectLocation, '/find_object')

    def send_request(self):
        #create a request
        self.req = GetObjectLocation.Request()
        #send request
        
        self.future = self.service_client.call_async(self.req)

        rclpy.spin_until_future_complete(self, self.future)

        self.response = self.future.result()
        self.publish_marker(self.response.object_pose)
    #------------------------------------------------
    #                    TODO:

    #  Populate the send_request function.This is where you should craft your 
    #  request and read the response to be passed to the publish_markers function
    #  In the publish_markers fuction do the following
    #------------------------------------------------

    def publish_marker(self, pose):		
        marker = Marker()
        marker.pose = pose

        marker.header.frame_id = "/map"
        marker.header.stamp = self.get_clock().now().to_msg()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        self.publisher_.publish(marker)


def main():
    rclpy.init()
    node = ObjectSpawnerNode()
    while rclpy.ok():
        node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()