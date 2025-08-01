
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray, Pose
from perception_msgs.srv import GetObjectLocation
import rclpy
from rclpy.node import Node

class ObjectSpawnerNode(Node):
    def __init__(self):
        super().__init__('object_spawner_node')

        self.marker_pub = self.create_publisher(Marker, '/marker',10) 
        self.service_client = self.create_client(GetObjectLocation, "find_objects")

    def send_request(self):
        self.req = GetObjectLocation.Request()
        self.future = self.service_client.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        self.response = self.future.result()
        self.response.object_pose
        self.response.result
        self.publish_marker(self.response.object_pose)
     

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
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = ObjectSpawnerNode()
    while rclpy.ok():
        node.send_request()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
