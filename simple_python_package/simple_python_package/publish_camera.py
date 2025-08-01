import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class UsbCameraNode(Node):
    def __init__(self):
        super().__init__('usb_camera_node')
        
        # Initialize CvBridge for converting OpenCV images to ROS messages
        self.bridge = CvBridge()
        
        # Create a publisher for the camera feed
        self.publisher_ = self.create_publisher(Image, '/camera/image_raw', 10)
        
        # Open the video capture (0 is the default camera, change if needed)
        self.cap = cv2.VideoCapture(2)
        
        if not self.cap.isOpened():
            self.get_logger().error("Failed to open the camera.")
            return
        
        # Set the camera resolution (optional)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        # Create a timer to publish camera frames
        self.timer = self.create_timer(0.1, self.timer_callback)  # Publish every 0.1 second
    
    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the frame to a ROS2 image message
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            
            # Publish the frame to the /camera/image_raw topic
            self.publisher_.publish(ros_image)
        else:
            self.get_logger().warn("Failed to capture image from the camera.")

    def destroy(self):
        # Release the camera when the node is destroyed
        if self.cap.isOpened():
            self.cap.release()

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    node = UsbCameraNode()

    try:
        # Keep the node running
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        node.destroy()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
