import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import pyrealsense2 as rs

from geometry_msgs.msg import PoseArray, Pose
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA

class CvCameraNode(Node):
    def __init__(self):
        super().__init__('cv_camera_node')
        self.get_logger().info('Starting RealSense ArUco node...')
        
        # === RealSense setup ===
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)
        self.intr = self.profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
        self.get_logger().info(f"RealSense intrinsics: {self.intr}")
        
        # Build camera matrix and dist coeffs from RealSense intrinsics
        self.camera_matrix = np.array([
            [self.intr.fx, 0, self.intr.ppx],
            [0, self.intr.fy, self.intr.ppy],
            [0, 0, 1]
        ])
        self.dist_coeffs = np.array(self.intr.coeffs)  # should be shape (5,) or (5,1)
        if self.dist_coeffs.shape == (5,):
            self.dist_coeffs = self.dist_coeffs.reshape((5,1))
        self.marker_length = 0.04  # meters (set to your printed marker size)

        # ArUco
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_50)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        # ROS2 publishers
        self.pose_pub = self.create_publisher(PoseArray, 'aruco_poses', 10)
        self.text_pub = self.create_publisher(MarkerArray, 'aruco_ids', 10)

        self.timer = self.create_timer(0.03, self.timer_callback)  # ~30 FPS

    def detect_aruco_markers(self, gray_img):
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray_img, self.aruco_dict, parameters=self.aruco_params
        )
        return corners, ids, rejected

    def estimate_marker_poses(self, corners, ids):
        poses = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], self.marker_length, self.camera_matrix, self.dist_coeffs
                )
                poses.append((marker_id, rvec[0][0], tvec[0][0]))
        return poses

    def make_pose(self, rvec, tvec):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = tvec[0], tvec[1], tvec[2]
        # Convert rvec to quaternion
        R, _ = cv2.Rodrigues(rvec)
        import math
        qw = math.sqrt(1 + R[0,0] + R[1,1] + R[2,2]) / 2
        qx = (R[2,1] - R[1,2]) / (4*qw)
        qy = (R[0,2] - R[2,0]) / (4*qw)
        qz = (R[1,0] - R[0,1]) / (4*qw)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        return pose

    def make_text_marker(self, marker_id, tvec, marker_ns, marker_idx, stamp):
        marker = Marker()
        marker.header.frame_id = 'camera_color_optical_frame'
        marker.header.stamp = stamp
        marker.ns = marker_ns
        marker.id = marker_idx
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.z = 0.03  # meters (text height)
        marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)
        marker.text = str(marker_id)
        marker.pose.position.x = tvec[0]
        marker.pose.position.y = tvec[1]
        marker.pose.position.z = tvec[2] + 0.02
        marker.pose.orientation.w = 1.0
        marker.lifetime.sec = 1
        return marker

    def timer_callback(self):
        # === RealSense frame grab ===
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            self.get_logger().warn('No RealSense color frame')
            return
        color_image = np.asanyarray(color_frame.get_data())
        
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = self.detect_aruco_markers(gray)
        display_frame = color_image.copy()

        pose_array = PoseArray()
        pose_array.header = Header()
        pose_array.header.frame_id = 'camera_color_optical_frame'
        pose_array.header.stamp = self.get_clock().now().to_msg()
        marker_array = MarkerArray()

        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(display_frame, corners, ids)
            poses = self.estimate_marker_poses(corners, ids)
            for idx, (marker_id, rvec, tvec) in enumerate(poses):
                # Overlay text on camera image
                c = corners[idx][0]
                corner = tuple(map(int, c[0]))  # Top-left corner (x, y)
                cv2.putText(display_frame, f"ID:{marker_id}", (corner[0], corner[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 
                            0.7, (0,255,255), 2, cv2.LINE_AA)
                # Add pose to PoseArray
                pose_array.poses.append(self.make_pose(rvec, tvec))
                # Add text marker for RViz
                marker = self.make_text_marker(marker_id, tvec, "aruco_id", idx, pose_array.header.stamp)
                marker_array.markers.append(marker)
        else:
            cv2.putText(display_frame, "No marker detected", (30, 30), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.9, (255,255,255), 2)

        # Publish
        self.pose_pub.publish(pose_array)
        self.text_pub.publish(marker_array)
        cv2.imshow('Aruco Detection', display_frame)

        key = cv2.waitKey(1)
        if key & 0xFF == ord('q'):
            self.get_logger().info('Exiting...')
            self.destroy_node()
            rclpy.shutdown()

    def destroy_node(self):
        self.pipeline.stop()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CvCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        if rclpy.ok():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
