#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator

def main():
    rclpy.init()

    # Tạo Node ROS2 riêng để publish với QoS
    node = Node("tb3_nav2_commander")

    qos_profile = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE
    )
    # Dùng BasicNavigator để điều khiển Nav2
    nav = BasicNavigator()

    # --- Set Initial Pose ---
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, 0.0)
    init_pose =  PoseStamped()    # PoseWithCovarianceStamped()
    init_pose.header.frame_id = 'map'
    init_pose.header.stamp = node.get_clock().now().to_msg()
    init_pose.pose.pose.position.x = 0.0
    init_pose.pose.pose.position.y = 0.0
    init_pose.pose.pose.orientation.x = qx
    init_pose.pose.pose.orientation.y = qy
    init_pose.pose.pose.orientation.z = qz
    init_pose.pose.pose.orientation.w = qw
    #nav.setInitialPose(init_pose)

    # Publish initial pose

    node.get_logger().info("Initial pose published")

    # Chờ Nav2 khởi động
    nav.waitUntilNav2Active()

    # --- Gửi goal_pose Pose ---
    # PI = 3.14 == 180
    # PI/2 == 1.57 == 90

    goal_pose = PoseStamped()
    qx, qy, qz, qw = tf_transformations.quaternion_from_euler(0.0, 0.0, 1.57)

    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = node.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.5
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.orientation.x = qx 
    goal_pose.pose.orientation.y = qy 
    goal_pose.pose.orientation.z = qz 
    goal_pose.pose.orientation.w = qw

    nav.goToPose(goal_pose)
    node.get_logger().info("goal_pose pose published")

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        rclpy.spin_once(node, timeout_sec=1)
        print(feedback)
    
    print(nav.getResult())

    node.get_logger().info("Navigation finished!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
