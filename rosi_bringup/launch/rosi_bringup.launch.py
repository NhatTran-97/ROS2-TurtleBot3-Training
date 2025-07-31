import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():


    joy_node = Node(
        package="simple_cpp_package",
        executable="joy_node",
        name="joystick",
        )


    return LaunchDescription([

    ])