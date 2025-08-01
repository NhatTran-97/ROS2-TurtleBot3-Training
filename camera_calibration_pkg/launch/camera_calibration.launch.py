from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[{
                "video_device": "/dev/video0",
                "image_width": 640,
                "image_height": 480,
                "framerate": 100.0,
                "camera_frame_id": "camera_link",
                "pixel_format": "yuyv",
                "camera_info_url":"package://camera_calibration_pkg/config/ost.yaml",
            }],
            name='camera'
        ),
        Node(
            package='image_viewer',
            executable='view_image',
            remappings=[
                ('image', 'camera/image_raw'),
            ],
            output='screen',
            emulate_tty=True,
            respawn=False,
            name='image_view'
        )
    ])

