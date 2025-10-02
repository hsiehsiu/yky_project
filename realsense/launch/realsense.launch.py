from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense',
            executable='realsense_node',
            name='my_rs_camera',
            output='screen',
            parameters=[{
                'rgb_profile': '640x480x30',
                'depth_profile': '640x480x30',
                'serial_no': '',
                'align_depth': True,
            }],
        )
    ])
