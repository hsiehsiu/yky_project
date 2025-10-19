from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # --- Subscriber 節點們 ---
        Node(
            package='arm_movement',
            executable='pose_subscriber',
            name='pose_subscriber',
            output='log'
        )
    ])
