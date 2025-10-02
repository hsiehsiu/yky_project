from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='path_planning',
            executable='path_planner_node',
            name='path_planner_node',
            output='screen',
            parameters=[{
                'n_waypoints': 10,
                'base_frame': 'camera_color_optical_frame',
            }]
        ),
    ])
