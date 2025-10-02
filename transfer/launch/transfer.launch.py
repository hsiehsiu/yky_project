from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_target_topic', default_value='/yolo/target_best'),
        DeclareLaunchArgument('min_z', default_value='0.05'),
        DeclareLaunchArgument('max_z', default_value='2.0'),
        DeclareLaunchArgument('dry_run', default_value='true'),

        Node(
            package='transfer',
            executable='decision_node',
            name='decision_node',
            output='screen',
            parameters=[{
                'input_target_topic': LaunchConfiguration('input_target_topic'),
                'min_z': LaunchConfiguration('min_z'),
                'max_z': LaunchConfiguration('max_z'),
            }]
        ),
        Node(
            package='transfer',
            executable='transfer_executor_node',
            name='transfer_executor_node',
            output='screen',
            parameters=[{
                'dry_run': LaunchConfiguration('dry_run'),
            }]
        ),
    ])
