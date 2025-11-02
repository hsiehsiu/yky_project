#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('keymap_file', default_value='/home/hsiu/tmrdriver_ws/resource/json/keycap_coordinate.json'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame'),
        DeclareLaunchArgument('units', default_value='mm'),  # 'mm' or 'm'

        Node(
            package='transfer',
            executable='transfer',
            name='transfer',          # 節點名稱=transfer
            output='screen',
            parameters=[{
                'rgb_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
                'detections_topic': '/yolo/detections',
                'base_frame': LaunchConfiguration('base_frame'),
                'camera_frame': LaunchConfiguration('camera_frame'),
                'keymap_file': LaunchConfiguration('keymap_file'),
                'units': LaunchConfiguration('units'),
                'lowercase_labels': True,
                'tf_timeout_sec': 0.08
            }]
        )
    ])
