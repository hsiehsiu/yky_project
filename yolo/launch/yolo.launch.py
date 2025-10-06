from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('yolo')
    # 預設模型路徑 → 指到你的 resource 目錄
    default_model_path = os.path.join(pkg_share, 'resource', 'yolo_model.pt')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model_path',
            default_value=default_model_path,
            description='Path to YOLO model (.pt)'
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.5',
            description='Confidence threshold'
        ),
        DeclareLaunchArgument(
            'estimate_3d',
            default_value='true',
            description='Estimate 3D position from depth'
        ),
        Node(
            package='yolo',
            executable='node',
            name='yolo',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'conf': LaunchConfiguration('conf'),
                'estimate_3d': LaunchConfiguration('estimate_3d'),
                'color_topic': '/camera/color/image_raw',
                'depth_topic': '/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/color/camera_info',
            }]
        ),
        Node(
            package='yolo',
            executable='yolo_visualization',
            name='yolo_visualization',
            output='screen',
            parameters=[
                
                {'image_topic': '/camera/color/image_raw'},
                {'detections_topic': '/yolo/detections'},
                {'out_image_topic': '/yolo/visualization/image'},
                {'sync_tolerance_ms': 120.0},
                {'draw_thickness': 2},
                {'font_scale': 0.5},
                {'min_score_to_draw': 0.0},
                {'box_color_bgr': [0, 255, 0]},
            ],
        ),
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='image_view'
        )
    ])
