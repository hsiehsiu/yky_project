from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    # ---- 可參數化設定 ----
    fx = LaunchConfiguration('fx')
    fy = LaunchConfiguration('fy')
    cx = LaunchConfiguration('cx')
    cy = LaunchConfiguration('cy')
    json_path = LaunchConfiguration('json_path')
    base_frame = LaunchConfiguration('base_frame')
    camera_frame = LaunchConfiguration('camera_frame')
    depth_topic = LaunchConfiguration('depth_topic')
    yolo_topic = LaunchConfiguration('yolo_topic')

    # ---- 宣告 Launch Arguments ----
    declare_args = [
        DeclareLaunchArgument('json_path', default_value='/home/hudenxiao/tmdriver_ws/src/tmr_ros2/json/keycap_coordinate.json',
                              description='Path to save detection coordinates JSON'),
        DeclareLaunchArgument('base_frame', default_value='base_link', description='Base frame for TF lookup'),
        DeclareLaunchArgument('camera_frame', default_value='camera_color_optical_frame', description='Camera frame for TF lookup'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/aligned_depth_to_color/image_raw', description='Depth image topic'),
        DeclareLaunchArgument('yolo_topic', default_value='/yolo/detections', description='YOLO detection topic'),
    ]

    # ---- 節點設定 ----
    detection_node = Node(
        package='tmr_ros2',  # 如果你的 package 名稱不同，請修改這裡
        executable='detection_to_base',
        name='detection_to_base',
        output='screen',
        parameters=[{
            'fx': fx,
            'fy': fy,
            'cx': cx,
            'cy': cy,
            'json_path': json_path,
            'base_frame': base_frame,
            'camera_frame': camera_frame,
            'depth_topic': depth_topic,
            'yolo_topic': yolo_topic
        }]
    )

    return LaunchDescription(declare_args + [detection_node])
