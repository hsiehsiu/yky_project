from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense',
            executable='realsense_node',
            name='my_rs_camera',
            output='screen',
            parameters=[{'enable_depth': True},           # 啟用深度串流 (通常預設為 True)
            {'enable_pointcloud': True},      # 啟用點雲 (PointCloud) 資料發布
            {'align_depth': True},            # 啟用深度到彩色圖像的對齊
            {'depth_module.profile': '640x480x30'}, # 設定深度串流的解析度與幀率
            {'color_module.profile': '640x480x30'}, # 設定彩色串流的解析度與幀率],
            ]
        )
    ])
