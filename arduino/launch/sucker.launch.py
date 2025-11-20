from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    這個 launch 檔會啟動 'arduino' package 中的 'sucker' 節點。
    """
    
    # 定義要啟動的節點
    sucker_node = Node(
        package='arduino',      # 您的 package 名稱
        executable='sucker',    # 您在 setup.py 中 entry_points 裡定義的執行檔名稱
        name='sucker',          # 節點的 ROS 2 名稱 (可以自訂)
        output='screen',        # 將節點的 log (日誌) 輸出到終端機畫面
        emulate_tty=True,       # 這有助於確保 output='screen' 能正常顯示 print() 內容
    )

    # 返回 LaunchDescription，其中包含所有要執行的動作
    return LaunchDescription([
        sucker_node
    ])