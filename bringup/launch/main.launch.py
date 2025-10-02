from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 可覆寫參數
    declare_robot_ip = DeclareLaunchArgument('robot_ip', default_value='192.168.10.2')
    declare_domain   = DeclareLaunchArgument('ros_domain_id', default_value='0')
    set_domain       = SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id'))

    # 到「各自套件」的 share 目錄去找它們的 launch 檔
    #tm_pkg        = get_package_share_directory('tm_driver')
    rs_pkg        = get_package_share_directory('realsense') 
    yolo_pkg      = get_package_share_directory('yolo')
    transfer_pkg  = get_package_share_directory('transfer')
    path_pkg      = get_package_share_directory('path_planning')
    '''
    tm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([tm_pkg, 'launch', 'tm_driver.launch.py'])
        ),
        launch_arguments={'robot_ip': LaunchConfiguration('robot_ip')}.items()
    )'''

    rs = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([rs_pkg, 'launch', 'realsense.launch.py'])  # 依你的實際檔名
        )
    )

    yolo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([yolo_pkg, 'launch', 'yolo.launch.py'])
        )
    )

    transfer = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([transfer_pkg, 'launch', 'transfer.launch.py'])
        )
    )

    path = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([path_pkg, 'launch', 'path_planning.launch.py'])
        )
    )

    return LaunchDescription([
        declare_robot_ip, declare_domain,
        set_domain,
        #tm, 
        rs, yolo, transfer, path
    ])
