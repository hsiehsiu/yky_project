# launch/driver.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    """
    Launch file for the Dynpick F/T Sensor Driver Node.

    It declares all necessary parameters for the C++ node and starts the executable.
    """

    # 1. Declare Launch Arguments (used for command line or nested launches)
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='/dev/ttyUSB0',
        description='Serial device name for the Dynpick sensor.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='sensor_link',
        description='The frame ID for the published WrenchStamped messages.'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='100.0',  # [優化] 從 1000 Hz 降到 100 Hz，減少 CPU 負擔
        description='The frequency (Hz) at which the sensor data is queried and published.'
    )

    calibration_arg = DeclareLaunchArgument(
        'acquire_calibration',
        default_value='true',
        description='Set to true to acquire calibration factors from the sensor upon startup.'
    )
    
    frequency_div_arg = DeclareLaunchArgument(
        'frequency_div',
        default_value='1',
        description='The hardware frequency divider (1, 2, 4, or 8) used for filtering.'
    )

    # 2. Define the Nodes to be launched
    driver_node = Node(
        package='wacoh_dynpick',
        executable='dynpick_driver_node',
        name='dynpick_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'device': LaunchConfiguration('device'),
            'frame_id': LaunchConfiguration('frame_id'),
            'rate': LaunchConfiguration('rate'),
            'acquire_calibration': LaunchConfiguration('acquire_calibration'),
            'frequency_div': LaunchConfiguration('frequency_div'),
        }]
    )
    # [修復] force_listener 改用不同變數名稱，避免覆蓋 driver_node
    listener_node = Node(
        package='wacoh_dynpick',
        executable='force_listener',
        name='force_node',
        output='screen',
        emulate_tty=True,
        parameters=[]
    )

    # 3. Return the launch description
    return LaunchDescription([
        device_arg,
        frame_id_arg,
        rate_arg,
        calibration_arg,
        frequency_div_arg,
        driver_node,
        listener_node  # [修復] 同時啟動兩個節點
    ])
