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
        default_value='1000.0',
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

    # 2. Define the Node to be launched
    driver_node = Node(
        package='wacoh_dynpick',
        executable='dynpick_driver_node',
        name='dynpick_driver_node',
        output='screen',
        emulate_tty=True, # Recommended for seeing printouts/logs immediately
        parameters=[{
            # Map launch arguments to node parameters
            'device': LaunchConfiguration('device'),
            'frame_id': LaunchConfiguration('frame_id'),
            'rate': LaunchConfiguration('rate'),
            'acquire_calibration': LaunchConfiguration('acquire_calibration'),
            'frequency_div': LaunchConfiguration('frequency_div'),
        }]
    )

    # 3. Return the launch description
    return LaunchDescription([
        device_arg,
        frame_id_arg,
        rate_arg,
        calibration_arg,
        frequency_div_arg,
        driver_node
    ])
