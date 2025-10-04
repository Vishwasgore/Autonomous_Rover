import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'articubot_one'

    # Robot State Publisher - NO ros2_control for real hardware
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'use_ros2_control': 'false'}.items()
    )

    # GPIO Motor Controller (REAL HARDWARE) - REPLACES ros2_control
    motor_controller = Node(
        package=package_name,
        executable='motor_controller.py',
        name='motor_controller',
        output='screen'
    )

    # Keyboard control
    keyboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','keyboard.launch.py'
        )])
    )

    # YDLIDAR with correct parameters from your reference
    ydlidar = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_node',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'ignore_array': "",
            'baudrate': 230400,           # Changed from serial_baudrate
            'lidar_type': 1,              # Specific to your LIDAR model
            'device_type': 0,
            'isSingleChannel': False,
            'intensity': False,
            'intensity_bit': 0,
            'sample_rate': 9,
            'abnormal_check_count': 4,
            'fixed_resolution': True,
            'reversion': False,
            'inverted': False,
            'auto_reconnect': True,
            'support_motor_dtr': False,   # Changed from True
            'angle_max': 180.0,
            'angle_min': -180.0,
            'range_max': 64.0,            # Increased from 16.0
            'range_min': 0.01,            # More precise
            'frequency': 10.0,
            'invalid_range_is_inf': False,
            'debug': False
        }],
        respawn=True,
        respawn_delay=3.0
    )

    # Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/cmd_vel')]  # Send to motor_controller
    )

    # Static transforms
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    base_footprint_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    )

    return LaunchDescription([
        rsp,
        motor_controller,  # GPIO control instead of ros2_control
        keyboard,
        ydlidar,
        twist_mux,
        base_to_laser,
        base_footprint_transform,
    ])