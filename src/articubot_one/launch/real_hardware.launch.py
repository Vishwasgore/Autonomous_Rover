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

    # GPIO Motor Controller (REAL HARDWARE)
    motor_controller = Node(
        package=package_name,
        executable='motor_controller.py',
        name='motor_controller',
        output='screen'
    )

    # RPLIDAR
    rplidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[{
            'serial_port': '/dev/ttyUSB0',
            'frame_id': 'laser_frame',
            'angle_compensate': True,
            'scan_mode': 'Standard'
        }]
    )

    # Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params],
        remappings=[('/cmd_vel_out','/cmd_vel')]
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
        motor_controller,
        rplidar,
        twist_mux,
        base_to_laser,
        base_footprint_transform,
    ])