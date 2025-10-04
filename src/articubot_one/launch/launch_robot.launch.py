import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable

def generate_launch_description():
    package_name = 'articubot_one'

    # Set use_sim_time to false for real hardware
    use_sim_time = 'false'

    # Robot State Publisher
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={
            'use_sim_time': use_sim_time, 
            'use_ros2_control': 'false'
        }.items()
    )

    # GPIO Motor Controller
    motor_controller = Node(
        package=package_name,
        executable='motor_controller.py',
        name='motor_controller',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Keyboard control
    keyboard = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','keyboard.launch.py'
        )])
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
            'scan_mode': 'Standard',
            'use_sim_time': use_sim_time
        }]
    )

    # Twist Mux
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[twist_mux_params, {'use_sim_time': use_sim_time}],
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
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1'),
        rsp,
        motor_controller,
        keyboard,
        rplidar,
        twist_mux,
        base_to_laser,
        base_footprint_transform,
    ])