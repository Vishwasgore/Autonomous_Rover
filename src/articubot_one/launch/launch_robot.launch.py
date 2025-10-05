import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    package_name = 'articubot_one'

    # Robot State Publisher - Make sure it's not publishing the same transforms
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={
            'use_sim_time': 'false', 
            'use_ros2_control': 'false'
        }.items()
    )

    # GPIO Motor Controller
    motor_controller = Node(
        package=package_name,
        executable='motor_controller.py',
        name='motor_controller',
        output='screen'
    )

    # MPU6050 IMU Driver
    mpu6050_driver = Node(
        package=package_name,
        executable='mpu6050_driver.py',
        name='mpu6050_driver',
        output='screen'
    )

    # IMU Filter - FIXED parameters to avoid TF conflicts
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,
            'world_frame': 'odom',  # Changed from 'enu' to 'odom'
            'fixed_frame': 'imu_link',
            'publish_tf': False,    # Set to False - we'll handle TF separately
            'publish_debug_topics': False,
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data'),
            ('/imu/data', '/imu/filtered')
        ]
    )

    # Simple IMU to Odometry converter
    imu_to_odom = Node(
        package=package_name,
        executable='imu_to_odom.py',
        name='imu_to_odom',
        output='screen'
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

    # Static transforms - ONLY ONE SET OF STATIC TRANSFORMS
    base_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser_frame']
    )
    
    # Only include base_footprint_to_base_link if your URDF doesn't already have it
    # base_footprint_transform = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='base_footprint_to_base_link',
    #     arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
    # )

    base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu',
        arguments=['0', '0', '0.05', '0', '0', '0', 'base_link', 'imu_link']
    )

    return LaunchDescription([
        rsp,
        motor_controller,
        mpu6050_driver,
        imu_filter,
        imu_to_odom,
        keyboard,
        rplidar,
        twist_mux,
        base_to_laser,
        # base_footprint_transform,  # Commented out - might be duplicate
        base_to_imu,
    ])