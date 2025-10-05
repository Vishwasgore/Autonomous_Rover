from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    imu_filter = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'use_mag': False,  # MPU6050 doesn't have magnetometer
            'world_frame': 'enu',
            'fixed_frame': 'imu_link',
            'publish_tf': True,
            'publish_debug_topics': True,
            'orientation_stddev': 0.05,
            'gain': 0.1,
            'zeta': 0.05
        }],
        remappings=[
            ('/imu/data_raw', '/imu/data'),
            ('/imu/data', '/imu/filtered')
        ]
    )
    
    return LaunchDescription([
        imu_filter,
    ])