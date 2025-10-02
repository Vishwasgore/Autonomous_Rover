import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory("articubot_one"), 'config', 'mapper_params_online_async.yaml')

    start_async_slam_toolbox_node = Node(
        parameters=[
          params_file,
          {'use_sim_time': True}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen')

    ld = LaunchDescription()
    ld.add_action(start_async_slam_toolbox_node)
    return ld
