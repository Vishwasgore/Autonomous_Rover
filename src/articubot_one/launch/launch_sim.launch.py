import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'articubot_one'
    
    # Get the path to the world file
    world_file = os.path.join(
        get_package_share_directory(package_name),
        'worlds',
        'obstacles.world'
    )
    
    # Robot State Publisher with sim time
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), 
        launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Gazebo launch with the obstacles world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py'
        )]),
        launch_arguments={
            'world': world_file,
            'verbose': 'true'
        }.items()
    )
    
    # Spawn the robot entity
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'bottybot'],
        output='screen'
    )

    # Add the SLAM launch file to your main launch
    slam = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','online_async_slam.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )


    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        slam
    ])