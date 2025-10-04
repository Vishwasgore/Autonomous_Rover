from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    keyboard_teleop = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        prefix='xterm -e',  # Opens in new terminal
        remappings=[('/cmd_vel', '/cmd_vel_keyboard')],
        output='screen'
    )

    return LaunchDescription([
        keyboard_teleop,
    ])