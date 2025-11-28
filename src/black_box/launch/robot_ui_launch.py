from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='black_box',
            executable='black_box_node',
            name='black_box_node',
            output='screen'
        ),
        Node(
            package='black_box',
            executable='robot_controller_node',
            name='robot_controller_node',
            output='screen'
        )
    ])
