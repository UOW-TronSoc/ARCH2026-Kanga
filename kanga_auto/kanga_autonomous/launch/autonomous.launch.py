from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='kanga_autonomous',
            executable='movement_sequence',
            name='movement_sequence',
            output='screen',
        ),
    ])
