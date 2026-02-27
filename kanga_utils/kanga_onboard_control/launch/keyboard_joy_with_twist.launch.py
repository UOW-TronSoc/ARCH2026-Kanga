import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    keyboard_joy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('keyboard_joy'),
                'launch',
                'keyboard_joy.launch.py',
            )
        ),
        launch_arguments={'device': '/dev/input/event1'}.items(),
    )

    joy_to_twist_node = Node(
        package='kanga_onboard_control',
        executable='joy_to_twist',
        name='joy_to_twist',
        output='screen',
    )

    return LaunchDescription([
        keyboard_joy_launch,
        joy_to_twist_node,
    ])
