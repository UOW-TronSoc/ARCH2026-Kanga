import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('kanga_drive')
    odrive_multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, 'launch', 'odrive_multi.launch.py')
        )
    )

    twist_to_percent_node = Node(
        package='kanga_drive',
        executable='twist_to_percent',
        name='twist_to_percent',
        output='screen',
        parameters=[{
            'max_linear_x': 0.4,
            'max_linear_y': 0.4,
            'max_angular_z': 2.2,
        }],
    )

    wheel_mapper_node = Node(
        package='kanga_drive',
        executable='wheel_command_mapper',
        name='wheel_command_mapper',
        output='screen',
    )

    return LaunchDescription([
        odrive_multi_launch,
        twist_to_percent_node,
        wheel_mapper_node,
    ])
