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

    wheel_mapper_node = Node(
        package='kanga_drive',
        executable='wheel_command_mapper',
        name='wheel_command_mapper',
        output='screen',
    )

    return LaunchDescription([
        odrive_multi_launch,
        wheel_mapper_node,
    ])
