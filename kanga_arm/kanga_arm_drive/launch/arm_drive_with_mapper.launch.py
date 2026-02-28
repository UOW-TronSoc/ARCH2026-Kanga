import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("kanga_arm_drive")
    odrive_multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "odrive_multi_arm.launch.py")
        )
    )

    arm_mapper_node = Node(
        package="kanga_arm_drive",
        executable="arm_command_mapper",
        name="arm_command_mapper",
        output="screen",
    )

    return LaunchDescription([
        odrive_multi_launch,
        arm_mapper_node,
    ])
