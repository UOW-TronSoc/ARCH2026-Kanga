import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    arm_bringup_share = get_package_share_directory("kanga_arm_bringup")

    ros2socketcan_bridge = Node(
        package="ros2socketcan_bridge",
        executable="ros2socketcan",
        name="ros2socketcan",
        output="screen",
    )

    hybrid_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_bringup_share, "launch", "hybrid_bringup.launch.py")
        ),
        launch_arguments={
            "use_sim": "false",
            "enable_drive": "true",
            "enable_joy": "false",
            "tool_mode": "end_effector",
        }.items(),
    )

    return LaunchDescription([
        ros2socketcan_bridge,
        hybrid_control,
    ])

