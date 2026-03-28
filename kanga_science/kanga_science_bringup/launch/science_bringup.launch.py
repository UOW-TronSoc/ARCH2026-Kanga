from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ros2socketcan_bridge = Node(
        package="ros2socketcan_bridge",
        executable="ros2socketcan",
        name="ros2socketcan",
        output="screen",
    )

    science_temperature_mapper = Node(
        package="kanga_science_drive",
        executable="science_temperature_mapper",
        name="science_temperature_mapper",
        output="screen",
    )

    return LaunchDescription([
        ros2socketcan_bridge,
        science_temperature_mapper,
    ])
