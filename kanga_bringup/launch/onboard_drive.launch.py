import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    drive_share = get_package_share_directory('kanga_drive')
    onboard_control_share = get_package_share_directory('kanga_onboard_control')

    drive_with_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drive_share, 'launch', 'drive_with_mapper.launch.py'),
        )
    )

    keyboard_twist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(onboard_control_share, 'launch', 'keyboard_joy_with_twist.launch.py'),
        )
    )

    return LaunchDescription([
        drive_with_mapper,
        keyboard_twist,
    ])
