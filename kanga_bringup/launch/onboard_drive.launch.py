import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    drive_share = get_package_share_directory('kanga_drive')
    onboard_control_share = get_package_share_directory('kanga_onboard_control')
    battery_share = get_package_share_directory('kanga_battery')
    cameras_share = get_package_share_directory('kanga_cameras')

    drive_with_mapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(drive_share, 'launch', 'drive_with_mapper.launch.py'),
        )
    )

    drive_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(onboard_control_share, 'launch', 'drive_control.launch.py'),
        )
    )

    daly_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(battery_share, 'launch', 'daly_launch.py'),
        )
    )

    camera_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(cameras_share, 'launch', 'camera_publisher.launch.py'),
        )
    )

    return LaunchDescription([
        drive_with_mapper,
        drive_control,
        daly_launch,
        camera_publisher_launch,
    ])
