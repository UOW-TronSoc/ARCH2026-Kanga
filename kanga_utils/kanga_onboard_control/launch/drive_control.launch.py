from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Physical joystick (e.g. gamepad on /dev/input/js0)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'device_id': 0}],
        output='screen',
    )

    # Converts /joy -> /cmd_vel
    joy_to_twist_node = Node(
        package='kanga_onboard_control',
        executable='joy_to_twist',
        name='joy_to_twist',
        output='screen',
    )

    # A button = disable drive (IDLE), B button = enable drive (CLOSED_LOOP)
    joy_drive_enable_node = Node(
        package='kanga_onboard_control',
        executable='joy_drive_enable',
        name='joy_drive_enable',
        output='screen',
    )

    return LaunchDescription([
        joy_node,
        joy_to_twist_node,
        joy_drive_enable_node,
    ])
