from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'deadzone': 0.5,
        }],
    )

    joy_to_joint_control = Node(
        package='kanga_arm_joy_control',
        executable='joy_to_joint_control',
        name='joy_to_joint_control',
        output='screen',
        parameters=[{
            'axis_indices': [0, 1, 5, 2],
            'axis_negative_j6': 3,
            'axis_positive_j6': 4,
            'j6_axis_pressed_threshold': 1.0,
            'button_negative_j5': 4,
            'button_positive_j5': 5,
            'joint_control_topic': '/kanga_arm/joint_control',
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_joint_control,
    ])
