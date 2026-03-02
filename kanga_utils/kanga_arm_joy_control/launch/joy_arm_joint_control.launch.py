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
            'axis_indices': [0, 1, 3, 2],
            'button_negative_j5': 10,
            'button_positive_j5': 9,
            'joint_control_topic': '/kanga_arm/joint_control',
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_joint_control,
    ])
