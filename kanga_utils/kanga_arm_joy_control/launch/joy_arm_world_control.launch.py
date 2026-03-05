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

    joy_to_world_state_control = Node(
        package='kanga_arm_joy_control',
        executable='joy_to_world_state_control',
        name='joy_to_world_state_control',
        output='screen',
        parameters=[{
            # Axis mapping: 0->X, 1->Y, 2->Z, 3->pitch.
            'axis_indices': [1, 0, 3, 2],
            # Reuse the same button pair currently used for joint-5 control.
            'button_negative_roll': 10,
            'button_positive_roll': 9,
            'linear_scale': 1.0,
            'pitch_scale': 1.0,
            'roll_scale': 1.0,
            'world_control_topic': 'kanga_arm/world_state_control',
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_world_state_control,
    ])

