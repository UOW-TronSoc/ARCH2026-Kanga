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
            'axis_indices': [1, 0, 5, 2],
            # Also send raw axis 0 into joint_control j1 for future use.
            'joint_axis0_index': 0,
            # Trigger mapping for j6: axis5 -> +1, axis4 -> -1.
            'axis_negative_j6': 3,
            'axis_positive_j6': 4,
            'j6_axis_pressed_threshold': 1.0,
            # Reuse the same button pair currently used for joint-5 control.
            'button_negative_roll': 4,
            'button_positive_roll': 5,
            'linear_scale': 1.0,
            'pitch_scale': 1.0,
            'roll_scale': 1.0,
            'world_control_topic': 'kanga_arm/world_state_control',
            'joint_control_topic': '/kanga_arm/joint_control',
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_world_state_control,
    ])
