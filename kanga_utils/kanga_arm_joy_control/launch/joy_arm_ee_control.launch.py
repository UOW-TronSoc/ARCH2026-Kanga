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

    joy_to_ee_state_control = Node(
        package='kanga_arm_joy_control',
        executable='joy_to_ee_state_control',
        name='joy_to_ee_state_control',
        output='screen',
        parameters=[{
            # Axis mapping: 0->X (forward), 1->Y (left), 2->Z (up), 3->pitch (end-effector frame).
            'axis_indices': [1, 0, 3, 2],
            'joint_axis0_index': 0,
            'axis_negative_j6': 5,
            'axis_positive_j6': 6,
            'j6_axis_pressed_threshold': 1.0,
            'button_negative_roll': 9,
            'button_positive_roll': 10,
            'linear_scale': 1.0,
            'pitch_scale': 1.0,
            'roll_scale': 1.0,
            'ee_control_topic': 'kanga_arm/ee_state_control',
            'joint_control_topic': '/kanga_arm/joint_control',
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_ee_state_control,
    ])
