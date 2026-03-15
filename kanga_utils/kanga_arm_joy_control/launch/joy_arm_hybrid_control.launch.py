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

    joy_to_hybrid_control = Node(
        package='kanga_arm_joy_control',
        executable='joy_to_hybrid_control',
        name='joy_to_hybrid_control',
        output='screen',
        parameters=[{
            'mode_toggle_button': 0,
            'start_mode': 'joint',
            # Joint mode: axes -> j1, j2, j3, j4
            'joint_axis_indices': [0, 1, 3, 2],
            # EE mode: axes -> X, Z, pitch (end-effector frame)
            'ee_axis_indices': [1, 0, 3, 2],
            'axis_negative_j6': 4,
            'axis_positive_j6': 5,
            'j6_axis_pressed_threshold': 1.0,
            'button_negative_roll': 9,
            'button_positive_roll': 10,
            'joint_velocity_scale': 1.0,
            'linear_scale': 1.0,
            'pitch_scale': 1.0,
            'roll_scale': 1.0,
            'joint_control_topic': '/kanga_arm/joint_control',
            'ee_control_topic': 'kanga_arm/ee_state_control',
            'mode_topic': 'kanga_arm/control_mode_joint',
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_hybrid_control,
    ])
