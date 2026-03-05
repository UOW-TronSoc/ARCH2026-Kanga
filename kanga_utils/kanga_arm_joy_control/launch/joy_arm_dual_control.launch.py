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

    joy_to_dual_control = Node(
        package='kanga_arm_joy_control',
        executable='joy_to_dual_control',
        name='joy_to_dual_control',
        output='screen',
        parameters=[{
            # Toggle between joint/world modes on button edge.
            'mode_toggle_button': 0,
            # Start mode: "joint" or "world".
            'start_mode': 'joint',
            # Joint mode mapping: axes -> j1..j4.
            'joint_axis_indices': [0, 1, 3, 2],
            'button_negative_j5': 10,
            'button_positive_j5': 9,
            # World mode mapping: axes -> x,y,z,pitch.
            'world_axis_indices': [0, 1, 2, 3],
            'button_negative_roll': 10,
            'button_positive_roll': 9,
            'joint_control_topic': '/kanga_arm/joint_control',
            'world_control_topic': 'kanga_arm/world_state_control',
            'joint_velocity_scale': 1.0,
            'linear_scale': 1.0,
            'pitch_scale': 1.0,
            'roll_scale': 1.0,
        }],
    )

    return LaunchDescription([
        joy_node,
        joy_to_dual_control,
    ])

