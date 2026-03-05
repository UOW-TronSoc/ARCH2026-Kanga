import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    can_interface = LaunchConfiguration("can_interface")
    tool_mode = LaunchConfiguration("tool_mode")
    package_share = get_package_share_directory("kanga_arm_drive")
    odrive_multi_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(package_share, "launch", "odrive_multi_arm.launch.py")
        )
    )

    arm_mapper_node = Node(
        package="kanga_arm_drive",
        executable="arm_command_mapper",
        name="arm_command_mapper",
        output="screen",
    )

    arm_feedback_bridge_node = Node(
        package="kanga_arm_drive",
        executable="arm_feedback_bridge",
        name="arm_feedback_bridge",
        output="screen",
    )

    arm_end_effector_mapper_node = Node(
        package="kanga_arm_drive",
        executable="arm_end_effector_mapper",
        name="arm_end_effector_mapper",
        output="screen",
        condition=IfCondition(
            PythonExpression([
                "'", tool_mode, "'.lower() in ['end_effector', 'end-effector', 'end effector']"
            ])
        ),
        parameters=[{
            "can_interface": can_interface,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "can_interface",
            default_value="can0",
            description="CAN interface for transmit topic CAN/<interface>/transmit",
        ),
        DeclareLaunchArgument(
            "tool_mode",
            default_value="end_effector",
            description="Tool mode: end_effector enables end_effector_mapper, gripper disables it",
        ),
        odrive_multi_launch,
        arm_mapper_node,
        arm_feedback_bridge_node,
        arm_end_effector_mapper_node,
    ])
