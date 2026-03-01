#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    end_effector_config = LaunchConfiguration("end_effector_config")

    robot_description_path = get_package_share_directory("kanga_arm_description")
    operation_path = get_package_share_directory("kanga_arm_bringup")
    simulation_path = get_package_share_directory("kanga_arm_simulation")
    joy_control_path = get_package_share_directory("kanga_arm_joy_control")

    kanga_arm_config = os.path.join(
        robot_description_path,
        "config",
        "kanga_arm_config.yaml",
    )

    operation_params = os.path.join(
        operation_path,
        "config",
        "operation.yaml",
    )

    simulation_params = os.path.join(
        simulation_path,
        "config",
        "simulation.yaml",
    )

    raisim_node = Node(
        package="kanga_arm_simulation",
        executable="raisim_bridge",
        name="raisim_bridge",
        output="screen",
        parameters=[
            {"robot_description_path": robot_description_path},
            kanga_arm_config,
            operation_params,
            simulation_params,
        ],
    )

    control_node = Node(
        package="kanga_arm_controller",
        executable="control_node",
        name="kanga_arm_controller",
        output="screen",
        parameters=[
            {"end_effector_config": end_effector_config},
            kanga_arm_config,
            operation_params,
        ],
    )

    joy_world_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_control_path, "launch", "joy_arm_world_control.launch.py")
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "end_effector_config",
            default_value="roll_tool",
            description="End-effector mode for world-space kinematics (roll_tool or scoop)",
        ),
        TimerAction(
            period=2.0,
            actions=[
                raisim_node,
                control_node,
                joy_world_control_launch,
            ],
        ),
    ])

