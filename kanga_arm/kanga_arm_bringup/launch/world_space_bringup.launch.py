#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    use_sim = LaunchConfiguration("use_sim")
    enable_drive = LaunchConfiguration("enable_drive")

    robot_description_path = get_package_share_directory("kanga_arm_description")
    operation_path = get_package_share_directory("kanga_arm_bringup")
    simulation_path = get_package_share_directory("kanga_arm_simulation")
    arm_drive_path = get_package_share_directory("kanga_arm_drive")
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

    # World-space arm controller (publishes joint_desired_control)
    control_node = Node(
        package="kanga_arm_controller",
        executable="control_node",
        name="kanga_arm_controller",
        output="screen",
        parameters=[
            kanga_arm_config,
            operation_params,
            {"use_sim_time": use_sim},
        ],
    )

    # Joystick -> world command bridge (always available)
    joy_world_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_control_path, "launch", "joy_arm_world_control.launch.py")
        )
    )

    # Simulation when only sim is enabled
    raisim_sim_only = Node(
        package="kanga_arm_simulation",
        executable="raisim_bridge",
        name="raisim_bridge",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                ["('", use_sim, "'.lower() in ['true','1']) and ('", enable_drive, "'.lower() in ['false','0'])"]
            )
        ),
        parameters=[
            {"robot_description_path": robot_description_path},
            kanga_arm_config,
            operation_params,
            simulation_params,
            {"use_sim_time": use_sim},
        ],
    )

    # Simulation when sim and drive are both enabled.
    # Keep sim feedback isolated on /sim/joint_states in this mixed mode.
    raisim_sim_with_drive = Node(
        package="kanga_arm_simulation",
        executable="raisim_bridge",
        name="raisim_bridge_with_drive",
        output="screen",
        condition=IfCondition(
            PythonExpression(
                [
                    "('", use_sim, "'.lower() in ['true','1']) and "
                    "('", enable_drive, "'.lower() in ['true','1'])"
                ]
            )
        ),
        remappings=[
            ("joint_states", "/sim/joint_states"),
        ],
        parameters=[
            {"robot_description_path": robot_description_path},
            kanga_arm_config,
            operation_params,
            simulation_params,
            {"use_sim_time": use_sim},
        ],
    )

    # Hardware stack (ODrive nodes + mapper + feedback bridge)
    arm_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_drive_path, "launch", "arm_drive_with_mapper.launch.py")
        ),
        condition=IfCondition(enable_drive),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="If true: run Raisim simulation stack.",
        ),
        DeclareLaunchArgument(
            "enable_drive",
            default_value="true",
            description="If true: run hardware arm drive stack.",
        ),
        control_node,
        joy_world_control_launch,
        raisim_sim_only,
        raisim_sim_with_drive,
        arm_drive_launch,
    ])
