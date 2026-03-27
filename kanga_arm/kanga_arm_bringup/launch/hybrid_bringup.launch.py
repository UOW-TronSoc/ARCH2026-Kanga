#!/usr/bin/env python3
"""
Hybrid arm bringup: joint + EE control with button toggle.

Runs both joint_control_relay and control_node (EE mode) with remapped outputs.
MUX selects which controller's output goes to joint_desired_control based on
joy_to_hybrid_control's mode (button 1 toggle).
"""
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
    enable_joy = LaunchConfiguration("enable_joy")
    tool_mode = LaunchConfiguration("tool_mode")

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

    # Joint-space controller: remap output so MUX can subscribe
    joint_control_relay = Node(
        package="kanga_arm_controller",
        executable="joint_control_relay_node",
        name="kanga_arm_joint_control_relay",
        output="screen",
        remappings=[
            ("joint_desired_control", "joint_desired_control_from_joint"),
        ],
        parameters=[
            kanga_arm_config,
            operation_params,
            {"use_sim_time": use_sim},
        ],
    )

    # EE-space controller: remap output so MUX can subscribe
    ee_control_node = Node(
        package="kanga_arm_controller",
        executable="control_node",
        name="kanga_arm_controller",
        output="screen",
        remappings=[
            ("joint_desired_control", "joint_desired_control_from_ee"),
        ],
        parameters=[
            kanga_arm_config,
            operation_params,
            {"use_sim_time": use_sim},
            {"control_input_frame": "end_effector"},
            {"end_effector_config": tool_mode},
        ],
    )

    # MUX: selects joint_desired_control_from_joint or _from_ee -> joint_desired_control
    joint_desired_control_mux = Node(
        package="kanga_arm_controller",
        executable="joint_desired_control_mux",
        name="joint_desired_control_mux",
        output="screen",
    )

    # Hybrid joy control (button 1 toggles joint/EE mode)
    joy_hybrid_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_control_path, "launch", "joy_arm_hybrid_control.launch.py")
        ),
        condition=IfCondition(enable_joy),
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
            {"end_effector_config": tool_mode},
        ],
    )

    # Simulation when sim and drive are both enabled
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
            {"end_effector_config": tool_mode},
        ],
    )

    # Hardware arm stack
    arm_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_drive_path, "launch", "arm_drive_with_mapper.launch.py")
        ),
        launch_arguments={
            "tool_mode": tool_mode,
        }.items(),
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
        DeclareLaunchArgument(
            "enable_joy",
            default_value="true",
            description="If true: run hybrid joystick control stack.",
        ),
        DeclareLaunchArgument(
            "tool_mode",
            default_value="end_effector",
            description="Tool mode: end_effector enables end_effector mapper, gripper disables it.",
        ),
        joint_control_relay,
        ee_control_node,
        joint_desired_control_mux,
        joy_hybrid_launch,
        raisim_sim_only,
        raisim_sim_with_drive,
        arm_drive_launch,
    ])
