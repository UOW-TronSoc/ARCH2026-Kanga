#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    use_joy = LaunchConfiguration("use_joy")

    # Set up paths to the necessary configuration files and packages
    robot_description_path = get_package_share_directory("kanga_arm_description")
    operation_path = get_package_share_directory("kanga_arm_bringup")
    simulation_path = get_package_share_directory("kanga_arm_simulation")
    arm_drive_path = get_package_share_directory("kanga_arm_drive")
    joy_control_path = get_package_share_directory("kanga_arm_joy_control")
    
    # Load config for leg configuration
    kanga_arm_config = os.path.join(
        robot_description_path,
        'config',
        'kanga_arm_config.yaml'
        )
    
    # Load config for operationparameters
    operation_params = os.path.join(
        operation_path,
        'config',
        'operation.yaml'
        )
    
    # Load config for simulation parameters
    simulation_params = os.path.join(
        simulation_path,
        'config',
        'simulation.yaml'
        )
    
    # Setup raisim node (sim mode)
    raisim_node = Node(
        package="kanga_arm_simulation",
        executable="raisim_bridge",
        name="raisim_bridge",
        output="screen",
        condition=IfCondition(use_sim),
        parameters=[
            {"robot_description_path": robot_description_path},
            kanga_arm_config,
            operation_params,
            simulation_params,
            {"use_sim_time": use_sim},
        ]
    )

    # Setup relay node (always runs; behavior selected by downstream stack)
    control_node = Node(
        package="kanga_arm_controller",
        executable="joint_control_relay_node",
        name="kanga_arm_joint_control_relay",
        output="screen",
        parameters=[
            kanga_arm_config,
            operation_params,
            {"use_sim_time": use_sim},
        ]
    )

    # Setup hardware arm stack (real robot mode)
    arm_drive_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(arm_drive_path, "launch", "arm_drive_with_mapper.launch.py")
        ),
        condition=UnlessCondition(use_sim),
    )

    joy_joint_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(joy_control_path, "launch", "joy_arm_joint_control.launch.py")
        ),
        condition=IfCondition(use_joy),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim",
            default_value="true",
            description="If true: run Raisim simulation. If false: run hardware arm drive stack.",
        ),
        DeclareLaunchArgument(
            "use_joy",
            default_value="true",
            description="If true: start joystick input -> /kanga_arm/joint_control.",
        ),
        control_node,
        joy_joint_control_launch,
        raisim_node,
        arm_drive_launch,
    ])
