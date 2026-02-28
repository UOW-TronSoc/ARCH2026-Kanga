#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Set up paths to the necessary configuration files and packages
    robot_description_path = get_package_share_directory("kanga_arm_description")
    operation_path = get_package_share_directory("kanga_arm_bringup")
    simulation_path = get_package_share_directory("kanga_arm_simulation")
    
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
    
    # Setup raisim node
    raisim_node = Node(
        package="kanga_arm_simulation",
        executable="raisim_bridge",
        name="raisim_bridge",
        output="screen",
        parameters=[
            {"robot_description_path": robot_description_path},
            kanga_arm_config,
            operation_params,
            simulation_params
        ]
    )

    # # Setup control node
    control_node = Node(
        package="kanga_arm_controller",
        executable="joint_control_relay_node",
        name="kanga_arm_joint_control_relay",
        output="screen",
        parameters=[
            kanga_arm_config,
            operation_params,
        ]
    )

        # Setup control node
    # control_node = Node(
    #     package="kanga_arm_controller",
    #     executable="control_node",
    #     name="control_node",
    #     output="screen",
    #     parameters=[
    #         kanga_arm_config,
    #         operation_params,
    #     ]
    # )

    return LaunchDescription([
        # Start Foxglove immediately for visualization
        # foxglove,

        # After a delay, start the raisim node and control node
        TimerAction(
            period=2.0,
            actions=[
                     raisim_node, 
                     control_node, 
                     ],
        ),
    ])
