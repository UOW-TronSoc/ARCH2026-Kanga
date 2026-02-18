"""Visualize the Kanga robot URDF in RViz2 with robot_state_publisher."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Resolve installed URDF and default RViz profile.
    pkg_share = get_package_share_directory('kanga_sim')
    urdf_path = os.path.join(pkg_share, 'urdf', 'kanga_robot.urdf')
    rviz_config_default = os.path.join(pkg_share, 'rviz2', 'basic.rviz')

    with open(urdf_path, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()
    # Normalize mesh paths so RViz can always resolve resources via package URIs.
    install_mesh_prefix = os.path.join(pkg_share, 'urdf') + '/'
    source_mesh_prefix = '/home/hunter/gazebo_ws/src/kanga_sim/urdf/'
    for prefix in (install_mesh_prefix, source_mesh_prefix):
        robot_description = robot_description.replace(
            f'filename="{prefix}',
            'filename="package://kanga_sim/urdf/',
        )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start the RViz2 GUI',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=rviz_config_default,
            description='Path to the RViz2 config file',
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                }
            ],
        ),
        # Publishes wheel joint states for URDF joints in RViz when Gazebo is not the source.
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[
                {
                    'robot_description': robot_description,
                    'use_sim_time': use_sim_time,
                }
            ],
        ),
        # Optional RViz process; disabled when use_rviz:=false.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
