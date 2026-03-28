"""Nav2 bringup for the real Kanga robot (no Gazebo).

Run your robot / perception bringup first (e.g. pipeline with RTAB-Map), which must provide:
  - /odom                       (nav_msgs/Odometry)
  - TF: map -> odom             (localization)
  - TF: odom -> base_link       (odometry)

Without that stack, costmaps will warn that ``map`` / ``odom`` frames do not exist.

For bench testing Nav2 alone, pass ``use_fake_tf:=true`` to publish identity static
transforms map->odom and odom->base_link (do not use on the real robot while SLAM runs).

This launch outputs:
  - /cmd_vel                    velocity commands (geometry_msgs/Twist)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _static_tf_node(name, parent, child, condition):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--qx', '0', '--qy', '0', '--qz', '0', '--qw', '1',
            '--frame-id', parent, '--child-frame-id', child,
        ],
        condition=condition,
    )


def generate_launch_description():
    pkg_share = get_package_share_directory('kanga_nav2')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    default_map = os.path.join(pkg_share, 'maps', 'fence_map.yaml')
    default_params = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_share, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    use_fake_tf = LaunchConfiguration('use_fake_tf')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'yaml_filename': map_yaml},
        ],
    )

    lifecycle_manager_map = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']},
        ],
    )

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml,
            'params_file': params_file,
            'autostart': 'true',
            'use_composition': 'False',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_nav2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    fake_tf_cond = IfCondition(use_fake_tf)
    fake_tf_map_odom = _static_tf_node(
        'fake_tf_map_to_odom', 'map', 'odom', fake_tf_cond)
    fake_tf_odom_base = _static_tf_node(
        'fake_tf_odom_to_base_link', 'odom', 'base_link', fake_tf_cond)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument(
            'use_fake_tf',
            default_value='false',
            description='If true, publish static map->odom and odom->base_link (testing only; '
            'conflicts with real SLAM / odometry).',
        ),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        fake_tf_map_odom,
        fake_tf_odom_base,
        map_server,
        lifecycle_manager_map,
        nav2_bringup,
        rviz,
    ])
