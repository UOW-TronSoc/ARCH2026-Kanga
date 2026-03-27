"""Reset Gazebo robot entity, respawn robot, then bring up Nav2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    kanga_nav2_share = get_package_share_directory('kanga_nav2')
    kanga_sim_share = get_package_share_directory('kanga_sim')

    default_map = os.path.join(kanga_nav2_share, 'maps', 'fence_map.yaml')
    default_params = os.path.join(kanga_nav2_share, 'config', 'nav2_params.yaml')

    entity = LaunchConfiguration('entity')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    delete_entity = ExecuteProcess(
        cmd=[
            'ros2',
            'service',
            'call',
            '/delete_entity',
            'gazebo_msgs/srv/DeleteEntity',
            ["{name: '", entity, "'}"],
        ],
        output='screen',
    )

    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kanga_sim_share, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={
            'entity': entity,
            'use_sim_time': use_sim_time,
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }.items(),
    )

    nav2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(kanga_nav2_share, 'launch', 'nav2_sim.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_rviz': use_rviz,
            'map': map_yaml,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('entity', default_value='kanga_robot'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('z', default_value='0.0'),
        delete_entity,
        TimerAction(period=2.0, actions=[spawn_robot]),
        TimerAction(period=6.0, actions=[nav2_stack]),
    ])
