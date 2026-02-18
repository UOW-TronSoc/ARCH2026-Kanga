"""Spawn the Kanga robot URDF into an already running Gazebo instance."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('kanga_sim')
    urdf_path_str = os.path.join(pkg_share, 'urdf', 'kanga_robot.urdf')

    with open(urdf_path_str, 'r', encoding='utf-8') as urdf_file:
        robot_description = urdf_file.read()

    # Runtime args for naming and positioning the spawned model in Gazebo.
    entity_arg = DeclareLaunchArgument(
        'entity',
        default_value='kanga_robot',
        description='Name for the spawned robot entity in Gazebo',
    )
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='0.0',
        description='Spawn position X in meters',
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='Spawn position Y in meters',
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.0',
        description='Spawn position Z in meters',
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    # Resolve URDF from the installed package share path.
    urdf_path = PathJoinSubstitution(
        [FindPackageShare('kanga_sim'), 'urdf', 'kanga_robot.urdf']
    )

    # Call gazebo_ros factory script to insert the URDF into a running world.
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-entity',
            LaunchConfiguration('entity'),
            '-file',
            urdf_path,
            '-x',
            LaunchConfiguration('x'),
            '-y',
            LaunchConfiguration('y'),
            '-z',
            LaunchConfiguration('z'),
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    return LaunchDescription([
        entity_arg,
        x_arg,
        y_arg,
        z_arg,
        use_sim_time_arg,
        robot_state_publisher,
        joint_state_publisher,
        spawn_entity,
    ])
