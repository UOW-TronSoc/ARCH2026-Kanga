"""Spawn the Kanga robot URDF into an already running Gazebo instance."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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

    return LaunchDescription([
        entity_arg,
        x_arg,
        y_arg,
        z_arg,
        spawn_entity,
    ])
