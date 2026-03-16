from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.substitutions import Command, FindExecutable


def generate_launch_description():
    pkg_share = get_package_share_directory("kanga_description")
    xacro_path = os.path.join(
        pkg_share, "urdf", "assemblies", "kanga_arm_scoop_descr.urdf.xacro"
    )
    rviz_config_path = os.path.join(pkg_share, "rviz", "rsp_kanga_arm_scoop.rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher", default="false")
    align_with_sim = LaunchConfiguration("align_with_sim", default="false")

    nodes = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock time (set true when visualizing simulation).",
        ),
        DeclareLaunchArgument(
            "use_joint_state_publisher",
            default_value="false",
            description="Run joint_state_publisher(_gui). Keep false when another node publishes /joint_states.",
        ),
        DeclareLaunchArgument(
            "align_with_sim",
            default_value="false",
            description="Publish world->base_link so robot aligns with sim arm position (z=2). Set RViz Fixed Frame to 'world'.",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[
                {"robot_description": robot_description, "use_sim_time": use_sim_time}
            ],
            output="screen",
        ),
    ]

    nodes.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_base_link",
            arguments=["-0.379844", "0.1575", "1.48143", "0", "0", "0", "world", "base_link"],
            condition=IfCondition(align_with_sim),
            output="screen",
        )
    )

    try:
        get_package_share_directory("joint_state_publisher_gui")
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                condition=IfCondition(use_joint_state_publisher),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )
    except PackageNotFoundError:
        try:
            get_package_share_directory("joint_state_publisher")
            nodes.append(
                Node(
                    package="joint_state_publisher",
                    executable="joint_state_publisher",
                    condition=IfCondition(use_joint_state_publisher),
                    parameters=[{"use_sim_time": use_sim_time}],
                    output="screen",
                )
            )
        except PackageNotFoundError:
            pass

    try:
        get_package_share_directory("rviz2")
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config_path],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            )
        )
    except PackageNotFoundError:
        pass

    return LaunchDescription(nodes)
