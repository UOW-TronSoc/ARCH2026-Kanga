from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.substitutions import Command, FindExecutable

def generate_launch_description():
    pkg_share = get_package_share_directory("kanga_description")
    xacro_path = os.path.join(pkg_share, "urdf", "core", "kanga_core_simple_descr.urdf.xacro")
    rviz_config_path = os.path.join(pkg_share, "rviz", "rsp_core_simple.rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_path]), value_type=str
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
            output="screen",
        ),
    ]

    try:
        get_package_share_directory("joint_state_publisher_gui")
        nodes.append(
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
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
                output="screen",
            )
        )
    except PackageNotFoundError:
        pass

    return LaunchDescription(nodes)
