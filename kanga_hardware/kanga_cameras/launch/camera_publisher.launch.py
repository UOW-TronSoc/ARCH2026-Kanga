import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("kanga_cameras")
    default_config = str(Path(pkg_share) / "config" / "cameras.yaml")

    config_arg = DeclareLaunchArgument(
        "config_file",
        default_value=default_config,
        description="Path to cameras YAML config (video_id and name per camera).",
    )

    camera_node = Node(
        package="kanga_cameras",
        executable="camera_publisher_node",
        name="camera_publisher",
        output="screen",
        parameters=[
            {"config_file": LaunchConfiguration("config_file")},
        ],
    )

    actions = [config_arg, camera_node]

    # ZED 2i camera (requires zed_wrapper from slam workspace to be sourced)
    # try:
    zed_wrapper_share = get_package_share_directory("zed_wrapper")
    zed_launch_path = os.path.join(zed_wrapper_share, "launch", "zed_camera.launch.py")
    zed_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_path),
        #launch_arguments={"camera_model": "zed2i"}.items(),
        launch_arguments={'camera_model': "zed2i",
            'publish_map_tf': 'false',
            "publish_tf": "false",
            "publish_odom_tf": "false",
            "publish_imu_tf": "true",
            }.items(),


    )
    actions.append(zed_launch)
    # except PackageNotFoundError:
    #     pass

    # Relay ZED rectified RGB compressed image to /camera/front
    zed_relay_node = Node(
        package="topic_tools",
        executable="relay",
        name="zed_to_camera_front_relay",
        output="screen",
        parameters=[
            {
                "input_topic": "/zed/zed_node/rgb/color/rect/image/compressed",
                "output_topic": "/camera/front",
            },
        ],
    )
    actions.append(zed_relay_node)

    return LaunchDescription(actions)
