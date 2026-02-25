import os
from typing import List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _load_config(config_path: str) -> List[dict]:
    with open(config_path, "r", encoding="utf-8") as config_file:
        data = yaml.safe_load(config_file) or {}

    params = data.get("kanga_drive", {}).get("ros__parameters", {})
    nodes = params.get("odrive_nodes", [])
    if not isinstance(nodes, list):
        raise RuntimeError("odrive_nodes must be a list of node configurations")
    return nodes


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory("kanga_drive")
    config_path = os.path.join(package_share, "config", "odrive_node_ids.yaml")
    node_configs = _load_config(config_path)

    launch_actions = []
    for index, node_cfg in enumerate(node_configs):
        namespace = node_cfg.get("namespace", f"odrive_axis{index}")
        node_id = node_cfg.get("node_id")
        interface = node_cfg.get("interface", "can1")

        if node_id is None:
            raise RuntimeError(f"Missing node_id for odrive configuration at index {index}")

        launch_actions.append(
            Node(
                package="kanga_odrive",
                executable="odrive_can_node",
                name="can_node",
                namespace=namespace,
                parameters=[
                    {"node_id": int(node_id)},
                    {"interface": interface},
                ],
                output="screen",
            )
        )

    return LaunchDescription(launch_actions)
