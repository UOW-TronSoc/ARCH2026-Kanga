from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="kanga_battery",
            executable="daly_can_node",
            name="can_node",
            namespace="battery",
            parameters=[
                {"local_node_id": 320},
                {"daly_node_id": 16385},
                {"interface": "can1"},
                {"req_period": 1},
            ],
            output="screen",
        )
    ])
