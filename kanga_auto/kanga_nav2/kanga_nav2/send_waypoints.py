#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def make_pose(navigator: BasicNavigator, x: float, y: float, yaw_w: float = 1.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = yaw_w
    return pose


def main() -> None:
    rclpy.init()
    navigator = BasicNavigator()

    # No localization stack: map->odom is fixed, so current Gazebo pose is directly usable.
    navigator.waitUntilNav2Active()

    waypoints = [
        make_pose(navigator, 5.0, 5.0),
        make_pose(navigator, 15.0, 5.0),
        make_pose(navigator, 15.0, 15.0),
        make_pose(navigator, 5.0, 15.0),
        make_pose(navigator, 0.0, 0.0),
    ]

    navigator.goThroughPoses(waypoints)

    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            navigator.get_logger().info('Following waypoint route...')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Waypoint route completed')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().error('Waypoint route canceled')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Waypoint route failed')
    else:
        navigator.get_logger().error('Waypoint route ended with unknown result')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
