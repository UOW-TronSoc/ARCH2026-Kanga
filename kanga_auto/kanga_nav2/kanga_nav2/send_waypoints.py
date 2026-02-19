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
    # No AMCL in this simulation setup; use map_server lifecycle as readiness gate.
    navigator.waitUntilNav2Active(localizer='map_server')

    waypoints = [

        make_pose(navigator, 5.0, 5.0),
        make_pose(navigator, 20.0, 5.0),
        make_pose(navigator, 20.0, 15.0, 0.0),
        make_pose(navigator, 5.0, 15.0),
        # make_pose(navigator, 0.0, 0.0),
    ]

    accepted = navigator.goThroughPoses(waypoints)
    if not accepted:
        navigator.get_logger().error('Waypoint route was rejected by the action server')
        rclpy.shutdown()
        return

    last_feedback = None
    while not navigator.isTaskComplete():
        feedback = navigator.getFeedback()
        if feedback is not None:
            last_feedback = feedback
            navigator.get_logger().info('Following waypoint route...')

    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        navigator.get_logger().info('Waypoint route completed')
    elif result == TaskResult.CANCELED:
        navigator.get_logger().error('Waypoint route canceled')
        navigator.get_logger().error(f'Action status code: {navigator.status}')
    elif result == TaskResult.FAILED:
        navigator.get_logger().error('Waypoint route failed')
        navigator.get_logger().error(f'Action status code: {navigator.status}')
    else:
        navigator.get_logger().error('Waypoint route ended with unknown result')

    if result != TaskResult.SUCCEEDED and last_feedback is not None:
        navigator.get_logger().error(
            'Last feedback: remaining poses=%d, distance_remaining=%.3f, '
            'navigation_time=%d.%09d, estimated_time_remaining=%d.%09d, '
            'recoveries=%d',
            last_feedback.number_of_poses_remaining,
            last_feedback.distance_remaining,
            last_feedback.navigation_time.sec,
            last_feedback.navigation_time.nanosec,
            last_feedback.estimated_time_remaining.sec,
            last_feedback.estimated_time_remaining.nanosec,
            last_feedback.number_of_recoveries,
        )

    rclpy.shutdown()


if __name__ == '__main__':
    main()
