#!/usr/bin/env python3

import argparse
import math
import os
import time
from dataclasses import dataclass
from typing import List, Union, Optional, Any

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration

from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import Spin
import yaml

from tf2_ros import Buffer, TransformListener


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    """
    Local implementation to avoid tf_transformations -> transforms3d -> numpy alias issues.
    Returns (x, y, z, w).
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return (qx, qy, qz, qw)


@dataclass
class SpinOp:
    """Spin in place by target_yaw (radians)."""
    radians: float = 2.0 * math.pi
    time_allowance_sec: float = 30.0


@dataclass
class FaceYawOp:
    """
    Rotate to an absolute yaw in the map frame.
    This is Pattern C: reach XY first, then optionally enforce yaw.
    """
    target_yaw_rad: float
    time_allowance_sec: float = 30.0


Operation = Union[PoseStamped, SpinOp, FaceYawOp]


def make_xy_pose(navigator: BasicNavigator, x: float, y: float) -> PoseStamped:
    """
    XY only goal. Orientation is identity so yaw does not matter for goal checking.
    Pattern C expects yaw to be handled with FaceYawOp when needed.
    """
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.w = 1.0
    return pose


def make_pose_with_yaw(navigator: BasicNavigator, x: float, y: float, yaw_rad: float) -> PoseStamped:
    """
    Use this only if you explicitly want navigation to enforce yaw.
    In your case this was the source of "never reaches goal" failures.
    Prefer make_xy_pose + FaceYawOp.
    """
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rclpy.time.Time().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)

    qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, float(yaw_rad))
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    return pose


def spin_op(radians: float = 2.0 * math.pi, time_allowance_sec: float = 30.0) -> SpinOp:
    return SpinOp(radians=radians, time_allowance_sec=time_allowance_sec)


def face_yaw_op(target_yaw_rad: float, time_allowance_sec: float = 30.0) -> FaceYawOp:
    return FaceYawOp(target_yaw_rad=float(target_yaw_rad), time_allowance_sec=float(time_allowance_sec))


def _to_float(value: Any, name: str, op_index: int) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as e:
        raise ValueError(f"Operation {op_index}: '{name}' must be numeric, got {value!r}") from e


def load_operations_from_yaml(config_path: str, navigator: BasicNavigator) -> List[Operation]:
    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    if isinstance(data, dict):
        raw_ops = data.get("operations")
    else:
        raw_ops = data

    if not isinstance(raw_ops, list):
        raise ValueError("Config must contain an 'operations' list")

    operations: List[Operation] = []
    for idx, item in enumerate(raw_ops):
        if not isinstance(item, dict):
            raise ValueError(f"Operation {idx}: expected a mapping, got {type(item).__name__}")

        op_type = item.get("type")
        if op_type == "goto":
            x = _to_float(item.get("x"), "x", idx)
            y = _to_float(item.get("y"), "y", idx)
            operations.append(make_xy_pose(navigator, x, y))
            continue

        if op_type == "spin":
            if "degrees" in item:
                radians = math.radians(_to_float(item.get("degrees"), "degrees", idx))
            elif "radians" in item:
                # Backward-compatible fallback.
                radians = _to_float(item.get("radians"), "radians", idx)
            else:
                radians = 2.0 * math.pi
            time_allowance_sec = _to_float(item.get("time_allowance_sec", 30.0), "time_allowance_sec", idx)
            operations.append(spin_op(radians=radians, time_allowance_sec=time_allowance_sec))
            continue

        if op_type == "face_yaw":
            if "target_yaw_deg" in item:
                target_yaw_rad = math.radians(_to_float(item.get("target_yaw_deg"), "target_yaw_deg", idx))
            elif "target_yaw_rad" in item:
                # Backward-compatible fallback.
                target_yaw_rad = _to_float(item.get("target_yaw_rad"), "target_yaw_rad", idx)
            else:
                raise ValueError(f"Operation {idx}: missing 'target_yaw_deg'")
            time_allowance_sec = _to_float(item.get("time_allowance_sec", 30.0), "time_allowance_sec", idx)
            operations.append(face_yaw_op(target_yaw_rad=target_yaw_rad, time_allowance_sec=time_allowance_sec))
            continue

        raise ValueError(
            f"Operation {idx}: unsupported type '{op_type}'. Supported: goto, spin, face_yaw"
        )

    return operations


def default_operations_config_path() -> str:
    return os.path.join(get_package_share_directory("kanga_nav2"), "config", "test_ops.yaml")


def wrap_to_pi(x: float) -> float:
    return (x + math.pi) % (2.0 * math.pi) - math.pi


def do_spin(node: rclpy.node.Node, spin_client: ActionClient, radians: float, time_allowance_sec: float) -> bool:
    if not spin_client.wait_for_server(timeout_sec=5.0):
        node.get_logger().error("Spin action server 'spin' not available")
        return False

    goal = Spin.Goal()
    goal.target_yaw = float(radians)
    goal.time_allowance = Duration(seconds=float(time_allowance_sec)).to_msg()

    send_future = spin_client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)

    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        node.get_logger().error("Spin goal rejected")
        return False

    result_future = goal_handle.get_result_async()
    t0 = time.time()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
        if result_future.done():
            break
        if time.time() - t0 > time_allowance_sec + 2.0:
            node.get_logger().error("Spin timed out, canceling")
            cancel_future = goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(node, cancel_future)
            return False

    res = result_future.result()
    if res is None:
        node.get_logger().error("Spin result missing")
        return False

    # rclpy action result status: 4 = SUCCEEDED
    if res.status == 4:
        node.get_logger().info("Spin completed")
        return True

    node.get_logger().error(f"Spin failed with status={res.status}")
    return False


def do_face_yaw(
    node: rclpy.node.Node,
    tf_buffer: Buffer,
    spin_client: ActionClient,
    target_yaw_rad: float,
    time_allowance_sec: float,
) -> bool:
    """
    Rotate to an absolute yaw in map frame by computing delta yaw from current map->base_link.
    Requires valid TF for map -> base_link.
    """
    try:
        max_retries = 10
        for attempt in range(max_retries):
            timeout = Duration(seconds=2.0)
            if tf_buffer.can_transform("map", "base_link", rclpy.time.Time(), timeout=timeout):
                break
            node.get_logger().warn(
                f"TF map->base_link not available (attempt {attempt + 1}/{max_retries}), retrying..."
            )
            rclpy.spin_once(node, timeout_sec=0.5)
        else:
            node.get_logger().error(
                f"TF map->base_link not available after {max_retries} attempts"
            )
            return False
        tf = tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
        q = tf.transform.rotation
        # quaternion -> yaw
        # yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        cur_yaw = math.atan2(siny_cosp, cosy_cosp)
    except Exception as e:
        node.get_logger().error(f"Failed to read TF map->base_link for FaceYawOp: {e}")
        return False

    delta = wrap_to_pi(float(target_yaw_rad) - float(cur_yaw))
    node.get_logger().info(f"FaceYawOp current={cur_yaw:.3f} target={target_yaw_rad:.3f} delta={delta:.3f}")
    return do_spin(node, spin_client, delta, time_allowance_sec)


def do_goto(navigator: BasicNavigator, pose: PoseStamped, timeout_sec: float = 180.0) -> TaskResult:
    navigator.goToPose(pose)

    t0 = time.time()
    while not navigator.isTaskComplete():
        if time.time() - t0 > timeout_sec:
            navigator.cancelTask()
            return TaskResult.CANCELED
        time.sleep(0.1)

    return navigator.getResult()


def main() -> None:
    parser = argparse.ArgumentParser(description="Send waypoint/spin operations to Nav2")
    parser.add_argument(
        "--ops-file",
        default=default_operations_config_path(),
        help="Path to YAML operations config (default: package config/waypoint_ops.yaml)",
    )
    args = parser.parse_args()

    rclpy.init()

    navigator = BasicNavigator()
    node = navigator  # BasicNavigator is a Node
    spin_client = ActionClient(node, Spin, "spin")
    tf_buffer = Buffer()
    tf_listener = TransformListener(tf_buffer, node)

    navigator.waitUntilNav2Active(localizer="map_server")

    try:
        operations = load_operations_from_yaml(args.ops_file, navigator)
    except Exception as e:
        node.get_logger().error(f"Failed to load operations config '{args.ops_file}': {e}")
        rclpy.shutdown()
        return
    node.get_logger().info(f"Loaded {len(operations)} operations from {args.ops_file}")

    for i, op in enumerate(operations):
        if isinstance(op, PoseStamped):
            node.get_logger().info(f"[{i}] Navigate to ({op.pose.position.x:.2f}, {op.pose.position.y:.2f})")
            result = do_goto(navigator, op, timeout_sec=180.0)
            if result == TaskResult.SUCCEEDED:
                node.get_logger().info("Reached pose")
            else:
                node.get_logger().error(f"Navigation failed (result={result}), continuing to next operation")
                continue

        elif isinstance(op, SpinOp):
            node.get_logger().info(f"[{i}] Spin {op.radians:.3f} rad")
            ok = do_spin(node, spin_client, op.radians, op.time_allowance_sec)
            if not ok:
                node.get_logger().error("Spin failed, continuing to next operation")
                continue

        elif isinstance(op, FaceYawOp):
            node.get_logger().info(f"[{i}] Face yaw {op.target_yaw_rad:.3f} rad")
            ok = do_face_yaw(node, tf_buffer, spin_client, op.target_yaw_rad, op.time_allowance_sec)
            if not ok:
                node.get_logger().error("FaceYaw failed, continuing to next operation")
                continue

        else:
            node.get_logger().error(f"Unknown operation type: {type(op)}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
