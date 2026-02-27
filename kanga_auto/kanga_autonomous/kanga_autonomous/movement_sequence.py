#!/usr/bin/env python3
"""
Autonomous movement sequence node.

Publishes geometry_msgs/Twist to /cmd_vel for the duration of each step.
The wheel_command_mapper (kanga_drive) subscribes to /cmd_vel and forwards
velocity commands to the ODrive axes.

Edit MOVEMENT_SEQUENCE below to change the autonomous behaviour. No recompile
needed — edit and re-run the node (or use a launch file that restarts it).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# -----------------------------------------------------------------------------
# EDIT THIS: movement sequence (dev-friendly)
# -----------------------------------------------------------------------------
# Use cmd(t, x=0, y=0, z=0) = (duration_sec, linear_x, linear_y, angular_z).
# Optional helpers: FWD(t, v), BACK(t, v), STOP(t), ROT_L(t, v), ROT_R(t, v),
#                   STRAFE_L(t, v), STRAFE_R(t, v). Default speed v=5 (linear), 3 (angular).
# -----------------------------------------------------------------------------

def cmd(t, x=0.0, y=0.0, z=0.0):
    """One step: t=duration (s), x=forward/back, y=strafe, z=rotate (CCW+)."""
    return {"duration": t, "linear_x": x, "linear_y": y, "angular_z": z}


# Shortcuts (duration_sec, optional speed)
FWD = lambda t, v=22.0: cmd(t, v, 0, 0)
BACK = lambda t, v=22.0: cmd(t, -v, 0, 0)
STOP = lambda t: cmd(t, 0, 0, 0)
ROT_L = lambda t, v=22.0: cmd(t, 0, 0, v)
ROT_R = lambda t, v=22.0: cmd(t, 0, 0, -v)
STRAFE_L = lambda t, v=22.0: cmd(t, 0, -v, 0)
STRAFE_R = lambda t, v=22.0: cmd(t, 0, v, 0)

MOVEMENT_SEQUENCE = [
    FWD(8.25),      #  forward
    STOP(1.0),     # 1s stop
    ROT_L(4.6),
    STOP(1.0),     # 2s backward
    FWD(12.3),
    STOP(1.0),
    ROT_L(13.75),    # 2s rotate left (CCW)
    STOP(1.0),
    FWD(24.0),    # 2s rotate right (CW)
    STOP(1.0),
    ROT_R(4.4),
    STOP(1.0),
    FWD(8.2),
    STOP(1.0),
]

# Run the sequence once and then stop (set True to loop forever)
LOOP_SEQUENCE = False

# Rate at which we publish Twist (Hz). Should be high enough for smooth control.
PUBLISH_RATE_HZ = 50


def _normalize_step(step):
    """Accept dict or (duration, vx, vy, wz) tuple."""
    if isinstance(step, (list, tuple)) and len(step) >= 4:
        return {"duration": step[0], "linear_x": step[1], "linear_y": step[2], "angular_z": step[3]}
    return step


def get_twist(step) -> Twist:
    step = _normalize_step(step)
    msg = Twist()
    msg.linear.x = float(step.get("linear_x", 0.0))
    msg.linear.y = float(step.get("linear_y", 0.0))
    msg.linear.z = 0.0
    msg.angular.x = 0.0
    msg.angular.y = 0.0
    msg.angular.z = float(step.get("angular_z", 0.0))
    return msg


class MovementSequenceNode(Node):
    def __init__(self):
        super().__init__("movement_sequence")
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._sequence = list(MOVEMENT_SEQUENCE)
        self._loop = LOOP_SEQUENCE
        self._step_index = 0
        self._step_elapsed = 0.0
        self._period = 1.0 / PUBLISH_RATE_HZ
        self._timer = self.create_timer(self._period, self._timer_cb)
        self.get_logger().info(
            "movement_sequence started: %d steps, loop=%s, rate=%d Hz"
            % (len(self._sequence), self._loop, PUBLISH_RATE_HZ)
        )

    def _timer_cb(self):
        if not self._sequence:
            self._publish_stop()
            return

        raw = self._sequence[self._step_index]
        step = _normalize_step(raw)
        duration = float(step.get("duration", 0.0))
        twist = get_twist(step)
        self._pub.publish(twist)

        self._step_elapsed += self._period
        if self._step_elapsed >= duration:
            self._step_elapsed = 0.0
            self._step_index += 1
            if self._step_index >= len(self._sequence):
                if self._loop:
                    self._step_index = 0
                    self.get_logger().info("sequence loop restart")
                else:
                    self._publish_stop()
                    self.get_logger().info("sequence finished; stopping")
                    self._timer.cancel()

    def _publish_stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MovementSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

