#!/usr/bin/env python3
"""
Odometry-based autonomous movement node.

Uses odometry feedback (typically from ZED2i or other odometry source) to move
the robot precise distances in meters, rather than time-based open-loop control.

Publishes geometry_msgs/Twist to /cmd_vel and monitors odometry to track progress.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math


# -----------------------------------------------------------------------------
# EDIT THIS: movement sequence (distance-based)
# -----------------------------------------------------------------------------
# Use cmd(d, x=0, y=0, z=0) = (distance_meters, vx, vy, angular_radians).
# For rotation: z is target angle in radians (positive = CCW).
# Optional helpers: FWD(d, v), BACK(d, v), STOP(t), ROT_L(rad, v), ROT_R(rad, v),
#                   STRAFE_L(d, v), STRAFE_R(d, v). Default speed v=10 (linear), 1.0 (angular).
# -----------------------------------------------------------------------------

def cmd(distance, x=0.0, y=0.0, z=0.0):
    """
    One step: distance=target distance (m or rad), 
    x=forward/back vel, y=strafe vel, z=rotate vel (rad/s).
    For translation: distance is in meters.
    For rotation: distance is target angle in radians, z is angular velocity.
    """
    return {"distance": distance, "vel_x": x, "vel_y": y, "vel_z": z}


# Shortcuts (distance/angle, optional velocity)
FWD = lambda d, v=10.0: cmd(d, v, 0, 0)
BACK = lambda d, v=10.0: cmd(d, -v, 0, 0)
STOP = lambda t: cmd(0, 0, 0, 0)  # Stop for t seconds (time-based)
ROT_L = lambda rad, v=1.0: cmd(rad, 0, 0, v)  # Rotate left (CCW) by rad radians
ROT_R = lambda rad, v=1.0: cmd(rad, 0, 0, -v)  # Rotate right (CW) by rad radians
STRAFE_L = lambda d, v=10.0: cmd(d, 0, -v, 0)
STRAFE_R = lambda d, v=10.0: cmd(d, 0, v, 0)

MOVEMENT_SEQUENCE = [
    FWD(2.0),           # Move forward 2 meters
    STOP(1.0),          # Stop for 1 second
    ROT_L(math.pi/2),   # Rotate left 90 degrees
    STOP(1.0),
    FWD(1.5),           # Move forward 1.5 meters
    STOP(1.0),
    ROT_R(math.pi/2),   # Rotate right 90 degrees
    STOP(1.0),
    BACK(1.0),          # Move backward 1 meter
]

# Run the sequence once and then stop (set True to loop forever)
LOOP_SEQUENCE = False

# Rate at which we publish Twist (Hz)
PUBLISH_RATE_HZ = 50

# Odometry topic to subscribe to
ODOM_TOPIC = "/zed/zed_node/odom"  # Change if your odometry topic is different

# Distance tolerance (meters) - how close to target before moving to next step
DISTANCE_TOLERANCE = 0.05  # 5cm

# Angle tolerance (radians) - how close to target angle before moving to next step
ANGLE_TOLERANCE = 0.05  # ~3 degrees


def normalize_angle(angle):
    """Normalize angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class OdometryMovementNode(Node):
    def __init__(self):
        super().__init__("odometry_movement")
        
        # Publishers and subscribers
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._odom_sub = self.create_subscription(
            Odometry, ODOM_TOPIC, self._odom_callback, 10
        )
        
        # Movement sequence state
        self._sequence = list(MOVEMENT_SEQUENCE)
        self._loop = LOOP_SEQUENCE
        self._step_index = 0
        
        # Odometry state
        self._current_x = 0.0
        self._current_y = 0.0
        self._current_yaw = 0.0
        self._odom_received = False
        
        # Target tracking
        self._start_x = 0.0
        self._start_y = 0.0
        self._start_yaw = 0.0
        self._target_distance = 0.0
        self._is_rotation = False
        self._step_complete = False
        
        # Time-based stop handling
        self._stop_start_time = None
        self._stop_duration = 0.0
        
        # Control loop
        self._period = 1.0 / PUBLISH_RATE_HZ
        self._timer = self.create_timer(self._period, self._control_loop)
        
        self.get_logger().info(
            f"Odometry movement started: {len(self._sequence)} steps, "
            f"loop={self._loop}, rate={PUBLISH_RATE_HZ} Hz"
        )
        self.get_logger().info(f"Waiting for odometry on {ODOM_TOPIC}...")

    def _odom_callback(self, msg: Odometry):
        """Update current position and orientation from odometry."""
        self._current_x = msg.pose.pose.position.x
        self._current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw
        quat = msg.pose.pose.orientation
        siny_cosp = 2.0 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        self._current_yaw = math.atan2(siny_cosp, cosy_cosp)
        
        if not self._odom_received:
            self._odom_received = True
            self._start_x = self._current_x
            self._start_y = self._current_y
            self._start_yaw = self._current_yaw
            self.get_logger().info("Odometry received! Starting movement sequence.")

    def _control_loop(self):
        """Main control loop called at PUBLISH_RATE_HZ."""
        if not self._odom_received:
            # Waiting for first odometry message
            return
        
        if not self._sequence:
            self._publish_stop()
            return
        
        # Get current step
        step = self._sequence[self._step_index]
        
        # Handle STOP command (time-based)
        if step["distance"] == 0 and step["vel_x"] == 0 and step["vel_y"] == 0 and step["vel_z"] == 0:
            if self._stop_start_time is None:
                self._stop_start_time = self.get_clock().now()
                self._stop_duration = 1.0  # Default 1 second stop
                self._publish_stop()
                self.get_logger().info(f"Step {self._step_index}: STOP for {self._stop_duration}s")
            else:
                elapsed = (self.get_clock().now() - self._stop_start_time).nanoseconds / 1e9
                if elapsed >= self._stop_duration:
                    self._stop_start_time = None
                    self._advance_step()
            return
        
        # Initialize step if needed
        if not self._step_complete:
            if self._start_x is None:
                self._start_x = self._current_x
                self._start_y = self._current_y
                self._start_yaw = self._current_yaw
                self._target_distance = step["distance"]
                
                # Determine if this is a rotation or translation
                self._is_rotation = (step["vel_z"] != 0 and step["vel_x"] == 0 and step["vel_y"] == 0)
                
                if self._is_rotation:
                    self.get_logger().info(
                        f"Step {self._step_index}: Rotate {math.degrees(self._target_distance):.1f}° "
                        f"at {step['vel_z']:.2f} rad/s"
                    )
                else:
                    self.get_logger().info(
                        f"Step {self._step_index}: Move {self._target_distance:.2f}m "
                        f"at vel=({step['vel_x']:.1f}, {step['vel_y']:.1f}, {step['vel_z']:.1f})"
                    )
        
        # Check progress
        if self._is_rotation:
            # Rotational movement
            angle_traveled = normalize_angle(self._current_yaw - self._start_yaw)
            remaining = abs(self._target_distance - abs(angle_traveled))
            
            if remaining < ANGLE_TOLERANCE:
                self._publish_stop()
                self.get_logger().info(
                    f"Step {self._step_index} complete: Rotated {math.degrees(angle_traveled):.1f}°"
                )
                self._step_complete = True
                self._advance_step()
                return
        else:
            # Translational movement
            dx = self._current_x - self._start_x
            dy = self._current_y - self._start_y
            distance_traveled = math.sqrt(dx * dx + dy * dy)
            remaining = abs(self._target_distance - distance_traveled)
            
            if remaining < DISTANCE_TOLERANCE:
                self._publish_stop()
                self.get_logger().info(
                    f"Step {self._step_index} complete: Moved {distance_traveled:.2f}m"
                )
                self._step_complete = True
                self._advance_step()
                return
        
        # Publish velocity command
        twist = Twist()
        twist.linear.x = step["vel_x"]
        twist.linear.y = step["vel_y"]
        twist.angular.z = step["vel_z"]
        self._pub.publish(twist)

    def _advance_step(self):
        """Move to the next step in the sequence."""
        self._step_index += 1
        self._start_x = None
        self._start_y = None
        self._start_yaw = None
        self._step_complete = False
        
        if self._step_index >= len(self._sequence):
            if self._loop:
                self._step_index = 0
                self.get_logger().info("Sequence loop restart")
            else:
                self._publish_stop()
                self.get_logger().info("Sequence finished; stopping")
                self._timer.cancel()

    def _publish_stop(self):
        """Publish zero velocity to stop the robot."""
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
    node = OdometryMovementNode()
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

