#!/usr/bin/env python3
"""Drive simulated mecanum wheels from cmd_vel and publish wheel odometry."""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


CMD_VEL_TOPIC = "/cmd_vel"
WHEEL_COMMAND_TOPIC = "/wheel_velocity_controller/commands"
ODOM_TOPIC = "/odom"
ODOM_FRAME = "odom"
BASE_FRAME = "base_footprint"
WHEEL_JOINT_NAMES = ["wheel_fl_joint", "wheel_fr_joint", "wheel_rl_joint", "wheel_rr_joint"]
# Match the active mecanum runtime geometry used by rover_twist_relay, the
# Gazebo kinematic drive, and the mecanum3 collision model.
WHEEL_RADIUS = 0.07
WHEEL_SEPARATION_WIDTH = 0.36
WHEEL_SEPARATION_LENGTH = 0.263
WHEELBASE_RADIUS = WHEEL_SEPARATION_WIDTH + WHEEL_SEPARATION_LENGTH
COMMAND_TIMEOUT_SEC = 0.25
PUBLISH_PERIOD_SEC = 0.05
MAX_LINEAR_SPEED = 0.45
MAX_ANGULAR_SPEED = 0.9
MAX_LINEAR_ACCEL = 0.8
MAX_ANGULAR_ACCEL = 1.5


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))


def _clamp_vector_length(x: float, y: float, max_length: float) -> tuple[float, float]:
    length = math.hypot(x, y)
    if length <= max_length or length <= 1.0e-9:
        return x, y
    scale = max_length / length
    return x * scale, y * scale


def wheel_speeds_from_twist(
    vx: float,
    vy: float,
    wz: float,
    wheel_radius: float = WHEEL_RADIUS,
    wheelbase_radius: float = WHEELBASE_RADIUS,
) -> list[float]:
    if wheel_radius <= 0.0:
        raise ValueError("wheel_radius must be positive")

    return [
        (vx - vy - wheelbase_radius * wz) / wheel_radius,
        (vx + vy + wheelbase_radius * wz) / wheel_radius,
        (vx + vy - wheelbase_radius * wz) / wheel_radius,
        (vx - vy + wheelbase_radius * wz) / wheel_radius,
    ]


def twist_from_wheel_speeds(
    wheel_speeds: list[float] | tuple[float, float, float, float],
    wheel_radius: float = WHEEL_RADIUS,
    wheelbase_radius: float = WHEELBASE_RADIUS,
) -> tuple[float, float, float]:
    if wheel_radius <= 0.0:
        raise ValueError("wheel_radius must be positive")
    if wheelbase_radius <= 0.0:
        raise ValueError("wheelbase_radius must be positive")
    if len(wheel_speeds) != 4:
        raise ValueError("four wheel speeds are required")

    fl, fr, rl, rr = [float(speed) for speed in wheel_speeds]
    vx = wheel_radius * (fl + fr + rl + rr) / 4.0
    vy = wheel_radius * (-fl + fr + rl - rr) / 4.0
    wz = wheel_radius * (-fl + fr - rl + rr) / (4.0 * wheelbase_radius)
    return vx, vy, wz


def clamp_twist_for_pose_model(
    twist: tuple[float, float, float],
    max_linear_speed: float = MAX_LINEAR_SPEED,
    max_angular_speed: float = MAX_ANGULAR_SPEED,
) -> tuple[float, float, float]:
    vx, vy, wz = [float(value) for value in twist]
    return (
        _clamp(vx, max_linear_speed),
        _clamp(vy, max_linear_speed),
        _clamp(wz, max_angular_speed),
    )


def limit_twist_for_pose_model(
    current_twist: tuple[float, float, float],
    target_twist: tuple[float, float, float],
    dt: float,
    max_linear_accel: float = MAX_LINEAR_ACCEL,
    max_angular_accel: float = MAX_ANGULAR_ACCEL,
) -> tuple[float, float, float]:
    current_vx, current_vy, current_wz = [float(value) for value in current_twist]
    target_vx, target_vy, target_wz = [float(value) for value in target_twist]
    dt = max(0.0, float(dt))

    linear_dx, linear_dy = _clamp_vector_length(
        target_vx - current_vx,
        target_vy - current_vy,
        max(0.0, max_linear_accel) * dt,
    )
    angular_delta = _clamp(
        target_wz - current_wz,
        max(0.0, max_angular_accel) * dt,
    )
    return (
        current_vx + linear_dx,
        current_vy + linear_dy,
        current_wz + angular_delta,
    )


def shutdown_if_running(ok=rclpy.ok, shutdown=rclpy.shutdown) -> None:
    if ok():
        shutdown()


class MecanumWheelVisualizer(Node):
    def __init__(self):
        super().__init__("mecanum_wheel_visualizer")
        self.declare_parameter("cmd_vel_topic", CMD_VEL_TOPIC)
        self.declare_parameter("wheel_command_topic", WHEEL_COMMAND_TOPIC)
        self.declare_parameter("odom_topic", ODOM_TOPIC)
        self.declare_parameter("odom_frame", ODOM_FRAME)
        self.declare_parameter("base_frame", BASE_FRAME)
        self.declare_parameter("wheel_radius", WHEEL_RADIUS)
        self.declare_parameter("wheelbase_radius", WHEELBASE_RADIUS)
        self.declare_parameter("command_timeout_sec", COMMAND_TIMEOUT_SEC)
        self.declare_parameter("publish_odom", True)
        self.declare_parameter("max_linear_speed", MAX_LINEAR_SPEED)
        self.declare_parameter("max_angular_speed", MAX_ANGULAR_SPEED)
        self.declare_parameter("max_linear_accel", MAX_LINEAR_ACCEL)
        self.declare_parameter("max_angular_accel", MAX_ANGULAR_ACCEL)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._wheelbase_radius = float(self.get_parameter("wheelbase_radius").value)
        self._command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        self._publish_odom_enabled = bool(self.get_parameter("publish_odom").value)
        self._max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self._max_angular_speed = float(self.get_parameter("max_angular_speed").value)
        self._max_linear_accel = float(self.get_parameter("max_linear_accel").value)
        self._max_angular_accel = float(self.get_parameter("max_angular_accel").value)
        command_topic = str(self.get_parameter("wheel_command_topic").value)
        odom_topic = str(self.get_parameter("odom_topic").value)

        self._last_command_time = 0.0
        self._last_odom_time = None
        self._last_speeds = [0.0, 0.0, 0.0, 0.0]
        self._current_twist = (0.0, 0.0, 0.0)
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self._odom_pub = (
            self.create_publisher(Odometry, odom_topic, 10)
            if self._publish_odom_enabled
            else None
        )
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)
        self.get_logger().info(
            "mecanum wheel driver: "
            f"{self._cmd_vel_topic} -> {command_topic}, "
            f"commanded wheel speeds -> {odom_topic}"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_command_time = time.monotonic()
        self._last_speeds = wheel_speeds_from_twist(
            msg.linear.x,
            msg.linear.y,
            msg.angular.z,
            wheel_radius=self._wheel_radius,
            wheelbase_radius=self._wheelbase_radius,
        )
        self._publish(self._last_speeds)

    def _on_timer(self) -> None:
        if self._last_command_time == 0.0:
            speeds = [0.0, 0.0, 0.0, 0.0]
            self._publish(speeds)
            self._integrate_wheel_speeds(speeds)
            return

        if time.monotonic() - self._last_command_time > self._command_timeout_sec:
            self._last_speeds = [0.0, 0.0, 0.0, 0.0]

        self._publish(self._last_speeds)
        self._integrate_wheel_speeds(self._last_speeds)

    def _publish(self, speeds: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = speeds
        self._pub.publish(msg)

    def _integrate_wheel_speeds(self, speeds: list[float]) -> None:
        now = self._clock_seconds()
        if self._last_odom_time is None:
            self._last_odom_time = now
            return

        dt = max(0.0, min(now - self._last_odom_time, 0.2))
        self._last_odom_time = now
        target_twist = clamp_twist_for_pose_model(
            twist_from_wheel_speeds(
                speeds,
                wheel_radius=self._wheel_radius,
                wheelbase_radius=self._wheelbase_radius,
            ),
            max_linear_speed=self._max_linear_speed,
            max_angular_speed=self._max_angular_speed,
        )
        self._current_twist = limit_twist_for_pose_model(
            self._current_twist,
            target_twist,
            dt,
            max_linear_accel=self._max_linear_accel,
            max_angular_accel=self._max_angular_accel,
        )
        vx, vy, wz = self._current_twist
        cos_yaw = math.cos(self._yaw)
        sin_yaw = math.sin(self._yaw)
        self._x += (vx * cos_yaw - vy * sin_yaw) * dt
        self._y += (vx * sin_yaw + vy * cos_yaw) * dt
        self._yaw = math.atan2(math.sin(self._yaw + wz * dt), math.cos(self._yaw + wz * dt))
        self._publish_odom(vx, vy, wz)

    def _clock_seconds(self) -> float:
        stamp = self.get_clock().now().to_msg()
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _publish_odom(self, vx: float, vy: float, wz: float) -> None:
        if not self._publish_odom_enabled or self._odom_pub is None:
            return
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._odom_frame
        msg.child_frame_id = self._base_frame
        msg.pose.pose.position.x = self._x
        msg.pose.pose.position.y = self._y
        msg.pose.pose.orientation.z = math.sin(self._yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(self._yaw / 2.0)
        msg.twist.twist.linear.x = vx
        msg.twist.twist.linear.y = vy
        msg.twist.twist.angular.z = wz
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[7] = 0.02
        msg.twist.covariance[35] = 0.05
        self._odom_pub.publish(msg)


def main():
    rclpy.init()
    node = MecanumWheelVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    shutdown_if_running()


if __name__ == "__main__":
    main()
