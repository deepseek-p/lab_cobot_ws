#!/usr/bin/env python3
"""Drive simulated mecanum wheel visuals from base cmd_vel."""
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray


CMD_VEL_TOPIC = "/cmd_vel"
WHEEL_COMMAND_TOPIC = "/wheel_velocity_controller/commands"
BASE_LENGTH = 0.55
BASE_WIDTH = 0.50
WHEEL_RADIUS = 0.08
WHEEL_WIDTH = 0.05
WHEEL_LONGITUDINAL_OFFSET = BASE_LENGTH / 2.0 - WHEEL_RADIUS
WHEEL_LATERAL_OFFSET = BASE_WIDTH / 2.0 + WHEEL_WIDTH / 2.0
WHEELBASE_RADIUS = WHEEL_LONGITUDINAL_OFFSET + WHEEL_LATERAL_OFFSET
COMMAND_TIMEOUT_SEC = 0.25
PUBLISH_PERIOD_SEC = 0.05


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


def shutdown_if_running(ok=rclpy.ok, shutdown=rclpy.shutdown) -> None:
    if ok():
        shutdown()


class MecanumWheelVisualizer(Node):
    def __init__(self):
        super().__init__("mecanum_wheel_visualizer")
        self.declare_parameter("cmd_vel_topic", CMD_VEL_TOPIC)
        self.declare_parameter("wheel_command_topic", WHEEL_COMMAND_TOPIC)
        self.declare_parameter("wheel_radius", WHEEL_RADIUS)
        self.declare_parameter("wheelbase_radius", WHEELBASE_RADIUS)
        self.declare_parameter("command_timeout_sec", COMMAND_TIMEOUT_SEC)

        self._cmd_vel_topic = str(self.get_parameter("cmd_vel_topic").value)
        self._wheel_radius = float(self.get_parameter("wheel_radius").value)
        self._wheelbase_radius = float(self.get_parameter("wheelbase_radius").value)
        self._command_timeout_sec = float(
            self.get_parameter("command_timeout_sec").value
        )
        command_topic = str(self.get_parameter("wheel_command_topic").value)

        self._last_command_time = 0.0
        self._last_speeds = [0.0, 0.0, 0.0, 0.0]
        self._pub = self.create_publisher(Float64MultiArray, command_topic, 10)
        self.create_subscription(Twist, self._cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_timer(PUBLISH_PERIOD_SEC, self._on_timer)
        self.get_logger().info(
            f"mecanum wheel visualizer: {self._cmd_vel_topic} -> {command_topic}"
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
            self._publish([0.0, 0.0, 0.0, 0.0])
            return

        if time.monotonic() - self._last_command_time > self._command_timeout_sec:
            self._last_speeds = [0.0, 0.0, 0.0, 0.0]

        self._publish(self._last_speeds)

    def _publish(self, speeds: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = speeds
        self._pub.publish(msg)


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
