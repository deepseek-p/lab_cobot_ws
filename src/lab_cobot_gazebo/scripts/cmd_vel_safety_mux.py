#!/usr/bin/env python3
"""Priority mux for final base velocity commands."""
import copy

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


NAV_TIMEOUT_SEC = 0.5
SAFETY_TIMEOUT_SEC = 0.5
MANUAL_TIMEOUT_SEC = 0.5
PUBLISH_HZ = 30.0


class CmdVelSafetyMux(Node):
    """Arbitrate nav / manual / safety on the single final cmd_vel."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_safety_mux")
        self._nav_cmd = Twist()
        self._safety_cmd = Twist()
        self._manual_cmd = Twist()
        self._last_nav_time = None
        self._last_safety_time = None
        self._last_manual_time = None
        self._safety_active = False
        self._manual_active = False

        self.create_subscription(Twist, "/cmd_vel_nav_smoothed", self._nav_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_safety", self._safety_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_manual", self._manual_cb, 10)
        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_timer(1.0 / PUBLISH_HZ, self._tick)
        self.get_logger().info(
            "cmd_vel safety mux started: "
            "nav <= manual <= safety -> /cmd_vel"
        )

    @property
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _nav_cb(self, msg: Twist) -> None:
        self._nav_cmd = copy.deepcopy(msg)
        self._last_nav_time = self._now

    def _safety_cb(self, msg: Twist) -> None:
        self._safety_cmd = copy.deepcopy(msg)
        self._last_safety_time = self._now

    def _manual_cb(self, msg: Twist) -> None:
        self._manual_cmd = copy.deepcopy(msg)
        self._last_manual_time = self._now

    def _fresh(self, stamp: float, timeout: float) -> bool:
        return stamp is not None and self._now - stamp <= timeout

    def _nonzero(self, cmd: Twist) -> bool:
        return (abs(cmd.linear.x) > 1e-4 or abs(cmd.linear.y) > 1e-4
                or abs(cmd.linear.z) > 1e-4 or abs(cmd.angular.z) > 1e-4)

    def _tick(self) -> None:
        # A zero safety message means "release override"; it must not mask
        # lower-priority commands. Only non-zero safety commands own the
        # final base velocity.
        if (self._fresh(self._last_safety_time, SAFETY_TIMEOUT_SEC)
                and self._nonzero(self._safety_cmd)):
            if not self._safety_active:
                self.get_logger().info("safety velocity override ON")
                self._safety_active = True
            self._pub.publish(self._safety_cmd)
            return

        if self._safety_active:
            self.get_logger().info("safety velocity override OFF")
            self._safety_active = False

        # Mission-level low-level motion (retreat/dock/stop) is routed here
        # so it can never race the safety layer on /cmd_vel. A fresh zero
        # manual command intentionally holds the robot still until release.
        if self._fresh(self._last_manual_time, MANUAL_TIMEOUT_SEC):
            if not self._manual_active:
                self.get_logger().info("manual velocity hold ON")
                self._manual_active = True
            self._pub.publish(self._manual_cmd)
            return
        if self._manual_active:
            self.get_logger().info("manual velocity hold OFF")
            self._manual_active = False

        if self._fresh(self._last_nav_time, NAV_TIMEOUT_SEC):
            self._pub.publish(self._nav_cmd)
        else:
            self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyMux()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
