#!/usr/bin/env python3
"""Bridge simple obstacle-box commands into MoveIt PlanningScene."""

import rclpy
import time
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

from lab_cobot_manipulation.scene_obstacles import (
    DYNAMIC_ARM_OBSTACLE_BOX_ID,
    PlanningSceneClient,
    make_dynamic_obstacle_scene,
    make_remove_dynamic_obstacle_scene,
)


DEFAULT_TOPIC = "/arm_dynamic_obstacle_box"
DEFAULT_FRAME_ID = "base_link"
DEFAULT_STATUS_TOPIC = "/g5/arm_dynamic_obstacle/status"
DEFAULT_MIN_UPDATE_PERIOD_SEC = 0.25


def parse_obstacle_box(data):
    """Return (center, size) from [cx, cy, cz, sx, sy, sz]."""
    values = [float(value) for value in data]
    if len(values) != 6:
        raise ValueError("dynamic obstacle command must contain 6 values")
    center = values[:3]
    size = values[3:]
    if any(value <= 0.0 for value in size):
        raise ValueError("dynamic obstacle size values must be positive")
    return center, size


class DynamicArmObstacleNode(Node):
    """Subscribe to a simple box command topic and update arm PlanningScene."""

    def __init__(self):
        super().__init__("dynamic_arm_obstacle_node")
        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("status_topic", DEFAULT_STATUS_TOPIC)
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("object_id", DYNAMIC_ARM_OBSTACLE_BOX_ID)
        self.declare_parameter("service_timeout_sec", 5.0)
        self.declare_parameter(
            "min_update_period_sec",
            DEFAULT_MIN_UPDATE_PERIOD_SEC,
        )

        self.topic = str(self.get_parameter("topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.object_id = str(self.get_parameter("object_id").value)
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )
        self.min_update_period_sec = float(
            self.get_parameter("min_update_period_sec").value
        )

        self._callback_group = ReentrantCallbackGroup()
        self.scene_client = PlanningSceneClient(
            self,
            callback_group=self._callback_group,
        )
        self.scene_client.wait_until_ready(timeout_sec=self.service_timeout_sec)
        self._status_pub = self.create_publisher(String, self.status_topic, 10)
        self._last_scene_signature = None
        self._last_scene_update_time = 0.0
        self.create_subscription(
            Float32MultiArray,
            self.topic,
            self._on_obstacle_box,
            10,
            callback_group=self._callback_group,
        )
        self.get_logger().info(
            "dynamic arm obstacle bridge listening topic=%s frame=%s id=%s"
            % (self.topic, self.frame_id, self.object_id)
        )
        self._publish_status("ready topic=%s frame=%s id=%s" % (
            self.topic,
            self.frame_id,
            self.object_id,
        ))

    def _publish_status(self, text: str) -> None:
        publisher = getattr(self, "_status_pub", None)
        if publisher is None:
            return
        msg = String()
        msg.data = str(text)
        publisher.publish(msg)

    def _apply_scene(self, scene, label) -> bool:
        ok = self.scene_client.apply(
            scene,
            timeout_sec=self.service_timeout_sec,
        )
        if not ok:
            self.get_logger().warn(f"dynamic obstacle {label} apply failed")
            self._publish_status(f"{label}_failed id={self.object_id}")
        return ok

    def _scene_signature(self, center, size, removed: bool) -> tuple:
        return (
            bool(removed),
            self.frame_id,
            self.object_id,
            tuple(round(float(value), 4) for value in center),
            tuple(round(float(value), 4) for value in size),
        )

    def _on_obstacle_box(self, msg) -> None:
        if len(msg.data) == 0:
            signature = self._scene_signature((), (), True)
            now = time.monotonic()
            if (
                getattr(self, "_last_scene_signature", None) == signature
                and now - float(getattr(self, "_last_scene_update_time", 0.0))
                < float(getattr(self, "min_update_period_sec", 0.0))
            ):
                return
            scene = make_remove_dynamic_obstacle_scene(self.object_id)
            if self._apply_scene(scene, "remove"):
                self._last_scene_signature = signature
                self._last_scene_update_time = now
                self.get_logger().info(
                    "dynamic arm obstacle removed id=%s" % self.object_id
                )
                self._publish_status("removed id=%s" % self.object_id)
            return
        try:
            center, size = parse_obstacle_box(msg.data)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            self._publish_status("invalid %s" % str(exc))
            return

        signature = self._scene_signature(center, size, False)
        now = time.monotonic()
        if (
            getattr(self, "_last_scene_signature", None) == signature
            and now - float(getattr(self, "_last_scene_update_time", 0.0))
            < float(getattr(self, "min_update_period_sec", 0.0))
        ):
            return
        scene = make_dynamic_obstacle_scene(
            center,
            size,
            frame_id=self.frame_id,
            object_id=self.object_id,
        )
        if self._apply_scene(scene, "update"):
            self._last_scene_signature = signature
            self._last_scene_update_time = now
            self.get_logger().info(
                "dynamic arm obstacle updated center=%s size=%s"
                % (center, size)
            )
            self._publish_status(
                "updated center=%s size=%s" % (center, size)
            )


def main() -> int:
    rclpy.init()
    node = DynamicArmObstacleNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
