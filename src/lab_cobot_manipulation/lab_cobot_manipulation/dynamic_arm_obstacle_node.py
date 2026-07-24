#!/usr/bin/env python3
"""Bridge simple obstacle-box commands into MoveIt PlanningScene."""

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

from lab_cobot_manipulation.scene_obstacles import (
    DYNAMIC_ARM_OBSTACLE_BOX_ID,
    PlanningSceneClient,
    make_dynamic_obstacle_scene,
    make_remove_dynamic_obstacle_scene,
)


DEFAULT_TOPIC = "/arm_dynamic_obstacle_box"
DEFAULT_FRAME_ID = "base_link"


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
        self.declare_parameter("frame_id", DEFAULT_FRAME_ID)
        self.declare_parameter("object_id", DYNAMIC_ARM_OBSTACLE_BOX_ID)
        self.declare_parameter("service_timeout_sec", 5.0)

        self.topic = str(self.get_parameter("topic").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.object_id = str(self.get_parameter("object_id").value)
        self.service_timeout_sec = float(
            self.get_parameter("service_timeout_sec").value
        )

        self._callback_group = ReentrantCallbackGroup()
        self.scene_client = PlanningSceneClient(
            self,
            callback_group=self._callback_group,
        )
        self.scene_client.wait_until_ready(timeout_sec=self.service_timeout_sec)
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

    def _apply_scene(self, scene, label) -> bool:
        ok = self.scene_client.apply(
            scene,
            timeout_sec=self.service_timeout_sec,
        )
        if not ok:
            self.get_logger().warn(f"dynamic obstacle {label} apply failed")
        return ok

    def _on_obstacle_box(self, msg) -> None:
        if len(msg.data) == 0:
            scene = make_remove_dynamic_obstacle_scene(self.object_id)
            if self._apply_scene(scene, "remove"):
                self.get_logger().info(
                    "dynamic arm obstacle removed id=%s" % self.object_id
                )
            return
        try:
            center, size = parse_obstacle_box(msg.data)
        except ValueError as exc:
            self.get_logger().warn(str(exc))
            return

        scene = make_dynamic_obstacle_scene(
            center,
            size,
            frame_id=self.frame_id,
            object_id=self.object_id,
        )
        if self._apply_scene(scene, "update"):
            self.get_logger().info(
                "dynamic arm obstacle updated center=%s size=%s"
                % (center, size)
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
