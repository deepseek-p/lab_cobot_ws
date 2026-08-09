#!/usr/bin/env python3
"""Summarize G4/G5 side-channel results during the formal A-to-B mission."""
from __future__ import annotations

import math

import rclpy
from gazebo_msgs.msg import ContactsState
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Float64MultiArray, String

from lab_cobot_manipulation.contact_force_recorder import force_for_target
from lab_cobot_manipulation.gripper_driver import (
    CONTACT_STATUS_TOPIC,
    DEFAULT_TARGET_OBJECT,
    FINGERS_STATUS_TOPIC,
    LEFT_FINGER_CONTACTS_TOPIC,
    RIGHT_FINGER_CONTACTS_TOPIC,
)


G4_CONTACT_FORCE_TOPIC = "/gripper/contact/force"
G5_STATUS_TOPIC = "/g5/arm_dynamic_obstacle/status"
RESULT_TOPIC = "/task/g4g5_result"
FAILED_REPORT_DELAY_SEC = 30.0


def _finite_nonnegative(value) -> float:
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(0.0, value)


def parse_fingers_status(data: str) -> tuple[bool, bool]:
    """Return left/right tactile contact flags from the plugin status line."""
    fields = {}
    for token in str(data).split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key.strip()] = value.strip()
    return fields.get("left") == "1", fields.get("right") == "1"


def summarize_result(
    task_status: str,
    force_samples: int,
    peak_left_force_n: float,
    peak_right_force_n: float,
    peak_sum_force_n: float,
    left_touch_count: int,
    right_touch_count: int,
    contact_events: list[str],
    g5_events: list[str],
) -> str:
    """Build one compact report line for logs and the result topic."""
    g4_ok = (
        str(task_status) == "DONE"
        and left_touch_count > 0
        and right_touch_count > 0
    )
    g5_ready = any(str(event).startswith("ready ") for event in g5_events)
    return (
        "G4G5_RESULT "
        f"task_status={task_status} "
        f"g4_touch_ok={g4_ok} "
        f"g4_samples={int(force_samples)} "
        f"g4_peak_left_n={peak_left_force_n:.3f} "
        f"g4_peak_right_n={peak_right_force_n:.3f} "
        f"g4_peak_sum_n={peak_sum_force_n:.3f} "
        f"g4_left_touches={int(left_touch_count)} "
        f"g4_right_touches={int(right_touch_count)} "
        f"g4_last_contact='{contact_events[-1] if contact_events else 'none'}' "
        f"g5_bridge_ready={g5_ready} "
        f"g5_dynamic_obstacle_updates={sum(str(e).startswith('updated ') for e in g5_events)} "
        f"g5_last_status='{g5_events[-1] if g5_events else 'none'}'"
    )


class G4G5ResultNode(Node):
    """Listen to task, G4 contact and G5 bridge topics, then publish a summary."""

    def __init__(self):
        super().__init__("g4g5_result_node")
        self.declare_parameter("target_object", DEFAULT_TARGET_OBJECT)
        self.declare_parameter("result_topic", RESULT_TOPIC)
        self.declare_parameter("g5_status_topic", G5_STATUS_TOPIC)
        self.declare_parameter("failed_report_delay_sec", FAILED_REPORT_DELAY_SEC)

        self.target_object = str(self.get_parameter("target_object").value)
        result_topic = str(self.get_parameter("result_topic").value)
        g5_status_topic = str(self.get_parameter("g5_status_topic").value)
        self.failed_report_delay_sec = float(
            self.get_parameter("failed_report_delay_sec").value
        )

        self._force_samples = 0
        self._peak_left_force = 0.0
        self._peak_right_force = 0.0
        self._peak_sum_force = 0.0
        self._left_touch_count = 0
        self._right_touch_count = 0
        self._contact_events: list[str] = []
        self._g5_events: list[str] = []
        self._terminal_reported = False
        self._pending_failed = False
        self._failed_timer = None
        result_qos = QoSProfile(depth=1)
        result_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._result_pub = self.create_publisher(String, result_topic, result_qos)

        self.create_subscription(
            String,
            "/task/status",
            self._on_task_status,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            G4_CONTACT_FORCE_TOPIC,
            self._on_contact_force,
            10,
        )
        self.create_subscription(
            ContactsState,
            LEFT_FINGER_CONTACTS_TOPIC,
            lambda msg: self._on_finger_contacts("left", msg),
            10,
        )
        self.create_subscription(
            ContactsState,
            RIGHT_FINGER_CONTACTS_TOPIC,
            lambda msg: self._on_finger_contacts("right", msg),
            10,
        )
        self.create_subscription(
            String,
            FINGERS_STATUS_TOPIC,
            self._on_fingers_status,
            10,
        )
        self.create_subscription(
            String,
            CONTACT_STATUS_TOPIC,
            self._on_contact_status,
            10,
        )
        self.create_subscription(
            String,
            g5_status_topic,
            self._on_g5_status,
            10,
        )
        self.get_logger().info(
            "G4/G5 result summary listening target=%s result_topic=%s"
            % (self.target_object, result_topic)
        )

    def _on_contact_force(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        left = _finite_nonnegative(msg.data[0])
        right = _finite_nonnegative(msg.data[1])
        self._force_samples += 1
        self._peak_left_force = max(self._peak_left_force, left)
        self._peak_right_force = max(self._peak_right_force, right)
        self._peak_sum_force = max(self._peak_sum_force, left + right)
        if left > 0.0:
            self._left_touch_count += 1
        if right > 0.0:
            self._right_touch_count += 1

    def _on_finger_contacts(self, side: str, msg: ContactsState) -> None:
        force = force_for_target(msg, self.target_object)
        if force <= 0.0:
            return
        if side == "left":
            self._left_touch_count += 1
        else:
            self._right_touch_count += 1

    def _on_fingers_status(self, msg: String) -> None:
        left_touch, right_touch = parse_fingers_status(msg.data)
        if left_touch:
            self._left_touch_count += 1
        if right_touch:
            self._right_touch_count += 1

    def _on_contact_status(self, msg: String) -> None:
        self._contact_events.append(str(msg.data))
        self._contact_events = self._contact_events[-8:]

    def _on_g5_status(self, msg: String) -> None:
        self._g5_events.append(str(msg.data))
        self._g5_events = self._g5_events[-8:]

    def _on_task_status(self, msg: String) -> None:
        status = str(msg.data)
        if status not in ("DONE", "FAILED") or self._terminal_reported:
            return
        if status == "FAILED":
            self._pending_failed = True
            if self._failed_timer is None:
                self._failed_timer = self.create_timer(
                    max(self.failed_report_delay_sec, 0.0),
                    self._report_pending_failed,
                )
            return
        if self._failed_timer is not None:
            self._failed_timer.cancel()
        self._pending_failed = False
        self._publish_report(status)

    def _report_pending_failed(self) -> None:
        if self._failed_timer is not None:
            self._failed_timer.cancel()
        if not self._pending_failed or self._terminal_reported:
            return
        self._pending_failed = False
        self._publish_report("FAILED")

    def _publish_report(self, status: str) -> None:
        if self._terminal_reported:
            return
        self._terminal_reported = True
        report = summarize_result(
            status,
            self._force_samples,
            self._peak_left_force,
            self._peak_right_force,
            self._peak_sum_force,
            self._left_touch_count,
            self._right_touch_count,
            self._contact_events,
            self._g5_events,
        )
        out = String()
        out.data = report
        self._result_pub.publish(out)
        self.get_logger().info(report)


def main() -> int:
    rclpy.init()
    node = G4G5ResultNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
