"""Gripper backend boundary for simulation and future hardware drivers."""
from __future__ import annotations

import threading
import time
from typing import Protocol

from std_msgs.msg import Empty, Float64MultiArray, String


GRIPPER_COMMAND_TOPIC = "/gripper_position_controller/commands"
OPEN_POSITIONS = [0.0, 0.0]
CLOSED_ON_SAMPLE_POSITIONS = [0.012, 0.012]
ATTACH_TOPIC = "/gripper/attach/aruco_sample"
DETACH_TOPIC = "/gripper/detach/aruco_sample"
ATTACH_STATUS_TOPIC = "/gripper/attach/status"
ATTACH_ACCEPTED_PREFIX = "attached aruco_sample"
ATTACH_REFUSED_PREFIX = "refused aruco_sample"
DEFAULT_ATTACH_TIMEOUT_SEC = 1.5


class GripperDriver(Protocol):
    def open(self) -> bool:
        """Open the gripper."""

    def close(self) -> bool:
        """Close the gripper on the sample."""

    def acquire_object(self) -> bool:
        """Acquire the currently grasped object."""

    def release_object(self) -> bool:
        """Release the currently held object."""


class SimAttachGripperDriver:
    """Parallel gripper driver using finger commands plus sim attach topics."""

    def __init__(
        self,
        node,
        command_settle_sec: float = 0.0,
        attach_timeout_sec: float = DEFAULT_ATTACH_TIMEOUT_SEC,
    ) -> None:
        self._node = node
        self._command_settle_sec = float(command_settle_sec)
        self._attach_timeout_sec = float(attach_timeout_sec)
        self._attach_status_event = threading.Event()
        self._last_attach_status = ""
        self._command_pub = node.create_publisher(
            Float64MultiArray,
            GRIPPER_COMMAND_TOPIC,
            10,
        )
        self._attach_pub = node.create_publisher(Empty, ATTACH_TOPIC, 10)
        self._detach_pub = node.create_publisher(Empty, DETACH_TOPIC, 10)
        self._attach_status_sub = node.create_subscription(
            String,
            ATTACH_STATUS_TOPIC,
            self._on_attach_status,
            10,
        )

    def open(self) -> bool:
        self._publish_positions(OPEN_POSITIONS)
        self._log("夹爪打开")
        return True

    def close(self) -> bool:
        self._publish_positions(CLOSED_ON_SAMPLE_POSITIONS)
        self._log("夹爪闭合")
        return True

    def acquire_object(self) -> bool:
        self._last_attach_status = ""
        self._attach_status_event.clear()
        self._attach_pub.publish(Empty())
        self._log("夹爪请求 attach aruco_sample")

        if not self._attach_status_event.wait(timeout=self._attach_timeout_sec):
            self._warn(
                "夹爪 attach aruco_sample timed out waiting for bridge status"
            )
            return False

        if self._last_attach_status.startswith(ATTACH_ACCEPTED_PREFIX):
            self._log("夹爪 attach aruco_sample accepted")
            return True

        self._warn(f"夹爪 attach aruco_sample refused: {self._last_attach_status}")
        return False

    def release_object(self) -> bool:
        self._detach_pub.publish(Empty())
        self._log("夹爪 detach aruco_sample")
        return True

    def _on_attach_status(self, msg: String) -> None:
        data = str(msg.data)
        if data.startswith(ATTACH_ACCEPTED_PREFIX) or data.startswith(
            ATTACH_REFUSED_PREFIX
        ):
            self._last_attach_status = data
            self._attach_status_event.set()

    def _publish_positions(self, positions: list[float]) -> None:
        msg = Float64MultiArray()
        msg.data = list(positions)
        self._command_pub.publish(msg)
        if self._command_settle_sec > 0.0:
            time.sleep(self._command_settle_sec)

    def _log(self, message: str) -> None:
        logger = getattr(self._node, "get_logger", lambda: None)()
        if logger is not None:
            logger.info(message)

    def _warn(self, message: str) -> None:
        logger = getattr(self._node, "get_logger", lambda: None)()
        if logger is not None:
            logger.warn(message)
