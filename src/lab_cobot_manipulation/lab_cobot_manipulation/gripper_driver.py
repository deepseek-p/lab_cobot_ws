"""Gripper backend boundary for simulation and future hardware drivers."""
from __future__ import annotations

import threading
import time
from typing import Protocol

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import Empty, Float64MultiArray, String


GRIPPER_COMMAND_TOPIC = "/gripper_position_controller/commands"
OPEN_POSITIONS = [0.0, 0.0]
CLOSED_ON_SAMPLE_POSITIONS = [0.009, 0.009]
DEFAULT_TARGET_OBJECT = "aruco_sample"
ATTACH_TOPIC = "/gripper/attach/aruco_sample"
DETACH_TOPIC = "/gripper/detach/aruco_sample"
ATTACH_STATUS_TOPIC = "/gripper/attach/status"
ATTACH_ACCEPTED_PREFIX = "attached aruco_sample"
ATTACH_REFUSED_PREFIX = "refused aruco_sample"
DEFAULT_ATTACH_TIMEOUT_SEC = 1.5
CONTACT_STATUS_TOPIC = "/gripper/contact/status"
HOLD_STATUS_TOPIC = "/gripper/contact/hold_status"
CONTACT_RELEASE_TOPIC = "/gripper/contact/release"
FINGERS_STATUS_TOPIC = "/gripper/contact/fingers"
LEFT_FINGER_CONTACTS_TOPIC = "/gripper/left_finger_contacts"
RIGHT_FINGER_CONTACTS_TOPIC = "/gripper/right_finger_contacts"
CONTACT_ATTACHED_PREFIX = "attached "
CONTACT_RELEASED_PREFIX = "released "
CONTACT_REFUSED_PREFIX = "refused "
DEFAULT_CONTACT_TIMEOUT_SEC = 2.0
CONTACT_BACKEND = "contact"
SIM_ATTACH_BACKEND = "sim_attach"
TACTILE_START_POSITION = 0.006
TACTILE_STEP = 0.0005
TACTILE_MAX_POSITION = 0.0185
TACTILE_DWELL_SEC = 0.1
TACTILE_CONTACT_FRESH_SEC = 0.2
HOLD_STATUS_FRESH_SEC = 300.0


def contact_object_name(data: str) -> str:
    """Return the object name carried by a contact status line."""
    parts = str(data).split(maxsplit=2)
    if len(parts) < 2:
        return ""
    return parts[1]


def contact_event_matches(data: str, prefix: str, target: str | None) -> bool:
    """Check whether a contact status line matches an event and target."""
    text = str(data)
    if not text.startswith(prefix):
        return False
    if target is None:
        return True
    return contact_object_name(text) == target


def contact_refusal_reason(data: str) -> str:
    """Return the reason carried by a refused contact status line."""
    parts = str(data).split(maxsplit=2)
    if len(parts) < 3 or parts[0] != CONTACT_REFUSED_PREFIX.strip():
        return ""
    return parts[2]


def contacts_msg_touches_object(
    msg,
    prefix: str = f"{DEFAULT_TARGET_OBJECT}::",
) -> bool:
    """Return true when any Gazebo contact state touches the target object."""
    for state in getattr(msg, "states", []):
        if str(state.collision1_name).startswith(prefix):
            return True
        if str(state.collision2_name).startswith(prefix):
            return True
    return False


class GripperDriver(Protocol):
    def open(self) -> bool:
        """Open the gripper."""

    def close(self) -> bool:
        """Close the gripper on the sample."""

    def acquire_object(self) -> bool:
        """Acquire the currently grasped object."""

    def release_object(self) -> bool:
        """Release the currently held object."""

    def last_tactile_contact_sides(self) -> tuple[bool, bool]:
        """Return whether left/right fingers touched during the last close."""

    def is_holding_object(self) -> bool:
        """Return whether the backend currently confirms the target is held."""


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
        self._holding_object = False
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
            self._holding_object = True
            self._log("夹爪 attach aruco_sample accepted")
            return True

        self._warn(f"夹爪 attach aruco_sample refused: {self._last_attach_status}")
        return False

    def release_object(self) -> bool:
        self._detach_pub.publish(Empty())
        self._holding_object = False
        self._log("夹爪 detach aruco_sample")
        return True

    def last_tactile_contact_sides(self) -> tuple[bool, bool]:
        return (False, False)

    def is_holding_object(self) -> bool:
        return self._holding_object

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


class ContactGripperDriver:
    """Parallel gripper driver that relies on Gazebo contact physics/grasp plugins."""

    def __init__(
        self,
        node,
        command_settle_sec: float = 0.0,
        contact_timeout_sec: float = DEFAULT_CONTACT_TIMEOUT_SEC,
        target_object: str = DEFAULT_TARGET_OBJECT,
        use_tactile_grasp: bool = False,
        tactile_dwell_sec: float = TACTILE_DWELL_SEC,
    ) -> None:
        self._node = node
        self._command_settle_sec = float(command_settle_sec)
        self._contact_timeout_sec = float(contact_timeout_sec)
        self._target_object = str(target_object)
        self._use_tactile_grasp = bool(use_tactile_grasp)
        self._tactile_dwell_sec = float(tactile_dwell_sec)
        self._contact_status_event = threading.Event()
        self._last_contact_status = ""
        self._holding_object = False
        self._last_hold_status_time = None
        self._last_left_contact_time = None
        self._last_right_contact_time = None
        self._command_pub = node.create_publisher(
            Float64MultiArray,
            GRIPPER_COMMAND_TOPIC,
            10,
        )
        self._release_pub = node.create_publisher(Empty, CONTACT_RELEASE_TOPIC, 10)
        self._contact_status_sub = node.create_subscription(
            String,
            CONTACT_STATUS_TOPIC,
            self._on_contact_status,
            10,
        )
        self._hold_status_sub = node.create_subscription(
            String,
            HOLD_STATUS_TOPIC,
            self._on_hold_status,
            10,
        )
        self._fingers_status_sub = node.create_subscription(
            String,
            FINGERS_STATUS_TOPIC,
            self._on_fingers_status,
            10,
        )
        self._left_contact_sub = node.create_subscription(
            ContactsState,
            LEFT_FINGER_CONTACTS_TOPIC,
            self._on_left_contacts,
            10,
        )
        self._right_contact_sub = node.create_subscription(
            ContactsState,
            RIGHT_FINGER_CONTACTS_TOPIC,
            self._on_right_contacts,
            10,
        )

    def open(self) -> bool:
        self._publish_positions(OPEN_POSITIONS)
        self._holding_object = False
        self._log("夹爪打开")
        return True

    def close(self) -> bool:
        if self._holding_object:
            self._log("夹爪已闭合")
            return True
        self._publish_positions(CLOSED_ON_SAMPLE_POSITIONS)
        self._log("夹爪闭合")
        return True

    def acquire_object(self) -> bool:
        self._last_contact_status = ""
        self._contact_status_event.clear()
        if self._use_tactile_grasp:
            if not self._step_close_until_contact():
                self._holding_object = False
                return False
        else:
            self.close()
        if self._wait_for_contact_status(
            CONTACT_ATTACHED_PREFIX,
            target=self._target_object,
            fail_on_refused=True,
        ):
            self._log(f"夹爪 contact attach {self._target_object} accepted")
            self._holding_object = True
            # 给插件下一次 10Hz 持有心跳一个启动窗口；窗口结束后必须由
            # _on_hold_status 刷新，否则 is_holding_object 会判定为丢失。
            self._last_hold_status_time = time.monotonic()
            return True
        if contact_event_matches(
            self._last_contact_status,
            CONTACT_ATTACHED_PREFIX,
            target=None,
        ):
            attached = contact_object_name(self._last_contact_status)
            self._warn(f"夹爪 contact attached wrong object: {attached}")
            self._release_pub.publish(Empty())
            self._holding_object = False
            return False
        if self._last_contact_status.startswith(CONTACT_REFUSED_PREFIX):
            self._warn(f"夹爪 contact attach refused: {self._last_contact_status}")
            self._holding_object = False
            return False
        self._warn("夹爪 contact attach timed out waiting for grasp plugin")
        self._holding_object = False
        return False

    def release_object(self) -> bool:
        self._last_contact_status = ""
        self._contact_status_event.clear()
        self._release_pub.publish(Empty())
        self._log(f"夹爪请求 contact release {self._target_object}")
        if self._wait_for_contact_status(
            CONTACT_RELEASED_PREFIX,
            target=None,
            fail_on_refused=False,
        ):
            self._log("夹爪 contact release accepted")
            self._holding_object = False
            return True
        self._warn("夹爪 contact release timed out waiting for grasp plugin")
        return False

    def last_tactile_contact_sides(self) -> tuple[bool, bool]:
        return (
            self._last_left_contact_time is not None,
            self._last_right_contact_time is not None,
        )

    def is_holding_object(self) -> bool:
        """Require a fresh plugin heartbeat, not just an old attach event."""
        if not self._holding_object or self._last_hold_status_time is None:
            return False
        return time.monotonic() - self._last_hold_status_time <= HOLD_STATUS_FRESH_SEC

    def refresh_holding_watchdog(self) -> None:
        """Start carry monitoring from now after an accepted contact attach."""
        if self._holding_object:
            self._last_hold_status_time = time.monotonic()

    def _on_contact_status(self, msg: String) -> None:
        self._last_contact_status = str(msg.data)
        if contact_event_matches(
            self._last_contact_status,
            CONTACT_RELEASED_PREFIX,
            target=self._target_object,
        ):
            self._holding_object = False
        self._contact_status_event.set()

    def _on_hold_status(self, msg: String) -> None:
        """Consume the grasp-plugin heartbeat for live carry monitoring."""
        status = str(msg.data)
        if contact_event_matches(status, "holding ", self._target_object):
            self._last_hold_status_time = time.monotonic()
            return
        if status.startswith("lost ") or status.startswith("empty"):
            self._holding_object = False

    def _on_fingers_status(self, msg: String) -> None:
        """Refresh per-finger contact times from the plugin snapshot topic."""
        # 插件 1kHz 权威判定的 50Hz 快照;bumper 上报率过低(实测 1/50)导致
        # 分侧停步长期失灵、probe 深穿透(实测 3mm)诱发接触求解爆发。
        now = time.monotonic()
        if "left=1" in msg.data:
            self._last_left_contact_time = now
        if "right=1" in msg.data:
            self._last_right_contact_time = now

    def _on_left_contacts(self, msg: ContactsState) -> None:
        if contacts_msg_touches_object(msg, prefix=f"{self._target_object}::"):
            self._last_left_contact_time = time.monotonic()

    def _on_right_contacts(self, msg: ContactsState) -> None:
        if contacts_msg_touches_object(msg, prefix=f"{self._target_object}::"):
            self._last_right_contact_time = time.monotonic()

    def _both_fingers_touch_target(self) -> bool:
        now = time.monotonic()
        return (
            self._left_finger_touches_target(now)
            and self._right_finger_touches_target(now)
        )

    def _left_finger_touches_target(self, now: float | None = None) -> bool:
        if self._last_left_contact_time is None:
            return False
        if now is None:
            now = time.monotonic()
        return now - self._last_left_contact_time <= TACTILE_CONTACT_FRESH_SEC

    def _right_finger_touches_target(self, now: float | None = None) -> bool:
        if self._last_right_contact_time is None:
            return False
        if now is None:
            now = time.monotonic()
        return now - self._last_right_contact_time <= TACTILE_CONTACT_FRESH_SEC

    def _wait_for_contact_status(
        self,
        prefix: str,
        target: str | None,
        fail_on_refused: bool,
    ) -> bool:
        deadline = time.monotonic() + self._contact_timeout_sec
        while True:
            if contact_event_matches(self._last_contact_status, prefix, target):
                return True
            if (
                fail_on_refused
                and self._last_contact_status.startswith(CONTACT_REFUSED_PREFIX)
            ):
                if not self._tactile_no_contact_refusal(self._last_contact_status):
                    return False
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._contact_status_event.wait(timeout=remaining)
            self._contact_status_event.clear()

    def _step_close_until_contact(self) -> bool:
        self._last_left_contact_time = None
        self._last_right_contact_time = None
        left_position = TACTILE_START_POSITION
        right_position = TACTILE_START_POSITION
        while (
            left_position <= TACTILE_MAX_POSITION + 1e-12
            and right_position <= TACTILE_MAX_POSITION + 1e-12
        ):
            msg = Float64MultiArray()
            msg.data = [round(left_position, 4), round(right_position, 4)]
            self._command_pub.publish(msg)
            if self._tactile_dwell_sec > 0.0:
                time.sleep(self._tactile_dwell_sec)
            if contact_event_matches(
                self._last_contact_status,
                CONTACT_ATTACHED_PREFIX,
                target=self._target_object,
            ):
                self._log(f"夹爪触觉闭合已附着 {self._target_object}")
                return True
            if contact_event_matches(
                self._last_contact_status,
                CONTACT_REFUSED_PREFIX,
                target=self._target_object,
            ):
                if self._tactile_no_contact_refusal(self._last_contact_status):
                    self._last_contact_status = ""
                else:
                    self._warn(
                        "夹爪触觉闭合被插件拒绝: "
                        f"{self._last_contact_status}"
                    )
                    return False
            now = time.monotonic()
            left_touching = self._left_finger_touches_target(now)
            right_touching = self._right_finger_touches_target(now)
            if left_touching and right_touching:
                self._log(f"夹爪触觉闭合接触 {self._target_object}")
                return True
            if not left_touching:
                left_position += TACTILE_STEP
            if not right_touching:
                right_position += TACTILE_STEP
        self._warn(f"夹爪触觉闭合到上限仍未双指接触 {self._target_object}")
        return False

    def _tactile_no_contact_refusal(self, status: str) -> bool:
        return (
            self._use_tactile_grasp
            and contact_event_matches(
                status,
                CONTACT_REFUSED_PREFIX,
                target=self._target_object,
            )
            and contact_refusal_reason(status) == "no_finger_contact"
        )

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


def make_gripper_driver(
    node,
    backend: str = CONTACT_BACKEND,
    command_settle_sec: float = 0.0,
    attach_timeout_sec: float = DEFAULT_ATTACH_TIMEOUT_SEC,
    target_object: str = DEFAULT_TARGET_OBJECT,
    use_tactile_grasp: bool = False,
    contact_timeout_sec: float = DEFAULT_CONTACT_TIMEOUT_SEC,
) -> GripperDriver:
    normalized = str(backend).strip().lower()
    if normalized == SIM_ATTACH_BACKEND:
        if target_object != DEFAULT_TARGET_OBJECT:
            raise ValueError("sim_attach backend only supports default target_object")
        return SimAttachGripperDriver(
            node,
            command_settle_sec=command_settle_sec,
            attach_timeout_sec=attach_timeout_sec,
        )
    if normalized == CONTACT_BACKEND:
        return ContactGripperDriver(
            node,
            command_settle_sec=command_settle_sec,
            contact_timeout_sec=contact_timeout_sec,
            target_object=target_object,
            use_tactile_grasp=use_tactile_grasp,
        )
    raise ValueError(
        "unsupported gripper backend %r; expected %r or %r"
        % (backend, CONTACT_BACKEND, SIM_ATTACH_BACKEND)
    )
