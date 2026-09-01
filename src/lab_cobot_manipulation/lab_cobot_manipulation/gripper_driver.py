"""Gripper backend boundary for simulation and future hardware drivers."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import threading
import time
from typing import Protocol

from gazebo_msgs.msg import ContactsState
from sensor_msgs.msg import JointState
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
CONTACT_FORCE_TOPIC = "/gripper/contact/force"
FORCE_CONTROL_STATUS_TOPIC = "/gripper/force_control/status"
CONTACT_ATTACHED_PREFIX = "attached "
CONTACT_RELEASED_PREFIX = "released "
CONTACT_REFUSED_PREFIX = "refused "
DEFAULT_CONTACT_TIMEOUT_SEC = 2.0
CONTACT_BACKEND = "contact"
SIM_ATTACH_BACKEND = "sim_attach"
TACTILE_START_POSITION = 0.006
TACTILE_STEP = 0.00025
TACTILE_MAX_POSITION = 0.0185
TACTILE_DWELL_SEC = 0.1
TACTILE_CONTACT_FRESH_SEC = 0.2
TACTILE_FORCE_FRESH_SEC = 0.5
TACTILE_TARGET_FORCE_N = 2.0
TACTILE_MAX_FORCE_N = 18.0
FORCE_CONTROL_TARGET_N = 8.0
FORCE_CONTROL_DEADBAND_N = 1.0
FORCE_CONTROL_KP = 0.00020
FORCE_CONTROL_MAX_CLOSE_STEP = 0.00030
FORCE_CONTROL_MAX_OPEN_STEP = 0.00008
FORCE_CONTROL_SAFETY_LIMIT_N = 18.0
FORCE_CONTROL_SAFETY_FRAMES = 3
FORCE_CONTROL_BALANCE_LIMIT_N = 12.0
FORCE_CONTROL_BALANCE_FRAMES = 3
FORCE_CONTROL_SETTLE_FRAMES = 3
FORCE_CONTROL_FILTER_WINDOW = 3
HOLD_STATUS_FRESH_SEC = 300.0
HOLD_CONFIRM_TIMEOUT_SEC = DEFAULT_CONTACT_TIMEOUT_SEC
HOLD_CONFIRM_CLOCK_STALL_SEC = 12.0
HOLD_CONFIRM_WAIT_LOG_PERIOD_SEC = 1.0
GRIPPER_LEFT_JOINT = "gripper_left_finger_joint"
GRIPPER_RIGHT_JOINT = "gripper_right_finger_joint"
GRIPPER_URDF_UPPER_LIMIT = 0.035
GRIPPER_OPEN_INNER_GAP_M = 0.092

FORCE_SOURCE_RAW_GAZEBO_WRENCH = "RAW_GAZEBO_WRENCH"
FORCE_SOURCE_VIRTUAL_ESTIMATE = "VIRTUAL_ESTIMATE"
FORCE_SOURCE_HARDWARE_SENSOR = "HARDWARE_SENSOR"
FORCE_SOURCE_INVALID = "INVALID"


@dataclass(frozen=True)
class ForceSample:
    left_force: float
    right_force: float
    source: str
    left_valid: bool
    right_valid: bool
    timestamp: float | None

    @property
    def both_valid(self) -> bool:
        return self.left_valid and self.right_valid


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

    def command_positions(
        self,
        positions: list[float] | tuple[float, float],
        settle_sec: float | None = None,
    ) -> bool:
        """Send explicit finger joint positions."""

    def acquire_object(self) -> bool:
        """Acquire the currently grasped object."""

    def release_object(self) -> bool:
        """Release the currently held object."""

    def last_tactile_contact_sides(self) -> tuple[bool, bool]:
        """Return whether left/right fingers touched during the last close."""

    def actual_finger_positions(self) -> tuple[float | None, float | None]:
        """Return last observed left/right finger joint positions."""

    def estimated_gap_mm(
        self,
        command: list[float] | tuple[float, float] | None = None,
    ) -> float:
        """Return estimated inner finger gap in millimeters."""

    def is_holding_object(self) -> bool:
        """Return whether the backend currently confirms the target is held."""

    def wait_until_holding(self, timeout_sec: float = HOLD_CONFIRM_TIMEOUT_SEC) -> bool:
        """Wait until the backend confirms that the target is held."""


class SimAttachGripperDriver:
    """Parallel gripper driver using finger commands plus sim attach topics."""

    def __init__(
        self,
        node,
        command_settle_sec: float = 0.0,
        open_settle_sec: float | None = None,
        attach_timeout_sec: float = DEFAULT_ATTACH_TIMEOUT_SEC,
    ) -> None:
        self._node = node
        self._command_settle_sec = float(command_settle_sec)
        self._open_settle_sec = (
            float(command_settle_sec)
            if open_settle_sec is None
            else float(open_settle_sec)
        )
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
        self._publish_positions(OPEN_POSITIONS, settle_sec=self._open_settle_sec)
        self._log("夹爪打开")
        return True

    def close(self) -> bool:
        self._publish_positions(CLOSED_ON_SAMPLE_POSITIONS)
        self._log("夹爪闭合")
        return True

    def command_positions(
        self,
        positions: list[float] | tuple[float, float],
        settle_sec: float | None = None,
    ) -> bool:
        self._publish_positions(list(positions), settle_sec=settle_sec)
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

    def actual_finger_positions(self) -> tuple[float | None, float | None]:
        return (None, None)

    def estimated_gap_mm(
        self,
        command: list[float] | tuple[float, float] | None = None,
    ) -> float:
        if command is None:
            command = OPEN_POSITIONS
        return (
            GRIPPER_OPEN_INNER_GAP_M
            - float(command[0])
            - float(command[1])
        ) * 1000.0

    def is_holding_object(self) -> bool:
        return self._holding_object

    def wait_until_holding(self, timeout_sec: float = HOLD_CONFIRM_TIMEOUT_SEC) -> bool:
        return self._holding_object

    def _on_attach_status(self, msg: String) -> None:
        data = str(msg.data)
        if data.startswith(ATTACH_ACCEPTED_PREFIX) or data.startswith(
            ATTACH_REFUSED_PREFIX
        ):
            self._last_attach_status = data
            self._attach_status_event.set()

    def _publish_positions(
        self,
        positions: list[float],
        settle_sec: float | None = None,
    ) -> None:
        msg = Float64MultiArray()
        msg.data = list(positions)
        self._command_pub.publish(msg)
        wait_sec = self._command_settle_sec if settle_sec is None else float(settle_sec)
        if wait_sec > 0.0:
            time.sleep(wait_sec)

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
        open_settle_sec: float | None = None,
        contact_timeout_sec: float = DEFAULT_CONTACT_TIMEOUT_SEC,
        target_object: str = DEFAULT_TARGET_OBJECT,
        use_tactile_grasp: bool = False,
        tactile_dwell_sec: float = TACTILE_DWELL_SEC,
        tactile_start_position: float | None = None,
        tactile_step_position: float | None = None,
        tactile_max_position: float | None = None,
        tactile_log_prefix: str = "TACTILE",
        open_positions: list[float] | tuple[float, float] | None = None,
        expected_object_width_mm: float | None = None,
        tactile_target_force_n: float | None = None,
        tactile_max_force_n: float | None = None,
        enable_force_gate: bool = False,
        enable_force_control: bool = False,
        force_target_n: float = FORCE_CONTROL_TARGET_N,
        force_deadband_n: float = FORCE_CONTROL_DEADBAND_N,
        force_kp: float = FORCE_CONTROL_KP,
        force_max_close_step: float = FORCE_CONTROL_MAX_CLOSE_STEP,
        force_max_open_step: float = FORCE_CONTROL_MAX_OPEN_STEP,
        force_safety_limit_n: float = FORCE_CONTROL_SAFETY_LIMIT_N,
        force_safety_frames: int = FORCE_CONTROL_SAFETY_FRAMES,
        force_balance_limit_n: float = FORCE_CONTROL_BALANCE_LIMIT_N,
        force_balance_frames: int = FORCE_CONTROL_BALANCE_FRAMES,
        force_settle_frames: int = FORCE_CONTROL_SETTLE_FRAMES,
        force_filter_window: int = FORCE_CONTROL_FILTER_WINDOW,
    ) -> None:
        self._node = node
        self._command_settle_sec = float(command_settle_sec)
        self._open_settle_sec = (
            float(command_settle_sec)
            if open_settle_sec is None
            else float(open_settle_sec)
        )
        self._contact_timeout_sec = float(contact_timeout_sec)
        self._target_object = str(target_object)
        self._use_tactile_grasp = bool(use_tactile_grasp)
        self._tactile_dwell_sec = float(tactile_dwell_sec)
        self._tactile_start_position = self._normalize_tactile_position(
            tactile_start_position,
            TACTILE_START_POSITION,
        )
        self._tactile_step_position = self._normalize_tactile_step_position(
            tactile_step_position
        )
        self._tactile_max_position = self._normalize_tactile_max_position(
            tactile_max_position
        )
        self._tactile_log_prefix = str(tactile_log_prefix).strip() or "TACTILE"
        self._open_positions = self._normalize_finger_positions(open_positions)
        self._expected_object_width_mm = (
            None if expected_object_width_mm is None else float(expected_object_width_mm)
        )
        self._tactile_target_force_n = self._normalize_force_limit(
            tactile_target_force_n,
            TACTILE_TARGET_FORCE_N,
        )
        self._tactile_max_force_n = max(
            self._tactile_target_force_n,
            self._normalize_force_limit(tactile_max_force_n, TACTILE_MAX_FORCE_N),
        )
        self._enable_force_gate = bool(enable_force_gate)
        self._enable_force_control = bool(enable_force_control)
        self._force_target_n = self._normalize_force_limit(
            force_target_n,
            FORCE_CONTROL_TARGET_N,
        )
        self._force_deadband_n = max(0.0, float(force_deadband_n))
        self._force_kp = max(0.0, float(force_kp))
        self._force_max_close_step = self._normalize_gripper_step(
            force_max_close_step,
            FORCE_CONTROL_MAX_CLOSE_STEP,
        )
        self._force_max_open_step = self._normalize_gripper_step(
            force_max_open_step,
            FORCE_CONTROL_MAX_OPEN_STEP,
        )
        self._force_safety_limit_n = self._normalize_force_limit(
            force_safety_limit_n,
            FORCE_CONTROL_SAFETY_LIMIT_N,
        )
        self._force_safety_frames = max(1, int(force_safety_frames))
        self._force_balance_limit_n = self._normalize_force_limit(
            force_balance_limit_n,
            FORCE_CONTROL_BALANCE_LIMIT_N,
        )
        self._force_balance_frames = max(1, int(force_balance_frames))
        self._force_settle_frames = max(1, int(force_settle_frames))
        self._force_filter_window = max(1, int(force_filter_window))
        self._contact_status_event = threading.Event()
        self._last_contact_status = ""
        self._holding_object = False
        self._release_pending = False
        self._last_hold_status = ""
        self._last_hold_status_time = None
        self._last_confirmed_holding_time = None
        self._attach_confirm_ros_time = None
        self._last_left_contact_time = None
        self._last_right_contact_time = None
        self._last_left_joint_position = None
        self._last_right_joint_position = None
        self._last_left_force_n = 0.0
        self._last_right_force_n = 0.0
        self._last_force_source = FORCE_SOURCE_INVALID
        self._last_left_force_valid = False
        self._last_right_force_valid = False
        self._last_force_time = None
        self._force_filter_samples: deque[tuple[float, float]] = deque(
            maxlen=self._force_filter_window
        )
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
        self._contact_force_sub = self._try_create_subscription(
            Float64MultiArray,
            CONTACT_FORCE_TOPIC,
            self._on_contact_force,
            10,
        )
        self._force_control_status_pub = node.create_publisher(
            String,
            FORCE_CONTROL_STATUS_TOPIC,
            10,
        )
        self._joint_state_sub = node.create_subscription(
            JointState,
            "/joint_states",
            self._on_joint_state,
            10,
        )

    def open(self) -> bool:
        self._publish_positions(self._open_positions, settle_sec=self._open_settle_sec)
        self._holding_object = False
        self._release_pending = False
        self._log("夹爪打开")
        return True

    def close(self) -> bool:
        if self._holding_object:
            self._log("夹爪已闭合")
            return True
        self._publish_positions(CLOSED_ON_SAMPLE_POSITIONS)
        self._log("夹爪闭合")
        return True

    def command_positions(
        self,
        positions: list[float] | tuple[float, float],
        settle_sec: float | None = None,
    ) -> bool:
        normalized = self._normalize_finger_positions(positions)
        self._publish_positions(normalized, settle_sec=settle_sec)
        return True

    def acquire_object(self) -> bool:
        self._last_contact_status = ""
        self._contact_status_event.clear()
        self._release_pending = False
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
            self._attach_confirm_ros_time = self._ros_now_sec()
            self._last_confirmed_holding_time = None
            self._last_hold_status = ""
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
        self._last_hold_status = ""
        self._contact_status_event.clear()
        self._release_pending = True
        self._release_pub.publish(Empty())
        self._log(f"夹爪请求 contact release {self._target_object}")
        if self._wait_for_release_status():
            self._log("夹爪 contact release accepted")
            self._holding_object = False
            self._release_pending = False
            return True
        self._warn("夹爪 contact release timed out waiting for grasp plugin")
        return False

    def last_tactile_contact_sides(self) -> tuple[bool, bool]:
        return (
            self._last_left_contact_time is not None,
            self._last_right_contact_time is not None,
        )

    def actual_finger_positions(self) -> tuple[float | None, float | None]:
        return self._actual_finger_positions()

    def estimated_gap_mm(
        self,
        command: list[float] | tuple[float, float] | None = None,
    ) -> float:
        if command is None:
            left_actual, right_actual = self._actual_finger_positions()
            if left_actual is not None and right_actual is not None:
                return self._gripper_gap_mm(left_actual, right_actual)
            command = self._open_positions
        return self._estimated_gap_mm(list(command))

    def is_holding_object(self) -> bool:
        """Require a fresh plugin heartbeat, not just an old attach event."""
        if not self._holding_object or self._last_hold_status_time is None:
            return False
        return time.monotonic() - self._last_hold_status_time <= HOLD_STATUS_FRESH_SEC

    def refresh_holding_watchdog(self) -> None:
        """Start carry monitoring from now after an accepted contact attach."""
        if self._holding_object:
            self._last_hold_status_time = time.monotonic()

    def wait_until_holding(self, timeout_sec: float = HOLD_CONFIRM_TIMEOUT_SEC) -> bool:
        if not self._holding_object:
            return False
        start_ros = self._ros_now_sec()
        start_wall = time.monotonic()
        last_progress_ros = start_ros
        last_progress_wall = start_wall
        last_log_wall = start_wall
        self._log(
            "HOLD_CONFIRM_WAIT_START target=%s timeout_sec=%.3f"
            % (self._target_object, float(timeout_sec))
        )
        while True:
            now_ros = self._ros_now_sec()
            now_wall = time.monotonic()
            if now_ros > last_progress_ros + 1.0e-6:
                last_progress_ros = now_ros
                last_progress_wall = now_wall
            if self._holding_confirmed_after_attach():
                sim_elapsed = max(0.0, now_ros - start_ros)
                wall_elapsed = max(0.0, now_wall - start_wall)
                self._log(
                    "HOLD_CONFIRMED target=%s sim_elapsed=%.3f wall_elapsed=%.3f"
                    % (self._target_object, sim_elapsed, wall_elapsed)
                )
                return True
            if not self._holding_object:
                return False
            sim_elapsed = max(0.0, now_ros - start_ros)
            wall_elapsed = max(0.0, now_wall - start_wall)
            if now_wall - last_log_wall >= HOLD_CONFIRM_WAIT_LOG_PERIOD_SEC:
                self._log(
                    "HOLD_CONFIRM_WAIT target=%s sim_elapsed=%.3f "
                    "wall_elapsed=%.3f last_hold_status=%s"
                    % (
                        self._target_object,
                        sim_elapsed,
                        wall_elapsed,
                        self._last_hold_status or "none",
                    )
                )
                last_log_wall = now_wall
            if sim_elapsed >= max(float(timeout_sec), 0.0):
                self._warn(
                    "HOLD_CONFIRM_TIMEOUT target=%s sim_elapsed=%.3f "
                    "wall_elapsed=%.3f last_hold_status=%s"
                    % (
                        self._target_object,
                        sim_elapsed,
                        wall_elapsed,
                        self._last_hold_status or "none",
                    )
                )
                return False
            if now_wall - last_progress_wall >= HOLD_CONFIRM_CLOCK_STALL_SEC:
                self._warn(
                    "HOLD_CONFIRM_CLOCK_STALLED target=%s wall_elapsed=%.3f "
                    "last_hold_status=%s"
                    % (
                        self._target_object,
                        wall_elapsed,
                        self._last_hold_status or "none",
                    )
                )
                return False
            self._contact_status_event.wait(timeout=0.05)
            self._contact_status_event.clear()

    def _holding_confirmed_after_attach(self) -> bool:
        if self._last_confirmed_holding_time is None:
            return False
        if self._attach_confirm_ros_time is None:
            return True
        return self._last_confirmed_holding_time >= self._attach_confirm_ros_time

    def _ros_now_sec(self) -> float:
        get_clock = getattr(self._node, "get_clock", None)
        if get_clock is None:
            return time.monotonic()
        try:
            now = get_clock().now()
        except Exception:  # noqa: BLE001
            return time.monotonic()
        nanoseconds = getattr(now, "nanoseconds", None)
        if nanoseconds is not None:
            return float(nanoseconds) * 1.0e-9
        seconds_nanoseconds = getattr(now, "seconds_nanoseconds", None)
        if callable(seconds_nanoseconds):
            sec, nanosec = seconds_nanoseconds()
            return float(sec) + float(nanosec) * 1.0e-9
        try:
            return float(now)
        except (TypeError, ValueError):
            return time.monotonic()

    def _on_contact_status(self, msg: String) -> None:
        self._last_contact_status = str(msg.data)
        if contact_event_matches(
            self._last_contact_status,
            CONTACT_RELEASED_PREFIX,
            target=self._target_object,
        ):
            self._holding_object = False
            self._release_pending = False
        self._contact_status_event.set()

    def _on_hold_status(self, msg: String) -> None:
        """Consume the grasp-plugin heartbeat for live carry monitoring."""
        status = str(msg.data)
        self._last_hold_status = status
        if contact_event_matches(status, "holding ", self._target_object):
            self._last_hold_status_time = time.monotonic()
            if self._holding_object:
                self._last_confirmed_holding_time = self._ros_now_sec()
            self._contact_status_event.set()
            return
        if status.startswith("lost ") or status.startswith("empty"):
            self._holding_object = False
            self._release_pending = False
            self._contact_status_event.set()

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

    def _on_joint_state(self, msg: JointState) -> None:
        positions_by_name = dict(zip(msg.name, msg.position))
        if GRIPPER_LEFT_JOINT in positions_by_name:
            self._last_left_joint_position = float(
                positions_by_name[GRIPPER_LEFT_JOINT]
            )
        if GRIPPER_RIGHT_JOINT in positions_by_name:
            self._last_right_joint_position = float(
                positions_by_name[GRIPPER_RIGHT_JOINT]
            )

    def _on_contact_force(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            return
        self._last_left_force_n = self._finite_nonnegative(msg.data[0])
        self._last_right_force_n = self._finite_nonnegative(msg.data[1])
        if len(msg.data) >= 5:
            source_code = int(float(msg.data[2]))
            self._last_force_source = {
                1: FORCE_SOURCE_RAW_GAZEBO_WRENCH,
                2: FORCE_SOURCE_VIRTUAL_ESTIMATE,
                3: FORCE_SOURCE_HARDWARE_SENSOR,
            }.get(source_code, FORCE_SOURCE_INVALID)
            self._last_left_force_valid = float(msg.data[3]) >= 0.5
            self._last_right_force_valid = float(msg.data[4]) >= 0.5
        else:
            self._last_force_source = FORCE_SOURCE_INVALID
            self._last_left_force_valid = False
            self._last_right_force_valid = False
        self._last_force_time = time.monotonic()

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

    def _fresh_contact_forces(self, now: float | None = None) -> tuple[float, float]:
        if now is None:
            now = time.monotonic()
        if (
            self._last_force_time is None
            or now - self._last_force_time > TACTILE_FORCE_FRESH_SEC
        ):
            return (0.0, 0.0)
        return (self._last_left_force_n, self._last_right_force_n)

    def _fresh_force_sample(self, now: float | None = None) -> ForceSample:
        if now is None:
            now = time.monotonic()
        if (
            self._last_force_time is None
            or now - self._last_force_time > TACTILE_FORCE_FRESH_SEC
        ):
            return ForceSample(
                0.0,
                0.0,
                FORCE_SOURCE_INVALID,
                False,
                False,
                self._last_force_time,
            )
        source = self._last_force_source
        left_valid = self._last_left_force_valid
        right_valid = self._last_right_force_valid
        if source == FORCE_SOURCE_VIRTUAL_ESTIMATE:
            # VirtualFingerForce is a software estimate published in contact-only
            # mode.  Once fresh, it is valid for algorithm prototyping but not as
            # a real wrench measurement.
            left_valid = True
            right_valid = True
        elif source == FORCE_SOURCE_HARDWARE_SENSOR:
            left_valid = True
            right_valid = True
        return ForceSample(
            self._last_left_force_n,
            self._last_right_force_n,
            source,
            left_valid,
            right_valid,
            self._last_force_time,
        )

    def _has_fresh_force_sample(self, now: float | None = None) -> bool:
        if now is None:
            now = time.monotonic()
        return (
            self._last_force_time is not None
            and now - self._last_force_time <= TACTILE_FORCE_FRESH_SEC
        )

    def _filtered_forces(self, sample: ForceSample) -> tuple[float, float]:
        self._force_filter_samples.append((sample.left_force, sample.right_force))
        count = len(self._force_filter_samples)
        if count <= 0:
            return sample.left_force, sample.right_force
        left = sum(value[0] for value in self._force_filter_samples) / count
        right = sum(value[1] for value in self._force_filter_samples) / count
        return left, right

    def _publish_force_control_status(
        self,
        state: str,
        command: list[float],
        sample: ForceSample,
        filtered_left: float,
        filtered_right: float,
        delta_q: float,
    ) -> None:
        if not self._enable_force_control:
            return
        mean_force = 0.5 * (filtered_left + filtered_right)
        error = self._force_target_n - mean_force
        balance = abs(filtered_left - filtered_right)
        left_actual, right_actual = self._actual_finger_positions()
        msg = String()
        msg.data = (
            "FORCE_CONTROL "
            f"state={state} "
            f"target={self._target_object} "
            f"command_left={float(command[0]):.6f} "
            f"command_right={float(command[1]):.6f} "
            f"left_actual={self._format_optional_float(left_actual)} "
            f"right_actual={self._format_optional_float(right_actual)} "
            f"force_left_raw={sample.left_force:.6f} "
            f"force_right_raw={sample.right_force:.6f} "
            f"force_left_filtered={filtered_left:.6f} "
            f"force_right_filtered={filtered_right:.6f} "
            f"force_mean={mean_force:.6f} "
            f"force_error={error:.6f} "
            f"force_balance_error={balance:.6f} "
            f"delta_q={delta_q:.6f} "
            f"force_source={sample.source} "
            f"left_valid={int(sample.left_valid)} "
            f"right_valid={int(sample.right_valid)}"
        )
        self._force_control_status_pub.publish(msg)
        self._log(msg.data)

    def _force_target_reached(self, now: float | None = None) -> bool:
        if not self._enable_force_gate:
            return True
        if not self._has_fresh_force_sample(now):
            return True
        left_force, right_force = self._fresh_contact_forces(now)
        return (
            left_force >= self._tactile_target_force_n
            and right_force >= self._tactile_target_force_n
        )

    def _force_limit_exceeded(self, now: float | None = None) -> bool:
        if not self._enable_force_gate:
            return False
        if not self._has_fresh_force_sample(now):
            return False
        left_force, right_force = self._fresh_contact_forces(now)
        return (
            left_force > self._tactile_max_force_n
            or right_force > self._tactile_max_force_n
        )

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

    def _wait_for_release_status(self) -> bool:
        deadline = time.monotonic() + self._contact_timeout_sec
        while True:
            if contact_event_matches(
                self._last_contact_status,
                CONTACT_RELEASED_PREFIX,
                target=None,
            ):
                return True
            if (
                self._last_hold_status.startswith("empty")
                or self._last_hold_status.startswith("lost ")
            ):
                return True
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                return False
            self._contact_status_event.wait(timeout=remaining)
            self._contact_status_event.clear()

    def _step_close_until_contact(self) -> bool:
        if self._enable_force_control:
            return self._step_close_with_force_control()
        return self._step_close_contact_only()

    def _step_close_contact_only(self) -> bool:
        self._last_left_contact_time = None
        self._last_right_contact_time = None
        left_position = self._tactile_start_position
        right_position = self._tactile_start_position
        max_position = self._tactile_max_position
        self._log(
            f"{self._tactile_log_label('config')} "
            f"target={self._target_object} "
            f"start={self._tactile_start_position:.6f} "
            f"step={self._tactile_step_position:.6f} "
            f"max={max_position:.6f} "
            f"urdf_upper={GRIPPER_URDF_UPPER_LIMIT:.6f} "
            f"enable_force_gate={str(self._enable_force_gate).lower()}"
        )
        while (
            left_position <= max_position + 1e-12
            and right_position <= max_position + 1e-12
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
                self._log_tactile_contact_reached(msg.data)
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
            if self._force_limit_exceeded(now):
                left_force, right_force = self._fresh_contact_forces(now)
                self._warn(
                    f"{self._tactile_log_label('force_limit')} "
                    f"left_force_n={left_force:.3f} right_force_n={right_force:.3f} "
                    f"max_force_n={self._tactile_max_force_n:.3f}"
                )
                return False
            self._log_tactile_close_step(
                msg.data,
                left_touching=left_touching,
                right_touching=right_touching,
            )
            if left_touching and right_touching and self._force_target_reached(now):
                self._log_tactile_contact_reached(msg.data)
                left_force, right_force = self._fresh_contact_forces(now)
                self._log(
                    f"夹爪力反馈闭合稳定 {self._target_object} "
                    f"left_force_n={left_force:.3f} right_force_n={right_force:.3f}"
                )
                return True
            if not left_touching:
                left_position += self._tactile_step_position
            if not right_touching:
                right_position += self._tactile_step_position
        self._log_tactile_limit_reached(
            [
                min(round(left_position, 4), max_position),
                min(round(right_position, 4), max_position),
            ]
        )
        self._warn(f"夹爪触觉闭合到上限仍未双指接触 {self._target_object}")
        return False

    def _step_close_with_force_control(self) -> bool:
        self._last_left_contact_time = None
        self._last_right_contact_time = None
        self._force_filter_samples.clear()
        left_position = self._tactile_start_position
        right_position = self._tactile_start_position
        max_position = self._tactile_max_position
        hold_frames = 0
        safety_frames = 0
        balance_frames = 0
        state = "POSITION_CLOSE"
        attached_force_deadline: float | None = None
        self._log(
            f"{self._tactile_log_label('config')} "
            f"target={self._target_object} "
            f"start={self._tactile_start_position:.6f} "
            f"step={self._tactile_step_position:.6f} "
            f"max={max_position:.6f} "
            f"enable_force_gate={str(self._enable_force_gate).lower()} "
            f"enable_force_control=true "
            f"force_target_n={self._force_target_n:.3f} "
            f"force_deadband_n={self._force_deadband_n:.3f} "
            f"force_kp={self._force_kp:.6f} "
            f"filter_window={self._force_filter_window}"
        )
        while (
            left_position <= max_position + 1e-12
            and right_position <= max_position + 1e-12
        ):
            command = [round(left_position, 4), round(right_position, 4)]
            msg = Float64MultiArray()
            msg.data = command
            self._command_pub.publish(msg)
            if self._tactile_dwell_sec > 0.0:
                time.sleep(self._tactile_dwell_sec)

            attached_now = contact_event_matches(
                self._last_contact_status,
                CONTACT_ATTACHED_PREFIX,
                target=self._target_object,
            )
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
            sample = self._fresh_force_sample(now)
            left_force_contact = (
                sample.source == FORCE_SOURCE_VIRTUAL_ESTIMATE
                and sample.left_force > 0.0
            )
            right_force_contact = (
                sample.source == FORCE_SOURCE_VIRTUAL_ESTIMATE
                and sample.right_force > 0.0
            )
            left_control_contact = left_touching or left_force_contact
            right_control_contact = right_touching or right_force_contact
            filtered_left = sample.left_force
            filtered_right = sample.right_force
            force_mean = 0.5 * (filtered_left + filtered_right)
            force_error = self._force_target_n - force_mean
            force_balance_error = abs(filtered_left - filtered_right)
            delta_q = 0.0

            if attached_now:
                if attached_force_deadline is None:
                    attached_force_deadline = now + TACTILE_FORCE_FRESH_SEC
                    self._force_filter_samples.clear()
                attached_force_ready = (
                    sample.source != FORCE_SOURCE_INVALID
                    and sample.left_force > 0.0
                    and sample.right_force > 0.0
                )
                if attached_force_ready:
                    filtered_left, filtered_right = self._filtered_forces(sample)
                    force_mean = 0.5 * (filtered_left + filtered_right)
                    force_error = self._force_target_n - force_mean
                    force_balance_error = abs(filtered_left - filtered_right)
                    self._publish_force_control_status(
                        "FORCE_REGULATION",
                        command,
                        sample,
                        filtered_left,
                        filtered_right,
                        0.0,
                    )
                    if (
                        force_error <= self._force_deadband_n
                        and force_balance_error <= self._force_balance_limit_n
                    ):
                        self._publish_force_control_status(
                            "FORCE_HOLD",
                            command,
                            sample,
                            filtered_left,
                            filtered_right,
                            0.0,
                        )
                        self._log_tactile_contact_reached(command)
                        self._log(f"夹爪触觉闭合已附着 {self._target_object}")
                        return True
                if now >= attached_force_deadline:
                    self._warn(
                        "夹爪已附着但力反馈样本未收敛: "
                        f"target={self._target_object} source={sample.source} "
                        f"left_force_n={sample.left_force:.3f} "
                        f"right_force_n={sample.right_force:.3f}"
                    )
                    self._log_tactile_contact_reached(command)
                    self._log(f"夹爪触觉闭合已附着 {self._target_object}")
                    return True
                self._publish_force_control_status(
                    "CONTACT_TRANSITION",
                    command,
                    sample,
                    filtered_left,
                    filtered_right,
                    0.0,
                )
                continue

            if left_control_contact and right_control_contact:
                if not sample.both_valid or sample.source == FORCE_SOURCE_INVALID:
                    self._publish_force_control_status(
                        "FORCE_SENSOR_INVALID",
                        command,
                        sample,
                        filtered_left,
                        filtered_right,
                        0.0,
                    )
                    self._warn(
                        "夹爪力反馈闭环无有效力样本 "
                        f"target={self._target_object} source={sample.source}"
                    )
                    return False
                filtered_left, filtered_right = self._filtered_forces(sample)
                force_mean = 0.5 * (filtered_left + filtered_right)
                force_error = self._force_target_n - force_mean
                force_balance_error = abs(filtered_left - filtered_right)
                state = "FORCE_REGULATION"
                force_safety_available = sample.source != FORCE_SOURCE_VIRTUAL_ESTIMATE
                if force_safety_available and (
                    sample.left_force > self._force_safety_limit_n
                    or sample.right_force > self._force_safety_limit_n
                ):
                    safety_frames += 1
                else:
                    safety_frames = 0
                if safety_frames >= self._force_safety_frames:
                    self._publish_force_control_status(
                        "FORCE_LIMIT",
                        command,
                        sample,
                        filtered_left,
                        filtered_right,
                        0.0,
                    )
                    self._warn(
                        f"夹爪力反馈安全上限触发 {self._target_object} "
                        f"left_force_n={sample.left_force:.3f} "
                        f"right_force_n={sample.right_force:.3f} "
                        f"limit_n={self._force_safety_limit_n:.3f}"
                    )
                    return False

                if force_balance_error > self._force_balance_limit_n:
                    balance_frames += 1
                    if balance_frames >= self._force_balance_frames:
                        self._publish_force_control_status(
                            "FORCE_UNBALANCED",
                            command,
                            sample,
                            filtered_left,
                            filtered_right,
                            0.0,
                        )
                        self._warn(
                            f"夹爪力反馈左右失衡 {self._target_object} "
                            f"left_force_n={filtered_left:.3f} "
                            f"right_force_n={filtered_right:.3f} "
                            f"balance_limit_n={self._force_balance_limit_n:.3f}"
                        )
                        return False
                else:
                    balance_frames = 0

                if (
                    force_error <= self._force_deadband_n
                    and force_balance_error <= self._force_balance_limit_n
                ):
                    hold_frames += 1
                    state = "FORCE_HOLD"
                    delta_q = 0.0
                    if hold_frames >= self._force_settle_frames:
                        self._publish_force_control_status(
                            state,
                            command,
                            sample,
                            filtered_left,
                            filtered_right,
                            delta_q,
                        )
                        self._log_tactile_contact_reached(command)
                        self._log(
                            f"夹爪力反馈闭环收敛 {self._target_object} "
                            f"mean_force_n={force_mean:.3f} "
                            f"target_force_n={self._force_target_n:.3f}"
                        )
                        if attached_now:
                            self._log(f"夹爪触觉闭合已附着 {self._target_object}")
                        return True
                else:
                    hold_frames = 0
                    raw_delta = self._force_kp * max(force_error, 0.0)
                    delta_q = min(raw_delta, self._force_max_close_step)
                    left_position = min(
                        left_position + delta_q,
                        max_position,
                    )
                    right_position = min(
                        right_position + delta_q,
                        max_position,
                    )
                self._publish_force_control_status(
                    state,
                    command,
                    sample,
                    filtered_left,
                    filtered_right,
                    delta_q,
                )
                continue

            hold_frames = 0
            safety_frames = 0
            balance_frames = 0
            self._force_filter_samples.clear()
            if left_control_contact or right_control_contact:
                state = "CONTACT_TRANSITION"
                step = min(self._tactile_step_position, self._force_max_close_step)
            else:
                state = "POSITION_CLOSE"
                step = self._tactile_step_position
            if not left_control_contact:
                left_position += step
            if not right_control_contact:
                right_position += step
            self._publish_force_control_status(
                state,
                command,
                sample,
                filtered_left,
                filtered_right,
                step,
            )
            self._log_tactile_close_step(
                command,
                left_touching=left_control_contact,
                right_touching=right_control_contact,
            )

        self._publish_force_control_status(
            "FORCE_LIMIT",
            [min(round(left_position, 4), max_position),
             min(round(right_position, 4), max_position)],
            self._fresh_force_sample(),
            self._last_left_force_n,
            self._last_right_force_n,
            0.0,
        )
        self._log_tactile_limit_reached(
            [
                min(round(left_position, 4), max_position),
                min(round(right_position, 4), max_position),
            ]
        )
        self._warn(f"夹爪力反馈闭环到上限仍未完成夹持 {self._target_object}")
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

    def _publish_positions(
        self,
        positions: list[float],
        settle_sec: float | None = None,
    ) -> None:
        msg = Float64MultiArray()
        msg.data = list(positions)
        self._command_pub.publish(msg)
        wait_sec = self._command_settle_sec if settle_sec is None else float(settle_sec)
        if wait_sec > 0.0:
            time.sleep(wait_sec)

    def _log(self, message: str) -> None:
        logger = getattr(self._node, "get_logger", lambda: None)()
        if logger is not None:
            logger.info(message)

    def _warn(self, message: str) -> None:
        logger = getattr(self._node, "get_logger", lambda: None)()
        if logger is not None:
            logger.warn(message)

    def _try_create_subscription(self, msg_type, topic, callback, qos):
        try:
            return self._node.create_subscription(msg_type, topic, callback, qos)
        except AssertionError:
            return None

    @staticmethod
    def _normalize_tactile_max_position(value: float | None) -> float:
        return ContactGripperDriver._normalize_tactile_position(
            value,
            TACTILE_MAX_POSITION,
        )

    @staticmethod
    def _normalize_tactile_step_position(value: float | None) -> float:
        if value is None:
            return float(TACTILE_STEP)
        step = float(value)
        if step <= 0.0:
            return float(TACTILE_STEP)
        return min(step, GRIPPER_URDF_UPPER_LIMIT)

    @staticmethod
    def _normalize_tactile_position(value: float | None, default: float) -> float:
        if value is None:
            return float(default)
        position = float(value)
        if position < 0.0:
            return float(default)
        return min(position, GRIPPER_URDF_UPPER_LIMIT)

    @staticmethod
    def _normalize_force_limit(value: float | None, default: float) -> float:
        if value is None:
            return float(default)
        force = float(value)
        if force <= 0.0:
            return float(default)
        return force

    @staticmethod
    def _normalize_gripper_step(value: float | None, default: float) -> float:
        if value is None:
            return float(default)
        step = float(value)
        if step <= 0.0:
            return float(default)
        return min(step, GRIPPER_URDF_UPPER_LIMIT)

    @staticmethod
    def _finite_nonnegative(value) -> float:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return 0.0
        if result != result or result == float("inf") or result == float("-inf"):
            return 0.0
        return max(0.0, result)

    @staticmethod
    def _normalize_finger_positions(
        positions: list[float] | tuple[float, float] | None,
    ) -> list[float]:
        if positions is None:
            return list(OPEN_POSITIONS)
        if len(positions) != 2:
            raise ValueError("open_positions must contain exactly two values")
        return [
            min(max(float(positions[0]), 0.0), GRIPPER_URDF_UPPER_LIMIT),
            min(max(float(positions[1]), 0.0), GRIPPER_URDF_UPPER_LIMIT),
        ]

    def _actual_finger_positions(self) -> tuple[float | None, float | None]:
        return self._last_left_joint_position, self._last_right_joint_position

    @staticmethod
    def _gripper_gap_mm(left_position: float, right_position: float) -> float:
        return (
            GRIPPER_OPEN_INNER_GAP_M
            - float(left_position)
            - float(right_position)
        ) * 1000.0

    def _estimated_gap_mm(self, command: list[float]) -> float:
        left_actual, right_actual = self._actual_finger_positions()
        if left_actual is not None and right_actual is not None:
            return self._gripper_gap_mm(left_actual, right_actual)
        return self._gripper_gap_mm(float(command[0]), float(command[1]))

    @staticmethod
    def _format_optional_float(value: float | None) -> str:
        if value is None:
            return "nan"
        return f"{float(value):.6f}"

    def _log_tactile_close_step(
        self,
        command: list[float],
        *,
        left_touching: bool,
        right_touching: bool,
    ) -> None:
        left_actual, right_actual = self._actual_finger_positions()
        self._log(
            f"{self._tactile_log_label('step')} "
            f"command={float(command[0]):.6f} "
            f"left_actual={self._format_optional_float(left_actual)} "
            f"right_actual={self._format_optional_float(right_actual)} "
                f"estimated_gap_mm={self._estimated_gap_mm(command):.2f} "
                f"left_contact={int(left_touching)} "
                f"right_contact={int(right_touching)} "
                f"left_force_n={self._last_left_force_n:.3f} "
                f"right_force_n={self._last_right_force_n:.3f} "
                f"target_force_n={self._tactile_target_force_n:.3f}"
            )

    def _log_tactile_limit_reached(self, command: list[float]) -> None:
        left_actual, right_actual = self._actual_finger_positions()
        self._warn(
            f"{self._tactile_log_label('limit')} "
            f"command={float(command[0]):.6f} "
            f"left_actual={self._format_optional_float(left_actual)} "
            f"right_actual={self._format_optional_float(right_actual)} "
            f"estimated_gap_mm={self._estimated_gap_mm(command):.2f}"
        )

    def _log_tactile_contact_reached(self, command: list[float]) -> None:
        left_actual, right_actual = self._actual_finger_positions()
        expected_width = (
            "nan"
            if self._expected_object_width_mm is None
            else f"{self._expected_object_width_mm:.2f}"
        )
        self._log(
            f"{self._tactile_log_label('contact')} "
            f"command={float(command[0]):.6f} "
            f"left_actual={self._format_optional_float(left_actual)} "
            f"right_actual={self._format_optional_float(right_actual)} "
            f"estimated_gap_mm={self._estimated_gap_mm(command):.2f} "
            f"expected_object_width_mm={expected_width} "
            f"left_force_n={self._last_left_force_n:.3f} "
            f"right_force_n={self._last_right_force_n:.3f}"
        )

    def _tactile_log_label(self, event: str) -> str:
        if self._tactile_log_prefix == "TUBE_TACTILE":
            return {
                "config": "TUBE_TACTILE_CONFIG",
                "step": "TUBE_TACTILE_CLOSE_STEP",
                "limit": "TUBE_TACTILE_LIMIT_REACHED",
                "contact": "TUBE_TACTILE_CONTACT_REACHED",
                "force_limit": "TUBE_TACTILE_FORCE_LIMIT_EXCEEDED",
            }[event]
        return {
            "config": f"{self._tactile_log_prefix}_CLOSE_CONFIG",
            "step": f"{self._tactile_log_prefix}_CLOSE_STEP",
            "limit": f"{self._tactile_log_prefix}_CLOSE_LIMIT_REACHED",
            "contact": f"{self._tactile_log_prefix}_CONTACT_REACHED",
            "force_limit": f"{self._tactile_log_prefix}_FORCE_LIMIT_EXCEEDED",
        }[event]


def make_gripper_driver(
    node,
    backend: str = CONTACT_BACKEND,
    command_settle_sec: float = 0.0,
    open_settle_sec: float | None = None,
    attach_timeout_sec: float = DEFAULT_ATTACH_TIMEOUT_SEC,
    target_object: str = DEFAULT_TARGET_OBJECT,
    use_tactile_grasp: bool = False,
    contact_timeout_sec: float = DEFAULT_CONTACT_TIMEOUT_SEC,
    tactile_start_position: float | None = None,
    tactile_step_position: float | None = None,
    tactile_max_position: float | None = None,
    tactile_log_prefix: str = "TACTILE",
    open_positions: list[float] | tuple[float, float] | None = None,
    expected_object_width_mm: float | None = None,
    tactile_target_force_n: float | None = None,
    tactile_max_force_n: float | None = None,
    enable_force_gate: bool = False,
    enable_force_control: bool = False,
    force_target_n: float = FORCE_CONTROL_TARGET_N,
    force_deadband_n: float = FORCE_CONTROL_DEADBAND_N,
    force_kp: float = FORCE_CONTROL_KP,
    force_max_close_step: float = FORCE_CONTROL_MAX_CLOSE_STEP,
    force_max_open_step: float = FORCE_CONTROL_MAX_OPEN_STEP,
    force_safety_limit_n: float = FORCE_CONTROL_SAFETY_LIMIT_N,
    force_safety_frames: int = FORCE_CONTROL_SAFETY_FRAMES,
    force_balance_limit_n: float = FORCE_CONTROL_BALANCE_LIMIT_N,
    force_balance_frames: int = FORCE_CONTROL_BALANCE_FRAMES,
    force_settle_frames: int = FORCE_CONTROL_SETTLE_FRAMES,
    force_filter_window: int = FORCE_CONTROL_FILTER_WINDOW,
) -> GripperDriver:
    normalized = str(backend).strip().lower()
    if normalized == SIM_ATTACH_BACKEND:
        if target_object != DEFAULT_TARGET_OBJECT:
            raise ValueError("sim_attach backend only supports default target_object")
        return SimAttachGripperDriver(
            node,
            command_settle_sec=command_settle_sec,
            open_settle_sec=open_settle_sec,
            attach_timeout_sec=attach_timeout_sec,
        )
    if normalized == CONTACT_BACKEND:
        return ContactGripperDriver(
            node,
            command_settle_sec=command_settle_sec,
            open_settle_sec=open_settle_sec,
            contact_timeout_sec=contact_timeout_sec,
            target_object=target_object,
            use_tactile_grasp=use_tactile_grasp,
            tactile_start_position=tactile_start_position,
            tactile_step_position=tactile_step_position,
            tactile_max_position=tactile_max_position,
            tactile_log_prefix=tactile_log_prefix,
            open_positions=open_positions,
            expected_object_width_mm=expected_object_width_mm,
            tactile_target_force_n=tactile_target_force_n,
            tactile_max_force_n=tactile_max_force_n,
            enable_force_gate=enable_force_gate,
            enable_force_control=enable_force_control,
            force_target_n=force_target_n,
            force_deadband_n=force_deadband_n,
            force_kp=force_kp,
            force_max_close_step=force_max_close_step,
            force_max_open_step=force_max_open_step,
            force_safety_limit_n=force_safety_limit_n,
            force_safety_frames=force_safety_frames,
            force_balance_limit_n=force_balance_limit_n,
            force_balance_frames=force_balance_frames,
            force_settle_frames=force_settle_frames,
            force_filter_window=force_filter_window,
        )
    raise ValueError(
        "unsupported gripper backend %r; expected %r or %r"
        % (backend, CONTACT_BACKEND, SIM_ATTACH_BACKEND)
    )
