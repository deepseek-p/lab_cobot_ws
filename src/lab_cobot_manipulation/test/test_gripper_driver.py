"""Unit contracts for the gripper backend boundary."""

import threading
import time

import pytest

from lab_cobot_manipulation import gripper_driver
from lab_cobot_manipulation.gripper_driver import (
    CLOSED_ON_SAMPLE_POSITIONS,
    ContactGripperDriver,
    DEFAULT_CONTACT_TIMEOUT_SEC,
    SimAttachGripperDriver,
    contact_event_matches,
    contact_object_name,
    contacts_msg_touches_object,
    make_gripper_driver,
)


CONTACT_STATUS_TOPIC = "/gripper/contact/status"
CONTACT_RELEASE_TOPIC = "/gripper/contact/release"
FINGERS_STATUS_TOPIC = "/gripper/contact/fingers"
LEFT_FINGER_CONTACTS_TOPIC = "/gripper/left_finger_contacts"
RIGHT_FINGER_CONTACTS_TOPIC = "/gripper/right_finger_contacts"


def test_closed_sample_positions_do_not_penetrate_70mm_sample():
    assert CLOSED_ON_SAMPLE_POSITIONS == [0.009, 0.009]


def _has_command(commands, expected):
    return any(command == pytest.approx(expected) for command in commands)


class FakePublisher:
    def __init__(self, topic, recorder):
        self.topic = topic
        self._recorder = recorder

    def publish(self, msg):
        self._recorder(self.topic, msg)


class FakeNode:
    def __init__(
        self,
        attach_status=None,
        contact_status_on_close=None,
        contact_status_on_command=None,
        contact_status_on_release=None,
        left_contact_on_command=None,
        right_contact_on_command=None,
        emit_legacy_close_status=True,
    ):
        self.float_arrays = []
        self.empty_topics = []
        self.logs = []
        self._attach_status = attach_status
        self._contact_status_on_close = contact_status_on_close
        self._contact_status_on_command = contact_status_on_command
        self._contact_status_on_release = contact_status_on_release
        self._left_contact_on_command = left_contact_on_command
        self._right_contact_on_command = right_contact_on_command
        self._emit_legacy_close_status = emit_legacy_close_status
        self._status_callback = None
        self._contact_status_callback = None
        self._left_contact_callback = None
        self._right_contact_callback = None

    def create_publisher(self, msg_type, topic, _qos):
        if msg_type.__name__ == "Float64MultiArray":
            return FakePublisher(topic, self._record_float_array)
        return FakePublisher(topic, self._record_empty)

    def create_subscription(self, msg_type, topic, callback, _qos):
        if topic == "/gripper/attach/status":
            assert msg_type.__name__ == "String"
            self._status_callback = callback
        elif topic == CONTACT_STATUS_TOPIC:
            assert msg_type.__name__ == "String"
            self._contact_status_callback = callback
        elif topic == FINGERS_STATUS_TOPIC:
            assert msg_type.__name__ == "String"
            self._fingers_status_callback = callback
        elif topic == LEFT_FINGER_CONTACTS_TOPIC:
            assert msg_type.__name__ == "ContactsState"
            self._left_contact_callback = callback
        elif topic == RIGHT_FINGER_CONTACTS_TOPIC:
            assert msg_type.__name__ == "ContactsState"
            self._right_contact_callback = callback
        else:
            raise AssertionError(topic)
        return object()

    def _record_float_array(self, topic, msg):
        self.float_arrays.append(list(msg.data))
        if topic == "/gripper_position_controller/commands":
            left_contact = self._contact_message(self._left_contact_on_command, msg)
            right_contact = self._contact_message(self._right_contact_on_command, msg)
            if left_contact:
                self._left_contact_callback(left_contact)
            if right_contact:
                self._right_contact_callback(right_contact)
            if (
                left_contact
                and right_contact
                and self._contact_status_on_close
            ):
                self._emit_contact_status(self._contact_status_on_close)
            if self._contact_status_on_command:
                status = self._contact_status_on_command
                if callable(status):
                    status = status(list(msg.data))
                if status:
                    self._emit_contact_status(status)
        if (
            topic == "/gripper_position_controller/commands"
            and self._emit_legacy_close_status
            and list(msg.data) == CLOSED_ON_SAMPLE_POSITIONS
            and self._contact_status_on_close
        ):
            self._emit_contact_status(self._contact_status_on_close)

    def _record_empty(self, topic, _msg):
        self.empty_topics.append(topic)
        if topic == "/gripper/attach/aruco_sample" and self._attach_status:
            status_msg = type("Msg", (), {})()
            status_msg.data = self._attach_status
            self._status_callback(status_msg)
        if topic == CONTACT_RELEASE_TOPIC and self._contact_status_on_release:
            if isinstance(self._contact_status_on_release, list):
                statuses = list(self._contact_status_on_release)

                def emit_sequence():
                    for status in statuses:
                        time.sleep(0.01)
                        self._emit_contact_status(status)

                threading.Thread(target=emit_sequence, daemon=True).start()
                return
            self._emit_contact_status(self._contact_status_on_release)

    def _contact_message(self, source, msg):
        if callable(source):
            return source(list(msg.data))
        return source

    def _emit_contact_status(self, status):
        status_msg = type("Msg", (), {})()
        status_msg.data = status
        self._contact_status_callback(status_msg)

    def get_logger(self):
        return self

    def info(self, msg):
        self.logs.append(("info", msg))

    def warn(self, msg):
        self.logs.append(("warn", msg))


class FakeContactState:
    def __init__(self, collision1_name, collision2_name):
        self.collision1_name = collision1_name
        self.collision2_name = collision2_name


class FakeContactsState:
    def __init__(self, pairs):
        self.states = [FakeContactState(left, right) for left, right in pairs]


def test_sim_attach_driver_publishes_open_close_and_detach_topics():
    fake_node = FakeNode()
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert driver.open()
    assert driver.close()
    assert driver.release_object()
    assert fake_node.float_arrays[0] == [0.0, 0.0]
    assert fake_node.float_arrays[1] == CLOSED_ON_SAMPLE_POSITIONS
    assert "/gripper/detach/aruco_sample" in fake_node.empty_topics


def test_acquire_object_returns_true_when_bridge_accepts_attach():
    fake_node = FakeNode(attach_status="attached aruco_sample")
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert driver.acquire_object()
    assert "/gripper/attach/aruco_sample" in fake_node.empty_topics


def test_acquire_object_returns_false_when_bridge_refuses_attach():
    fake_node = FakeNode(
        attach_status="refused aruco_sample object_outside_finger_gap"
    )
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert not driver.acquire_object()
    assert any("refused" in message for _level, message in fake_node.logs)


def test_acquire_object_returns_false_when_bridge_status_times_out():
    fake_node = FakeNode()
    driver = SimAttachGripperDriver(fake_node, attach_timeout_sec=0.0)

    assert not driver.acquire_object()
    assert any("timed out" in message for _level, message in fake_node.logs)


def test_contact_gripper_driver_uses_only_finger_commands_without_attach_topics():
    fake_node = FakeNode()
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.0)

    assert driver.open()
    assert driver.close()
    assert not driver.acquire_object()
    assert not driver.release_object()
    assert fake_node.float_arrays == [
        [0.0, 0.0],
        CLOSED_ON_SAMPLE_POSITIONS,
        CLOSED_ON_SAMPLE_POSITIONS,
    ]
    legacy_topics = {"/gripper/attach/aruco_sample", "/gripper/detach/aruco_sample"}
    assert not legacy_topics.intersection(fake_node.empty_topics)


def test_contact_gripper_acquire_closes_and_waits_for_plugin_attached_status():
    fake_node = FakeNode(contact_status_on_close="attached aruco_sample")
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.0)

    assert driver.acquire_object()
    assert fake_node.float_arrays == [CLOSED_ON_SAMPLE_POSITIONS]
    assert fake_node.empty_topics == []


def test_contact_gripper_close_after_acquire_is_idempotent():
    fake_node = FakeNode(contact_status_on_close="attached aruco_sample")
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.0)

    assert driver.acquire_object()
    assert driver.close()
    assert fake_node.float_arrays == [CLOSED_ON_SAMPLE_POSITIONS]


def test_contact_gripper_acquire_fails_without_plugin_attached_status():
    fake_node = FakeNode()
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.0)

    assert not driver.acquire_object()
    assert fake_node.float_arrays == [CLOSED_ON_SAMPLE_POSITIONS]
    assert any("contact attach timed out" in message for _level, message in fake_node.logs)


def test_contact_gripper_falls_back_to_attach_bridge_after_tactile_timeout():
    fake_node = FakeNode(
        attach_status="attached aruco_sample",
        left_contact_on_command=FakeContactsState([
            ("aruco_sample::link::collision", "left_finger::collision")
        ]),
        right_contact_on_command=FakeContactsState([
            ("aruco_sample::link::collision", "right_finger::collision")
        ]),
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
        enable_attach_fallback=True,
        attach_timeout_sec=0.0,
    )

    assert driver.acquire_object()
    assert "/gripper/attach/aruco_sample" in fake_node.empty_topics
    assert any("falling back to attach bridge" in message for _level, message in fake_node.logs)


def test_contact_gripper_acquire_fails_fast_when_plugin_refuses_grasp():
    fake_node = FakeNode(
        contact_status_on_close="refused aruco_sample offset=(0.080,0.000,0.006)"
    )
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=5.0)

    assert not driver.acquire_object()
    assert any("refused aruco_sample" in message for _level, message in fake_node.logs)


def test_contact_gripper_release_requests_plugin_detach_and_waits_for_status():
    fake_node = FakeNode(contact_status_on_release="released aruco_sample")
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.0)

    assert driver.release_object()
    assert CONTACT_RELEASE_TOPIC in fake_node.empty_topics


def test_contact_gripper_release_ignores_refused_statuses_until_release_arrives():
    fake_node = FakeNode(
        contact_status_on_release=[
            "refused aruco_sample offset=(1.523,-0.926,0.600)",
            "released aruco_sample",
        ],
    )
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.2)

    assert driver.release_object()
    assert CONTACT_RELEASE_TOPIC in fake_node.empty_topics


def test_make_gripper_driver_defaults_to_contact_backend():
    fake_node = FakeNode()
    driver = make_gripper_driver(fake_node, command_settle_sec=0.0)

    assert isinstance(driver, ContactGripperDriver)


def test_make_contact_gripper_driver_uses_contact_timeout_default():
    fake_node = FakeNode()

    driver = make_gripper_driver(fake_node, command_settle_sec=0.0)

    assert driver._contact_timeout_sec == DEFAULT_CONTACT_TIMEOUT_SEC


def test_contact_status_helpers_parse_object_names():
    assert contact_object_name("attached aruco_sample") == "aruco_sample"
    assert contact_object_name("refused reagent_bottle offset=(0,0,0)") == (
        "reagent_bottle"
    )
    assert contact_event_matches(
        "attached reagent_bottle",
        "attached ",
        "reagent_bottle",
    )
    assert not contact_event_matches(
        "attached reagent_bottle",
        "attached ",
        "aruco_sample",
    )


def test_contact_gripper_defensively_releases_when_wrong_object_attaches():
    fake_node = FakeNode(contact_status_on_close="attached reagent_bottle")
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        target_object="aruco_sample",
    )

    assert not driver.acquire_object()
    assert CONTACT_RELEASE_TOPIC in fake_node.empty_topics
    assert any("reagent_bottle" in message for _level, message in fake_node.logs)


def test_contact_gripper_can_target_non_default_object():
    fake_node = FakeNode(contact_status_on_close="attached reagent_bottle")
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        target_object="reagent_bottle",
    )

    assert driver.acquire_object()
    assert fake_node.float_arrays == [CLOSED_ON_SAMPLE_POSITIONS]


def test_contact_gripper_release_accepts_released_none_status():
    fake_node = FakeNode(contact_status_on_release="released none")
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=0.0)

    assert driver.release_object()
    assert CONTACT_RELEASE_TOPIC in fake_node.empty_topics


def test_contact_gripper_fails_fast_when_any_candidate_is_refused():
    fake_node = FakeNode(contact_status_on_close="refused reagent_bottle static_model")
    driver = ContactGripperDriver(fake_node, contact_timeout_sec=5.0)

    started = time.monotonic()
    assert not driver.acquire_object()
    assert time.monotonic() - started < 1.0
    assert any("refused reagent_bottle" in message for _level, message in fake_node.logs)


def test_make_gripper_driver_forwards_target_object_to_contact_backend():
    fake_node = FakeNode(contact_status_on_close="attached reagent_bottle")
    driver = make_gripper_driver(
        fake_node,
        command_settle_sec=0.0,
        target_object="reagent_bottle",
    )

    assert driver.acquire_object()


def test_sim_attach_backend_rejects_non_default_target_object():
    fake_node = FakeNode()

    try:
        make_gripper_driver(fake_node, backend="sim_attach", target_object="other")
    except ValueError as exc:
        assert "target_object" in str(exc)
    else:
        raise AssertionError("sim_attach accepted a non-default target object")


def test_contacts_msg_touches_object_matches_gazebo_collision_format():
    msg = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])

    assert contacts_msg_touches_object(msg, prefix="aruco_sample::")
    assert not contacts_msg_touches_object(msg, prefix="reagent_bottle::")


def test_tactile_acquire_steps_until_both_fingers_touch_target():
    contact = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])
    fake_node = FakeNode(
        contact_status_on_close="attached aruco_sample",
        left_contact_on_command=contact,
        right_contact_on_command=contact,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
    )

    assert driver.acquire_object()
    assert fake_node.float_arrays[0] == [0.006, 0.006]
    assert fake_node.float_arrays[-1] == [0.006, 0.006]


def test_tactile_acquire_accepts_fresh_dual_contact_without_position_gate():
    contact = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])

    def left_contact(command):
        if command == pytest.approx([0.006, 0.006]):
            return contact
        return None

    def right_contact(command):
        if command == pytest.approx([0.006, 0.006]):
            return contact
        return None

    def contact_status(command):
        if command == pytest.approx([0.006, 0.006]):
            return "attached aruco_sample"
        return None

    fake_node = FakeNode(
        contact_status_on_command=contact_status,
        left_contact_on_command=left_contact,
        right_contact_on_command=right_contact,
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
    )

    assert driver.acquire_object()
    assert fake_node.float_arrays == [[0.006, 0.006]]


def test_tactile_step_close_rejects_stale_split_contact(monkeypatch):
    contact = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])
    clock = {"now": 0.0}

    def monotonic():
        return clock["now"]

    def sleep(duration):
        clock["now"] += duration

    monkeypatch.setattr(gripper_driver.time, "monotonic", monotonic)
    monkeypatch.setattr(gripper_driver.time, "sleep", sleep)

    def left_contact(command):
        if command == pytest.approx([0.0105, 0.0105]):
            return contact
        return None

    def right_contact(command):
        if command == pytest.approx([0.011, 0.0125]):
            return contact
        return None

    fake_node = FakeNode(
        left_contact_on_command=left_contact,
        right_contact_on_command=right_contact,
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.1,
    )

    assert not driver._step_close_until_contact()
    assert driver.last_tactile_contact_sides() != (True, True)


def test_tactile_step_close_holds_touched_finger_and_continues_other_side():
    contact = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])

    def left_contact(command):
        if command[0] >= 0.006:
            return contact
        return None

    def right_contact(command):
        if command[1] >= 0.011:
            return contact
        return None

    fake_node = FakeNode(
        left_contact_on_command=left_contact,
        right_contact_on_command=right_contact,
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
    )

    assert driver._step_close_until_contact()
    assert driver.last_tactile_contact_sides() == (True, True)
    assert fake_node.float_arrays == [
        [0.006, 0.006],
        [0.006, 0.0065],
        [0.006, 0.007],
        [0.006, 0.0075],
        [0.006, 0.008],
        [0.006, 0.0085],
        [0.006, 0.009],
        [0.006, 0.0095],
        [0.006, 0.01],
        [0.006, 0.0105],
        [0.006, 0.011],
    ]


def test_tactile_acquire_stops_closing_when_plugin_attaches():
    contact = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])

    def left_contact(command):
        if command[0] >= 0.006:
            return contact
        return None

    fake_node = FakeNode(
        contact_status_on_command="attached aruco_sample",
        left_contact_on_command=left_contact,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
    )

    assert driver.acquire_object()
    assert fake_node.float_arrays == [[0.006, 0.006]]


def test_tactile_step_close_ignores_contact_gate_until_limit():
    def contact_status(command):
        if command == pytest.approx([0.006, 0.006]):
            return "refused aruco_sample no_finger_contact"
        return None

    fake_node = FakeNode(
        contact_status_on_command=contact_status,
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
    )

    assert not driver._step_close_until_contact()
    assert fake_node.float_arrays[0] == [0.006, 0.006]
    assert fake_node.float_arrays[-1] == [0.0185, 0.0185]
    assert any("仍未双指接触" in message for _level, message in fake_node.logs)


def test_tactile_step_close_stops_when_plugin_refuses_geometry():
    def contact_status(command):
        if command == pytest.approx([0.006, 0.006]):
            return "refused aruco_sample offset=(0.080,0.000,0.006)"
        return None

    fake_node = FakeNode(
        contact_status_on_command=contact_status,
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
    )

    assert not driver._step_close_until_contact()
    assert fake_node.float_arrays == [[0.006, 0.006]]
    assert any("refused" in message for _level, message in fake_node.logs)


def test_tactile_acquire_waits_past_stale_no_contact_refusal():
    contact = FakeContactsState([
        ("lab_cobot::gripper_left_finger::gripper_left_finger_collision",
         "aruco_sample::link::collision"),
    ])

    def contact_status(command):
        if command == pytest.approx([0.006, 0.006]):
            return "refused aruco_sample no_finger_contact"
        if command == pytest.approx([0.0065, 0.0065]):
            return "attached aruco_sample"
        return None

    def contact_at_second_step(command):
        if command == pytest.approx([0.0065, 0.0065]):
            return contact
        return None

    fake_node = FakeNode(
        contact_status_on_command=contact_status,
        left_contact_on_command=contact_at_second_step,
        right_contact_on_command=contact_at_second_step,
        emit_legacy_close_status=False,
    )
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
        tactile_dwell_sec=0.0,
    )

    assert driver.acquire_object()
    assert fake_node.float_arrays == [[0.006, 0.006], [0.0065, 0.0065]]


def test_tactile_acquire_fails_at_limit_without_dual_contact():
    fake_node = FakeNode()
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
    )

    assert not driver.acquire_object()
    assert fake_node.float_arrays[0] == [0.006, 0.006]
    assert fake_node.float_arrays[-1] == [0.0185, 0.0185]


def test_tactile_step_close_returns_false_after_full_sweep_without_dual_contact():
    fake_node = FakeNode()
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=True,
    )

    assert not driver._step_close_until_contact()
    assert fake_node.float_arrays[0] == [0.006, 0.006]
    assert fake_node.float_arrays[-1] == [0.0185, 0.0185]
    assert any("仍未双指接触" in message for _level, message in fake_node.logs)


def test_non_tactile_acquire_keeps_legacy_close_command_sequence():
    fake_node = FakeNode(contact_status_on_close="attached aruco_sample")
    driver = ContactGripperDriver(
        fake_node,
        contact_timeout_sec=0.0,
        use_tactile_grasp=False,
    )

    assert driver.acquire_object()
    assert fake_node.float_arrays == [CLOSED_ON_SAMPLE_POSITIONS]
