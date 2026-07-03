"""Pure sequencing tests for PickPlace gripper workflows."""

import pytest

from lab_cobot_manipulation import pick_place_node
from lab_cobot_manipulation.pick_place_node import (
    DEFAULT_APPROACH_HEIGHT,
    GRIPPER_CLOSE_SETTLE_SEC,
    PickPlace,
)


class FakeGripper:
    def __init__(
        self,
        events,
        open_ok=True,
        close_ok=True,
        acquire_ok=True,
        release_ok=True,
    ):
        self._events = events
        self._open_ok = open_ok
        self._close_ok = close_ok
        self._acquire_ok = acquire_ok
        self._release_ok = release_ok

    def open(self):
        self._events.append("open")
        return self._open_ok

    def close(self):
        self._events.append("close")
        return self._close_ok

    def acquire_object(self):
        self._events.append("acquire")
        return self._acquire_ok

    def release_object(self):
        self._events.append("release")
        return self._release_ok


def make_pick_place_without_ros(
    fake_moves,
    open_ok=True,
    close_ok=True,
    acquire_ok=True,
    release_ok=True,
):
    pick_place = PickPlace.__new__(PickPlace)
    pick_place.approach_height = DEFAULT_APPROACH_HEIGHT
    pick_place.events = []
    pick_place.move_positions = []
    pick_place.move_kwargs = []
    pick_place.gripper = FakeGripper(
        pick_place.events,
        open_ok=open_ok,
        close_ok=close_ok,
        acquire_ok=acquire_ok,
        release_ok=release_ok,
    )
    move_results = iter(fake_moves)
    move_names = iter(["move_above", "move_grasp", "move_above"])

    def fake_move(pos, quat=None, frame_id="base_link", **kwargs):
        pick_place.events.append(next(move_names))
        pick_place.move_positions.append(list(pos))
        pick_place.move_kwargs.append(kwargs)
        return next(move_results)

    pick_place._move = fake_move
    return pick_place


def assert_positions_close(actual, expected):
    assert len(actual) == len(expected)
    for actual_pos, expected_pos in zip(actual, expected):
        assert actual_pos == pytest.approx(expected_pos)


def test_pick_sequence_validates_attachment_before_closing_and_lifting():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert pick_place.events == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "close",
        "move_above",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.86],
        [0.8, 0.0, 0.80],
        [0.8, 0.0, 0.86],
    ])


def test_pick_targets_gripper_tcp_directly_to_avoid_tool0_tilt_offsets():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert all(
        kwargs.get("target_link") == "gripper_tcp"
        for kwargs in pick_place.move_kwargs
    )
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.86],
        [0.8, 0.0, 0.80],
        [0.8, 0.0, 0.86],
    ])


def test_pick_relaxes_approach_but_keeps_grasp_orientation_tight():
    assert hasattr(pick_place_node, "DEFAULT_GRASP_TOLERANCE_ORIENTATION")
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[0].get("tolerance_orientation") == pytest.approx(0.2)
    assert "tolerance_position" not in pick_place.move_kwargs[1]
    assert pick_place.move_kwargs[1].get("tolerance_orientation") == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION
    )
    assert pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION <= 0.05
    assert pick_place.move_kwargs[2].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[2].get("tolerance_orientation") == pytest.approx(0.2)


def test_pick_moves_use_bounded_moveit_waits():
    assert hasattr(pick_place_node, "DEFAULT_MOVE_TIMEOUT_SEC")
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert all(
        kwargs.get("timeout_sec")
        == pytest.approx(pick_place_node.DEFAULT_MOVE_TIMEOUT_SEC)
        for kwargs in pick_place.move_kwargs
    )


def test_place_sequence_moves_releases_opens_and_lifts():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.place([0.8, 0.0, 0.78])
    assert pick_place.events == [
        "move_above",
        "move_grasp",
        "release",
        "open",
        "move_above",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.78],
        [0.8, 0.0, 0.84],
    ])


def test_place_relaxes_orientation_for_approach_descent_and_lift():
    assert hasattr(pick_place_node, "DEFAULT_GRASP_TOLERANCE_ORIENTATION")
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.place([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[0].get("tolerance_orientation") == pytest.approx(0.2)
    assert "tolerance_position" not in pick_place.move_kwargs[1]
    assert pick_place.move_kwargs[1].get("tolerance_orientation") == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION
    )
    assert pick_place.move_kwargs[2].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[2].get("tolerance_orientation") == pytest.approx(0.2)


def test_default_approach_height_stays_within_reachable_gripper_band():
    assert DEFAULT_APPROACH_HEIGHT == pytest.approx(0.06)


def test_gripper_close_waits_for_visible_motion():
    assert GRIPPER_CLOSE_SETTLE_SEC >= 0.8


def test_pick_stops_before_lift_when_attach_is_refused():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True],
        acquire_ok=False,
    )

    assert not pick_place.pick([0.8, 0.0, 0.78])
    assert pick_place.events == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.86],
        [0.8, 0.0, 0.80],
    ])


def test_moveit_settings_are_tuned_for_fast_reliable_pick_place():
    assert hasattr(pick_place_node, "configure_moveit_for_pick_place")
    assert hasattr(pick_place_node, "ARM_ALLOWED_PLANNING_TIME_SEC")
    assert hasattr(pick_place_node, "ARM_MAX_ACCELERATION_SCALING")
    assert hasattr(pick_place_node, "ARM_NUM_PLANNING_ATTEMPTS")
    assert hasattr(pick_place_node, "ARM_MAX_VELOCITY_SCALING")

    class FakeMoveIt:
        max_velocity = None
        max_acceleration = None
        allowed_planning_time = None
        num_planning_attempts = None

    fake = FakeMoveIt()

    pick_place_node.configure_moveit_for_pick_place(fake)

    assert fake.max_velocity == pytest.approx(
        pick_place_node.ARM_MAX_VELOCITY_SCALING
    )
    assert fake.max_acceleration == pytest.approx(
        pick_place_node.ARM_MAX_ACCELERATION_SCALING
    )
    assert fake.allowed_planning_time == pytest.approx(
        pick_place_node.ARM_ALLOWED_PLANNING_TIME_SEC
    )
    assert fake.num_planning_attempts == pick_place_node.ARM_NUM_PLANNING_ATTEMPTS
    assert pick_place_node.ARM_MAX_VELOCITY_SCALING >= 0.7
    assert pick_place_node.ARM_MAX_ACCELERATION_SCALING >= 0.7
    assert 2.0 <= pick_place_node.ARM_ALLOWED_PLANNING_TIME_SEC <= 3.0
    assert 3 <= pick_place_node.ARM_NUM_PLANNING_ATTEMPTS <= 5


def test_go_home_retries_transient_invalid_moveit_execution():
    pick_place = PickPlace.__new__(PickPlace)
    configs = []

    class FakeMoveIt:
        def __init__(self):
            self.results = iter([False, True])

        def move_to_configuration(self, config):
            configs.append(list(config))

        def wait_until_executed(self, timeout_sec=None):
            return next(self.results)

    pick_place.moveit2 = FakeMoveIt()

    assert pick_place.go_home()
    assert len(configs) == 2
    assert all(config == pick_place_node.HOME_CONFIG for config in configs)
