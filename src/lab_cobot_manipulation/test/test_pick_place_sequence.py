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
        contact_sides=(False, False),
        holding_ok=True,
    ):
        self._events = events
        self._open_ok = open_ok
        self._close_ok = close_ok
        if isinstance(acquire_ok, list):
            self._acquire_results = iter(acquire_ok)
            self._acquire_ok = True
        else:
            self._acquire_results = None
            self._acquire_ok = acquire_ok
        self._release_ok = release_ok
        self._contact_sides = contact_sides
        self._holding_ok = holding_ok

    def open(self):
        self._events.append("open")
        return self._open_ok

    def close(self):
        self._events.append("close")
        return self._close_ok

    def acquire_object(self):
        self._events.append("acquire")
        if self._acquire_results is not None:
            return next(self._acquire_results)
        return self._acquire_ok

    def release_object(self):
        self._events.append("release")
        return self._release_ok

    def last_tactile_contact_sides(self):
        return self._contact_sides

    def is_holding_object(self):
        return self._holding_ok


class FakeLogger:
    def __init__(self, events):
        self._events = events

    def info(self, message):
        self._events.append(f"log_info:{message}")

    def warn(self, message):
        self._events.append(f"log_warn:{message}")

    def error(self, message):
        self._events.append(f"log_error:{message}")


class FakeSceneClient:
    """Record planning scene diffs as sequence events."""

    def __init__(self, events, ok=True):
        self._events = events
        self._ok = ok

    def apply(self, scene, **kwargs):
        attached = scene.robot_state.attached_collision_objects
        if attached:
            operation = attached[0].object.operation
            if operation == attached[0].object.ADD:
                self._events.append("scene_attach")
            else:
                self._events.append("scene_detach")
        elif scene.world.collision_objects:
            obj = scene.world.collision_objects[0]
            if obj.id == pick_place_node.DYNAMIC_ARM_OBSTACLE_BOX_ID:
                if obj.operation == obj.ADD:
                    self._events.append("scene_dynamic_update")
                else:
                    self._events.append("scene_dynamic_remove")
            else:
                self._events.append("scene_surface")
        return self._ok


def make_pick_place_without_ros(
    fake_moves,
    open_ok=True,
    close_ok=True,
    acquire_ok=True,
    release_ok=True,
    use_tactile_grasp=False,
    contact_sides=(False, False),
    holding_ok=True,
    move_names=None,
):
    pick_place = PickPlace.__new__(PickPlace)
    pick_place.approach_height = DEFAULT_APPROACH_HEIGHT
    pick_place.events = []
    pick_place.move_positions = []
    pick_place.move_kwargs = []
    pick_place.gripper_backend = "test"
    pick_place.use_tactile_grasp = use_tactile_grasp
    pick_place.scene_client = None
    pick_place.get_logger = lambda: FakeLogger(pick_place.events)
    pick_place.gripper = FakeGripper(
        pick_place.events,
        open_ok=open_ok,
        close_ok=close_ok,
        acquire_ok=acquire_ok,
        release_ok=release_ok,
        contact_sides=contact_sides,
        holding_ok=holding_ok,
    )
    move_results = iter(fake_moves)
    if move_names is None:
        move_names = [
            "move_above",
            "move_grasp",
            "move_above",
            "move_above_retry",
            "move_grasp_retry",
            "move_above_retry",
            "move_grasp_retry",
            "move_above_retry",
            "move_grasp_retry",
            "move_above_retry",
        ]
    move_names = iter(move_names)

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


def action_events(events):
    return [event for event in events if not event.startswith("log_")]


def test_pick_sequence_validates_attachment_before_closing_and_lifting():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "close",
        "move_above",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.88],
    ])


def test_pick_stops_when_plugin_does_not_confirm_held_object():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True], holding_ok=False
    )

    assert not pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
    ]
    assert any("持有监控失败" in event for event in pick_place.events)


def test_pick_targets_gripper_tcp_directly_to_avoid_tool0_tilt_offsets():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert all(
        kwargs.get("target_link") == "gripper_tcp"
        for kwargs in pick_place.move_kwargs
    )
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.88],
    ])


def test_pick_uses_vertical_approach_for_outer_visual_detection_pose():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.812, 0.042, 0.725])
    assert_positions_close(pick_place.move_positions, [
        [0.812, 0.042, 0.825],
        [0.812, 0.042, 0.785],
        [0.812, 0.042, 0.825],
    ])


def test_pick_refine_callback_recomputes_descent_target_after_approach():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])
    target_inputs = []
    original_pick_tcp_target = pick_place._pick_tcp_target

    def record_pick_tcp_target(pos):
        target_inputs.append(list(pos))
        return original_pick_tcp_target(pos)

    pick_place._pick_tcp_target = record_pick_tcp_target

    assert pick_place.pick(
        [0.8, 0.0, 0.78],
        refine_cb=lambda: [0.82, -0.01, 0.79],
    )

    assert target_inputs[-1] == [0.82, -0.01, 0.79]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.82, -0.01, 0.85],
        [0.82, -0.01, 0.89],
    ])


def test_pick_refine_callback_none_keeps_coarse_descent_target():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])
    target_inputs = []
    original_pick_tcp_target = pick_place._pick_tcp_target

    def record_pick_tcp_target(pos):
        target_inputs.append(list(pos))
        return original_pick_tcp_target(pos)

    pick_place._pick_tcp_target = record_pick_tcp_target

    assert pick_place.pick(
        [0.8, 0.0, 0.78],
        refine_cb=lambda: None,
    )

    assert target_inputs[-1] == [0.8, 0.0, 0.78]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.88],
    ])


def test_pick_refine_callback_exception_degrades_to_coarse_target():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    def raise_refine_error():
        raise RuntimeError("wrist camera unavailable")

    assert pick_place.pick(
        [0.8, 0.0, 0.78],
        refine_cb=raise_refine_error,
    )

    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.88],
    ])
    assert "log_info:refine=miss(callback_exception)" in pick_place.events


def test_tactile_pick_keeps_visual_lateral_target_inside_safe_band():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.006, 0.8525],
        [0.8, 0.006, 0.7925],
        [0.8, 0.006, 0.8525],
    ])


def test_tactile_pick_preserves_negative_visual_lateral_residual_inside_safe_band():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.pick([0.8, -0.006, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.8525],
        [0.8, 0.0, 0.7925],
        [0.8, 0.0, 0.8525],
    ])


def test_tactile_pick_preserves_positive_visual_lateral_residual_inside_safe_band():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.pick([0.8, 0.012, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.018, 0.8525],
        [0.8, 0.018, 0.7925],
        [0.8, 0.018, 0.8525],
    ])


def test_tactile_pick_clamps_large_visual_lateral_residuals_on_first_attempt():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.pick([0.8, 0.030, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.018, 0.8525],
        [0.8, 0.018, 0.7925],
        [0.8, 0.018, 0.8525],
    ])


def test_tactile_pick_targets_deep_grasp_clearance():
    """Tactile pick should place the sample near the finger middle."""
    # DG v1.1 静态预算:0.0125m 理论重叠 34.5mm,
    # 同时在最坏 -20mm 散布下仍保留约 11.5mm 指尖台面间隙。
    assert pick_place_node.TACTILE_PICK_TCP_Z_CLEARANCE == pytest.approx(0.0125)


def test_tactile_pick_retries_laterally_after_left_only_contact_failure():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True, True, True, True],
        acquire_ok=[False, True],
        use_tactile_grasp=True,
        contact_sides=(True, False),
    )

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert [
        event for event in action_events(pick_place.events)
        if event in ("open", "acquire", "close")
    ] == [
        "open",
        "acquire",
        "open",
        "open",
        "acquire",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.006, 0.8525],
        [0.8, 0.006, 0.7925],
        [0.8, 0.006, 0.8525],
        [0.8, 0.0, 0.8525],
        [0.8, 0.0, 0.7925],
        [0.8, 0.0, 0.8525],
    ])


def test_tactile_pick_retries_laterally_after_right_only_contact_failure():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True, True, True, True],
        acquire_ok=[False, True],
        use_tactile_grasp=True,
        contact_sides=(False, True),
    )

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.006, 0.8525],
        [0.8, 0.006, 0.7925],
        [0.8, 0.006, 0.8525],
        [0.8, 0.012, 0.8525],
        [0.8, 0.012, 0.7925],
        [0.8, 0.012, 0.8525],
    ])


def test_tactile_pick_continues_retries_away_from_left_only_contact():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True, True, True, True, True, True, True],
        acquire_ok=[False, False, True],
        use_tactile_grasp=True,
        contact_sides=(True, False),
    )

    assert pick_place.pick([0.8, -0.030, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, -0.018, 0.8525],
        [0.8, -0.018, 0.7925],
        [0.8, -0.018, 0.8525],
        [0.8, -0.024, 0.8525],
        [0.8, -0.024, 0.7925],
        [0.8, -0.024, 0.8525],
        [0.8, -0.030, 0.8525],
        [0.8, -0.030, 0.7925],
        [0.8, -0.030, 0.8525],
    ])


def test_tactile_pick_retries_away_from_right_only_contact():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True, True, True, True],
        acquire_ok=[False, True],
        use_tactile_grasp=True,
        contact_sides=(False, True),
    )

    assert pick_place.pick([0.8, 0.030, 0.78])
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.018, 0.8525],
        [0.8, 0.018, 0.7925],
        [0.8, 0.018, 0.8525],
        [0.8, 0.024, 0.8525],
        [0.8, 0.024, 0.7925],
        [0.8, 0.024, 0.8525],
    ])


def test_tactile_pick_uses_default_grasp_orientation_for_reachable_descent():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[1].get(
        "tolerance_orientation"
    ) == pytest.approx(pick_place_node.TACTILE_GRASP_TOLERANCE_ORIENTATION)
    assert pick_place_node.TACTILE_GRASP_TOLERANCE_ORIENTATION == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION
    )


def test_pick_relaxes_approach_but_keeps_grasp_orientation_tight():
    assert hasattr(pick_place_node, "DEFAULT_GRASP_TOLERANCE_POSITION")
    assert hasattr(pick_place_node, "DEFAULT_GRASP_TOLERANCE_ORIENTATION")
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[0].get("tolerance_orientation") == pytest.approx(0.2)
    assert pick_place.move_kwargs[1].get("tolerance_position") == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_POSITION
    )
    assert pick_place.move_kwargs[1].get("tolerance_orientation") == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION
    )
    assert pick_place_node.DEFAULT_GRASP_TOLERANCE_POSITION == pytest.approx(0.005)
    assert pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION == pytest.approx(0.2)
    assert pick_place.move_kwargs[2].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[2].get("tolerance_orientation") == pytest.approx(0.2)


def test_pick_moves_use_bounded_moveit_waits():
    assert hasattr(pick_place_node, "DEFAULT_MOVE_TIMEOUT_SEC")
    assert pick_place_node.DEFAULT_MOVE_TIMEOUT_SEC >= 240.0
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
    assert action_events(pick_place.events) == [
        "move_above",
        "move_grasp",
        "release",
        "open",
        "move_above",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.82],
        [0.8, 0.0, 0.78 + pick_place_node.PLACE_RELEASE_CLEARANCE],
        [0.8, 0.0, 0.82],
    ])


def test_place_releases_in_midair_above_target_to_avoid_constraint_fight():
    """Place must release in midair above the nominal drop pose."""
    # 根因回归:带焊物块被压向台面时,固定关节与台面接触约束冲突,
    # ODE 求解器给物块注入巨大速度(实测 twist 达 181 m/s 弹飞).
    # 悬空释放从机制上避免"焊接下压"这一约束冲突场景.
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.place([0.8, 0.0, 0.78])
    descend_z = pick_place.move_positions[1][2]
    assert descend_z == pytest.approx(
        0.78 + pick_place_node.PLACE_RELEASE_CLEARANCE
    )
    # 悬空余量至少 2cm,且不超过 4cm(0.05kg 样件自由落体安全带)
    assert 0.02 <= pick_place_node.PLACE_RELEASE_CLEARANCE <= 0.04


def test_tactile_place_releases_above_target_without_extra_downward_compensation():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.place([0.8, 0.0, 0.78])
    assert pick_place.move_positions[1][2] == pytest.approx(
        0.78
        + pick_place_node.TACTILE_PLACE_RELEASE_CLEARANCE
        - pick_place_node.TACTILE_PLACE_TCP_Z_COMPENSATION
    )
    assert pick_place_node.TACTILE_PLACE_RELEASE_CLEARANCE == pytest.approx(0.025)
    assert pick_place_node.TACTILE_PLACE_TCP_Z_COMPENSATION == pytest.approx(0.0)


def test_tactile_place_waits_for_object_drop_before_opening(monkeypatch):
    sleeps = []
    monkeypatch.setattr(
        pick_place_node.time,
        "sleep",
        lambda duration: sleeps.append(duration),
    )
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.place([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "move_above",
        "move_grasp",
        "release",
        "open",
        "move_above",
    ]
    assert sleeps == [pytest.approx(pick_place_node.TACTILE_PLACE_DROP_SETTLE_SEC)]
    assert pick_place_node.TACTILE_PLACE_DROP_SETTLE_SEC == pytest.approx(0.3)


def test_pick_descend_and_lift_use_cartesian_straight_line():
    """Pick descend/lift must be Cartesian to avoid sweeping the object."""
    # 根因回归:OMPL 关节空间规划不保证 TCP 直线,下降段偶发横向弧
    # 扫飞 0.05kg 物块(E2E 实测物块被撞至 100m 外,twist 30m/s).
    # approach 段保留自由规划(远离物块,横弧无害).
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("cartesian") is not True  # approach
    assert pick_place.move_kwargs[1].get("cartesian") is True      # descend
    assert pick_place.move_kwargs[2].get("cartesian") is True      # lift


def test_pick_retries_transient_grasp_descent_failure_before_acquiring_object():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, False, True, True],
        move_names=[
            "move_above",
            "move_grasp",
            "move_grasp_retry",
            "move_above",
        ],
    )

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "move_grasp_retry",
        "acquire",
        "close",
        "move_above",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.84],
        [0.8, 0.0, 0.88],
    ])


def test_place_descend_and_lift_use_cartesian_straight_line():
    """Place descend/lift must be Cartesian while carrying the object."""
    # lift 时物块已焊接在手,横向弧会带着物块扫掠台面.
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.place([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("cartesian") is not True  # approach
    assert pick_place.move_kwargs[1].get("cartesian") is True      # descend
    assert pick_place.move_kwargs[2].get("cartesian") is True      # lift


def test_tactile_place_approach_uses_cartesian_straight_line():
    """Tactile place approach must be Cartesian while carrying the object."""
    # T-4 实测:触觉路径下物块已由 fixed joint 带在手上,place approach
    # 若走自由关节规划,高空移动也会给样件注入大 twist。
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, True],
        use_tactile_grasp=True,
    )

    assert pick_place.place([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("cartesian") is True
    assert pick_place.move_kwargs[1].get("cartesian") is True
    assert pick_place.move_kwargs[2].get("cartesian") is True


def test_place_relaxes_orientation_for_approach_descent_and_lift():
    assert hasattr(pick_place_node, "DEFAULT_GRASP_TOLERANCE_POSITION")
    assert hasattr(pick_place_node, "DEFAULT_GRASP_TOLERANCE_ORIENTATION")
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])

    assert pick_place.place([0.8, 0.0, 0.78])
    assert pick_place.move_kwargs[0].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[0].get("tolerance_orientation") == pytest.approx(0.2)
    assert pick_place.move_kwargs[1].get("tolerance_position") == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_POSITION
    )
    assert pick_place.move_kwargs[1].get("tolerance_orientation") == pytest.approx(
        pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION
    )
    assert pick_place_node.DEFAULT_GRASP_TOLERANCE_POSITION == pytest.approx(0.005)
    assert pick_place_node.DEFAULT_GRASP_TOLERANCE_ORIENTATION == pytest.approx(0.2)
    assert pick_place.move_kwargs[2].get("tolerance_position") == pytest.approx(0.005)
    assert pick_place.move_kwargs[2].get("tolerance_orientation") == pytest.approx(0.2)


def test_default_approach_height_stays_within_reachable_gripper_band():
    assert DEFAULT_APPROACH_HEIGHT == pytest.approx(0.04)


def test_gripper_close_waits_for_visible_motion():
    assert GRIPPER_CLOSE_SETTLE_SEC >= 0.8


def test_pick_stops_before_lift_when_attach_is_refused():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True],
        acquire_ok=False,
    )

    assert not pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
    ]
    assert_positions_close(pick_place.move_positions, [
        [0.8, 0.0, 0.88],
        [0.8, 0.0, 0.84],
    ])


def test_pick_releases_object_when_close_fails_after_attach():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True],
        close_ok=False,
    )

    assert not pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "close",
        "release",
    ]


def test_pick_releases_object_when_lift_fails_after_attach():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, False, False])

    assert not pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "close",
        "move_above",
        "move_above_retry",
        "release",
    ]


def test_pick_retries_transient_lift_move_failure_before_releasing_object():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, False, True])

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "close",
        "move_above",
        "move_above_retry",
    ]


def test_move_treats_late_moveit_success_as_success(monkeypatch):
    pick_place = PickPlace.__new__(PickPlace)
    pick_place.events = []
    pick_place.get_logger = lambda: FakeLogger(pick_place.events)
    monkeypatch.setattr(pick_place_node.time, "sleep", lambda _seconds: None)

    class FakeMoveIt:
        def __init__(self):
            self.wait_results = iter([False, True])
            self.moves = []

        def move_to_pose(self, **kwargs):
            self.moves.append(kwargs)

        def wait_until_executed(self, timeout_sec=None):
            return next(self.wait_results)

    fake_moveit = FakeMoveIt()
    pick_place.moveit2 = fake_moveit

    assert pick_place._move([0.8, 0.0, 0.74])
    assert len(fake_moveit.moves) == 1


def test_moveit_wait_accepts_completed_private_state():
    class FakeMoveIt:
        _MoveIt2__send_goal_future_move_action = object()
        _MoveIt2__get_result_future_move_action = None
        _MoveIt2__last_execution_succeeded = True
        _MoveIt2__is_motion_requested = False
        _MoveIt2__is_executing = False

    assert pick_place_node._wait_for_moveit_result(FakeMoveIt(), 0.1)


def test_moveit_wait_ignores_stale_completed_result_future(monkeypatch):
    class FakeResult:
        status = pick_place_node.GoalStatus.STATUS_SUCCEEDED

    class FakeFuture:
        def __init__(self, done=True):
            self._done = done

        def done(self):
            return self._done

        def result(self):
            return FakeResult()

    class FakeMoveIt:
        _MoveIt2__last_execution_succeeded = True
        _MoveIt2__is_motion_requested = True
        _MoveIt2__is_executing = True

    fake = FakeMoveIt()
    old_send = FakeFuture()
    old_result = FakeFuture()
    new_send = FakeFuture()
    new_result = FakeFuture()
    fake._MoveIt2__send_goal_future_move_action = old_send
    fake._MoveIt2__get_result_future_move_action = old_result

    def finish_current_goal(_seconds):
        fake._MoveIt2__send_goal_future_move_action = new_send
        fake._MoveIt2__get_result_future_move_action = new_result
        fake._MoveIt2__is_motion_requested = False
        fake._MoveIt2__is_executing = False

    monkeypatch.setattr(pick_place_node.time, "sleep", finish_current_goal)

    assert pick_place_node._wait_for_moveit_result(
        fake,
        0.1,
        previous_send_future=old_send,
        previous_result_future=old_result,
    )


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


def test_tactile_moveit_settings_use_lower_speed_to_bound_object_twist():
    assert hasattr(pick_place_node, "TACTILE_ARM_MAX_VELOCITY_SCALING")
    assert hasattr(pick_place_node, "TACTILE_ARM_MAX_ACCELERATION_SCALING")

    class FakeMoveIt:
        max_velocity = None
        max_acceleration = None
        allowed_planning_time = None
        num_planning_attempts = None

    fake = FakeMoveIt()

    pick_place_node.configure_moveit_for_pick_place(
        fake,
        use_tactile_grasp=True,
    )

    assert fake.max_velocity == pytest.approx(
        pick_place_node.TACTILE_ARM_MAX_VELOCITY_SCALING
    )
    assert fake.max_acceleration == pytest.approx(
        pick_place_node.TACTILE_ARM_MAX_ACCELERATION_SCALING
    )
    assert fake.max_velocity == pytest.approx(0.30)
    assert fake.max_acceleration == pytest.approx(0.30)
    assert fake.max_velocity < pick_place_node.ARM_MAX_VELOCITY_SCALING
    assert fake.max_acceleration < pick_place_node.ARM_MAX_ACCELERATION_SCALING
    assert fake.allowed_planning_time == pytest.approx(
        pick_place_node.ARM_ALLOWED_PLANNING_TIME_SEC
    )
    assert fake.num_planning_attempts == pick_place_node.ARM_NUM_PLANNING_ATTEMPTS


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


def test_move_to_observe_uses_probed_fixed_joint_configuration():
    configs = []

    class FakeMoveIt:
        def move_to_configuration(self, config):
            configs.append(list(config))

        def wait_until_executed(self, timeout_sec):
            assert timeout_sec == pick_place_node.DEFAULT_MOVE_TIMEOUT_SEC
            return True

    pick_place = pick_place_node.PickPlace.__new__(pick_place_node.PickPlace)
    pick_place.moveit2 = FakeMoveIt()

    assert pick_place.move_to_observe()
    assert configs == [pick_place_node.OBSERVE_CONFIG]
    assert pick_place_node.OBSERVE_CONFIG == pytest.approx([
        -0.116421,
        -0.807952,
        0.425992,
        -1.337190,
        4.581185,
        -1.844921,
    ])


def test_move_to_observe_retries_transient_execution_failure(monkeypatch):
    results = iter([False, True])
    configs = []

    class FakeMoveIt:
        def move_to_configuration(self, config):
            configs.append(list(config))

        def wait_until_executed(self, timeout_sec):
            assert timeout_sec == pick_place_node.DEFAULT_MOVE_TIMEOUT_SEC
            return next(results)

    monkeypatch.setattr(pick_place_node.time, "sleep", lambda _delay: None)
    pick_place = pick_place_node.PickPlace.__new__(pick_place_node.PickPlace)
    pick_place.moveit2 = FakeMoveIt()

    assert pick_place.move_to_observe()
    assert configs == [pick_place_node.OBSERVE_CONFIG] * 2


def test_pick_injects_surface_before_arm_motion_and_attaches_after_acquire():
    # 台面盒必须先于第一次臂规划注入(否则 approach 弧仍对台面盲);
    # 样件附着盒必须在 acquire 成功后立即挂上(持物段全程护航)。
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])
    pick_place.scene_client = FakeSceneClient(pick_place.events)

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "scene_surface",
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "scene_attach",
        "close",
        "move_above",
    ]


def test_dynamic_arm_obstacle_update_and_clear_use_planning_scene():
    pick_place = make_pick_place_without_ros(fake_moves=[])
    pick_place.scene_client = FakeSceneClient(pick_place.events)

    assert pick_place.update_dynamic_arm_obstacle(
        [0.35, 0.12, 0.50],
        [0.12, 0.12, 0.20],
    )
    assert pick_place.clear_dynamic_arm_obstacle()
    assert action_events(pick_place.events) == [
        "scene_dynamic_update",
        "scene_dynamic_remove",
    ]


def test_dynamic_arm_obstacle_disabled_without_scene_client():
    pick_place = make_pick_place_without_ros(fake_moves=[])

    assert not pick_place.update_dynamic_arm_obstacle(
        [0.35, 0.12, 0.50],
        [0.12, 0.12, 0.20],
    )
    assert not pick_place.clear_dynamic_arm_obstacle()
    assert action_events(pick_place.events) == []


def test_pick_detaches_scene_box_when_close_fails_after_attach():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True],
        close_ok=False,
    )
    pick_place.scene_client = FakeSceneClient(pick_place.events)

    assert not pick_place.pick([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "scene_surface",
        "open",
        "move_above",
        "move_grasp",
        "acquire",
        "scene_attach",
        "close",
        "release",
        "scene_detach",
    ]


def test_pick_detaches_scene_box_when_lift_fails_after_attach():
    pick_place = make_pick_place_without_ros(
        fake_moves=[True, True, False, False]
    )
    pick_place.scene_client = FakeSceneClient(pick_place.events)

    assert not pick_place.pick([0.8, 0.0, 0.78])
    events = action_events(pick_place.events)
    assert events[-2:] == ["release", "scene_detach"]


def test_place_injects_surface_and_detaches_after_release():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])
    pick_place.scene_client = FakeSceneClient(pick_place.events)

    assert pick_place.place([0.8, 0.0, 0.78])
    assert action_events(pick_place.events) == [
        "scene_surface",
        "move_above",
        "move_grasp",
        "release",
        "scene_detach",
        "open",
        "move_above",
    ]


def test_place_stops_hold_monitor_before_waiting_for_release_ack():
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])
    pick_place._hold_monitor_active = True
    pick_place._hold_monitor_fault = False
    holding = {"ok": True}

    def confirms_holding():
        return holding["ok"]

    def release_then_plugin_reports_not_holding():
        pick_place.events.append("release")
        holding["ok"] = False
        pick_place._monitor_held_object()
        return True

    pick_place.gripper.is_holding_object = confirms_holding
    pick_place.gripper.release_object = release_then_plugin_reports_not_holding

    assert pick_place.place([0.8, 0.0, 0.78])
    assert "release" in action_events(pick_place.events)
    assert not any("持有监控失败" in event for event in pick_place.events)
    assert not pick_place._hold_monitor_active


def test_scene_apply_failure_degrades_to_legacy_behavior():
    # 注入失败只降级回旧行为(规划对环境盲),不得阻断抓取任务。
    pick_place = make_pick_place_without_ros(fake_moves=[True, True, True])
    pick_place.scene_client = FakeSceneClient(pick_place.events, ok=False)

    assert pick_place.pick([0.8, 0.0, 0.78])
    assert "log_warn:planning scene surface add apply failed" in pick_place.events
