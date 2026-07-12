"""Mission visual docking policy tests."""

import math
import pytest
from builtin_interfaces.msg import Time

from lab_cobot_bringup.mission_node import (
    DOCK_TARGET_X,
    DOCK_TARGET_Y,
    DOCK_MAX_LINEAR_Y,
    DOCK_SAFE_HANDOFF_MAX_X,
    DOCK_TIMEOUT_SEC,
    DOCK_TOLERANCE_X,
    DOCK_TOLERANCE_Y,
    dock_velocity_for_object,
    transform_stamp_is_fresh,
)


def test_dock_velocity_moves_forward_when_object_is_too_far():
    done, cmd = dock_velocity_for_object([DOCK_TARGET_X + 0.08, DOCK_TARGET_Y, 0.63])

    assert not done
    assert cmd.linear.x > 0.0
    assert cmd.linear.y == pytest.approx(0.0)


def test_dock_velocity_strafes_toward_lateral_object_error():
    done, cmd = dock_velocity_for_object([DOCK_TARGET_X, DOCK_TARGET_Y + 0.08, 0.63])

    assert not done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y > 0.0


def test_pick_visual_dock_hands_off_at_chassis_safety_line():
    from lab_cobot_bringup import mission_node
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a")
    done, cmd = dock_velocity_for_object(
        [DOCK_TARGET_X + 0.12, DOCK_TARGET_Y, 0.63],
        base_pose=(2.0, safe_y, math.pi / 2.0), station="station_a")
    assert done
    assert cmd.linear.x == pytest.approx(0.0)


def test_pick_visual_dock_accepts_real_safe_station_deadband():
    # Dynamic regression: station docking already stopped safely here, while
    # visual x still requested forward motion that the safety limiter forbids.
    done, cmd = dock_velocity_for_object(
        [0.726, 0.010, 0.668],
        base_pose=(1.943, 0.599, math.radians(88.3)),
        station="station_a",
    )

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)


def test_pick_visual_dock_at_safety_line_only_corrects_lateral_error():
    from lab_cobot_bringup import mission_node
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a")
    done, cmd = dock_velocity_for_object(
        [DOCK_TARGET_X + 0.12, DOCK_TARGET_Y + 0.10, 0.63],
        base_pose=(2.0, safe_y, math.pi / 2.0), station="station_a")
    assert not done
    assert cmd.linear.x <= 0.0
    assert cmd.linear.y > 0.0


def test_pick_visual_dock_rejects_unreachable_safe_line_target():
    from lab_cobot_bringup import mission_node
    safe_y = mission_node.station_safe_base_y(math.pi / 2.0, "station_a")
    done, cmd = dock_velocity_for_object(
        [DOCK_SAFE_HANDOFF_MAX_X + 0.01, DOCK_TARGET_Y, 0.668],
        base_pose=(2.0, safe_y, math.pi / 2.0),
        station="station_a",
    )

    assert not done
    assert cmd.linear.x == pytest.approx(0.0)


def test_dock_velocity_stops_inside_pick_window():
    done, cmd = dock_velocity_for_object([
        DOCK_TARGET_X + DOCK_TOLERANCE_X * 0.5,
        DOCK_TARGET_Y - DOCK_TOLERANCE_Y * 0.5,
        0.63,
    ])

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)


def test_dock_velocity_accepts_reachable_lateral_offset_for_tcp_pick():
    done, cmd = dock_velocity_for_object([
        DOCK_TARGET_X,
        DOCK_TARGET_Y + 0.047,
        0.63,
    ])

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)
    assert DOCK_TOLERANCE_Y <= 0.07


def test_dock_velocity_accepts_force_model_longitudinal_deadband():
    done, cmd = dock_velocity_for_object([
        DOCK_TARGET_X + 0.042,
        DOCK_TARGET_Y,
        0.63,
    ])

    assert done
    assert cmd.linear.x == pytest.approx(0.0)


def test_dock_velocity_accepts_gui_verified_near_lateral_offset():
    done, cmd = dock_velocity_for_object([
        DOCK_TARGET_X,
        DOCK_TARGET_Y + 0.064,
        0.63,
    ])

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)


def test_docking_policy_allows_one_precise_lateral_alignment_attempt():
    assert DOCK_TIMEOUT_SEC >= 18.0
    assert DOCK_MAX_LINEAR_Y >= 0.08


def test_docking_standoff_keeps_pick_target_inside_ur5e_workspace():
    # The mecanum3 chassis stops through its wheel-derived cylindrical runtime.
    # Keep enough margin below the UR5e's nominal 0.85 m reach for a vertical TCP.
    assert DOCK_TARGET_X <= 0.70


def test_detection_stamp_rejects_stale_aruco_tf():
    now = Time(sec=12, nanosec=0)
    stale = Time(sec=9, nanosec=800_000_000)

    assert not transform_stamp_is_fresh(now, stale)


def test_detection_stamp_accepts_recent_aruco_tf():
    now = Time(sec=12, nanosec=0)
    recent = Time(sec=11, nanosec=600_000_000)

    assert transform_stamp_is_fresh(now, recent)
