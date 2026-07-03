"""Mission visual docking policy tests."""

import pytest

from lab_cobot_bringup.mission_node import (
    DOCK_TARGET_X,
    DOCK_TARGET_Y,
    DOCK_MAX_LINEAR_Y,
    DOCK_TIMEOUT_SEC,
    DOCK_TOLERANCE_X,
    DOCK_TOLERANCE_Y,
    dock_velocity_for_object,
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
