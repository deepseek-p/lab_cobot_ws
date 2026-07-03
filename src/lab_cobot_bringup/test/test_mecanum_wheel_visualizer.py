"""Regression checks for mecanum wheel visual command mapping."""
import pytest

from lab_cobot_bringup.mecanum_wheel_visualizer import (
    shutdown_if_running,
    wheel_speeds_from_twist,
)


def test_forward_motion_spins_all_wheels_same_direction():
    speeds = wheel_speeds_from_twist(
        vx=0.16,
        vy=0.0,
        wz=0.0,
        wheel_radius=0.08,
        wheelbase_radius=0.5,
    )

    assert speeds == pytest.approx([2.0, 2.0, 2.0, 2.0])


def test_lateral_motion_uses_mecanum_opposing_diagonals():
    speeds = wheel_speeds_from_twist(
        vx=0.0,
        vy=0.16,
        wz=0.0,
        wheel_radius=0.08,
        wheelbase_radius=0.5,
    )

    assert speeds == pytest.approx([-2.0, 2.0, 2.0, -2.0])


def test_yaw_motion_spins_left_and_right_sides_opposite():
    speeds = wheel_speeds_from_twist(
        vx=0.0,
        vy=0.0,
        wz=0.4,
        wheel_radius=0.08,
        wheelbase_radius=0.5,
    )

    assert speeds == pytest.approx([-2.5, 2.5, -2.5, 2.5])


def test_shutdown_guard_skips_already_shutdown_rclpy_context():
    shutdown_calls = []

    shutdown_if_running(
        ok=lambda: False,
        shutdown=lambda: shutdown_calls.append("called"),
    )

    assert shutdown_calls == []
