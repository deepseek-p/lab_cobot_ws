"""Regression checks for mecanum wheel visual command mapping."""
import pytest

from lab_cobot_bringup import mecanum_wheel_visualizer
from lab_cobot_bringup.mecanum_wheel_visualizer import (
    shutdown_if_running,
    twist_from_wheel_speeds,
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


def test_forward_kinematics_recovers_twist_from_wheel_speeds():
    wheel_speeds = wheel_speeds_from_twist(
        vx=0.12,
        vy=-0.04,
        wz=0.25,
        wheel_radius=0.08,
        wheelbase_radius=0.5,
    )

    twist = twist_from_wheel_speeds(
        wheel_speeds,
        wheel_radius=0.08,
        wheelbase_radius=0.5,
    )

    assert twist == pytest.approx((0.12, -0.04, 0.25))


def test_pose_model_odometry_twist_is_limited_like_gazebo_drive_plugin():
    limited = mecanum_wheel_visualizer.limit_twist_for_pose_model(
        current_twist=(0.0, 0.0, 0.0),
        target_twist=(0.45, 0.0, 0.9),
        dt=0.05,
        max_linear_accel=0.8,
        max_angular_accel=1.5,
    )

    assert limited == pytest.approx((0.04, 0.0, 0.075))


def test_shutdown_guard_skips_already_shutdown_rclpy_context():
    shutdown_calls = []

    shutdown_if_running(
        ok=lambda: False,
        shutdown=lambda: shutdown_calls.append("called"),
    )

    assert shutdown_calls == []
