"""Contract tests for the mecanum rover twist relay."""

import pytest

from lab_cobot_bringup.rover_twist_relay import (
    SimpleTwist,
    apply_deadband,
    limit_twist,
    ramp_twist,
    twist_to_wheel_speeds,
    zero_if_timed_out,
)


@pytest.mark.parametrize(
    ("twist", "expected"),
    [
        ((0.14, 0.0, 0.0), [-2.0, -2.0, -2.0, -2.0]),
        ((0.0, 0.14, 0.0), [2.0, -2.0, -2.0, 2.0]),
        ((0.0, 0.0, 0.14), [0.83, -0.83, 0.83, -0.83]),
    ],
)
def test_default_geometry_wheel_speed_contract(twist, expected):
    assert twist_to_wheel_speeds(*twist) == pytest.approx(expected)


def test_limit_twist_clamps_linear_and_angular_components():
    assert limit_twist(SimpleTwist(0.8, -0.7, 2.0), 0.5, 0.3, 1.2) == SimpleTwist(
        0.5, -0.3, 1.2
    )


def test_ramp_twist_moves_each_component_by_one_step():
    ramped = ramp_twist(
        SimpleTwist(), SimpleTwist(0.5, -0.5, 1.0), 0.1, 0.5, 1.5
    )
    assert (ramped.vx, ramped.vy, ramped.wz) == pytest.approx((0.05, -0.05, 0.15))


def test_apply_deadband_zeros_only_small_components():
    assert apply_deadband(SimpleTwist(0.01, -0.03, 0.02), 0.02, 0.03) == SimpleTwist(
        0.0, -0.03, 0.0
    )


def test_zero_if_timed_out_returns_zero_after_timeout():
    assert zero_if_timed_out(SimpleTwist(0.2, -0.1, 0.4), 1.5, 1.0) == SimpleTwist()


def test_zero_if_timed_out_preserves_recent_twist():
    twist = SimpleTwist(0.2, -0.1, 0.4)
    assert zero_if_timed_out(twist, 0.5, 1.0) is twist
