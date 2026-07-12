"""Contract tests for the mecanum rover twist relay."""

import inspect

import pytest

from lab_cobot_bringup import rover_twist_relay
from lab_cobot_bringup.rover_twist_relay import (
    SimpleTwist,
    apply_deadband,
    limit_twist,
    ramp_twist,
    reset_twists_on_clock_jump,
    sanitize_twist,
    twist_to_wheel_speeds,
    validate_configuration,
    zero_if_timed_out,
)


@pytest.mark.parametrize(
    "invalid_value",
    [float("nan"), float("inf"), float("-inf")],
)
@pytest.mark.parametrize("component", range(3))
def test_sanitize_twist_rejects_nonfinite_components(invalid_value, component):
    values = [0.1, 0.2, 0.3]
    values[component] = invalid_value

    assert sanitize_twist(SimpleTwist(*values)) == SimpleTwist()


def test_sanitize_twist_preserves_finite_components():
    twist = SimpleTwist(0.2, -0.1, 0.4)

    assert sanitize_twist(twist) is twist


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
    limited = limit_twist(SimpleTwist(0.8, -0.7, 2.0), 0.5, 0.3, 1.2)

    assert limited == SimpleTwist(0.5, -0.3, 1.2)


def test_ramp_twist_moves_each_component_by_one_step():
    ramped = ramp_twist(
        SimpleTwist(), SimpleTwist(0.5, -0.5, 1.0), 0.1, 0.5, 1.5
    )
    assert (ramped.vx, ramped.vy, ramped.wz) == pytest.approx(
        (0.05, -0.05, 0.15)
    )


def test_apply_deadband_zeros_only_small_components():
    result = apply_deadband(SimpleTwist(0.01, -0.03, 0.02), 0.02, 0.03)

    assert result == SimpleTwist(0.0, -0.03, 0.0)


def test_zero_if_timed_out_returns_zero_after_timeout():
    result = zero_if_timed_out(SimpleTwist(0.2, -0.1, 0.4), 1.5, 1.0)

    assert result == SimpleTwist()


def test_zero_if_timed_out_preserves_recent_twist():
    twist = SimpleTwist(0.2, -0.1, 0.4)
    assert zero_if_timed_out(twist, 0.5, 1.0) is twist


def test_clock_rollback_clears_commands_before_they_can_be_reused():
    target, current, reset = reset_twists_on_clock_jump(
        SimpleTwist(0.2, -0.1, 0.4),
        SimpleTwist(0.1, 0.0, 0.2),
        raw_dt=0.01,
        elapsed=-0.01,
    )

    assert reset is True
    assert target == SimpleTwist()
    assert current == SimpleTwist()


def test_normal_clock_progress_preserves_commands():
    target = SimpleTwist(0.2, -0.1, 0.4)
    current = SimpleTwist(0.1, 0.0, 0.2)

    assert reset_twists_on_clock_jump(target, current, 0.01, 0.01) == (
        target,
        current,
        False,
    )


def test_clock_rollback_before_new_twist_still_clears_old_current_speed():
    target, current, reset = reset_twists_on_clock_jump(
        SimpleTwist(0.2, -0.1, 0.4),
        SimpleTwist(0.1, 0.0, 0.2),
        raw_dt=-0.01,
        elapsed=0.0,
    )

    assert reset is True
    assert target == SimpleTwist()
    assert current == SimpleTwist()


@pytest.mark.parametrize(
    "overrides",
    [
        {"wheel_radius": 0.0},
        {"wheel_separation_width": -0.1},
        {"wheel_separation_length": -0.1},
        {"max_vx": -0.1},
        {"max_vy": -0.1},
        {"max_wz": -0.1},
        {"max_accel_xy": -0.1},
        {"max_accel_wz": -0.1},
        {"command_timeout": -0.1},
        {"linear_deadband": -0.1},
        {"angular_deadband": -0.1},
    ],
)
def test_invalid_configuration_is_rejected(overrides):
    configuration = {
        "wheel_radius": 0.07,
        "wheel_separation_width": 0.24,
        "wheel_separation_length": 0.175,
        "max_vx": 0.5,
        "max_vy": 0.3,
        "max_wz": 1.2,
        "max_accel_xy": 0.5,
        "max_accel_wz": 1.5,
        "command_timeout": 0.25,
        "linear_deadband": 0.001,
        "angular_deadband": 0.001,
    }
    configuration.update(overrides)

    with pytest.raises(ValueError):
        validate_configuration(**configuration)


def test_default_configuration_is_valid():
    validate_configuration(
        wheel_radius=0.07,
        wheel_separation_width=0.24,
        wheel_separation_length=0.175,
        max_vx=0.5,
        max_vy=0.3,
        max_wz=1.2,
        max_accel_xy=0.5,
        max_accel_wz=1.5,
        command_timeout=0.25,
        linear_deadband=0.001,
        angular_deadband=0.001,
    )


def test_node_source_does_not_override_ros_time_configuration():
    source = inspect.getsource(rover_twist_relay.RoverTwistRelay)

    assert "'use_sim_time'" not in source
    assert "TimeSource" not in source


def test_node_source_keeps_controller_publisher_and_both_twist_subscriptions():
    source = inspect.getsource(rover_twist_relay.RoverTwistRelay)

    assert "'/wheel_velocity_controller/commands'" in source
    assert "'/rover_twist'" in source
    assert "'/cmd_vel'" in source
