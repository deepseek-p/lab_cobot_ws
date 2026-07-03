"""Unit tests for TCP-frame gripper grasp validation."""

import pytest

from lab_cobot_bringup.grasp_validator import (
    GraspValidationConfig,
    validate_tcp_object_grasp,
)


def test_centered_object_inside_finger_band_is_accepted():
    config = GraspValidationConfig()

    result = validate_tcp_object_grasp((0.0, 0.0, -0.025), config)

    assert result.accepted
    assert result.reason == "accepted"
    assert result.offset_tcp == pytest.approx((0.0, 0.0, -0.025))


def test_tcp_too_far_is_refused_before_axis_specific_reasons():
    config = GraspValidationConfig(max_center_distance_m=0.050)

    result = validate_tcp_object_grasp((0.060, 0.0, 0.0), config)

    assert not result.accepted
    assert result.reason == "tcp_too_far"


def test_object_outside_palm_width_is_refused():
    config = GraspValidationConfig(max_center_distance_m=0.200, max_abs_x_m=0.030)

    result = validate_tcp_object_grasp((0.040, 0.0, -0.020), config)

    assert not result.accepted
    assert result.reason == "object_outside_palm_width"


def test_object_outside_finger_gap_is_refused():
    config = GraspValidationConfig(max_center_distance_m=0.200, max_abs_y_m=0.020)

    result = validate_tcp_object_grasp((0.0, 0.035, -0.020), config)

    assert not result.accepted
    assert result.reason == "object_outside_finger_gap"


def test_default_finger_gap_accepts_small_simulation_alignment_error():
    result = validate_tcp_object_grasp((0.0, 0.014, -0.025))

    assert result.accepted
    assert result.reason == "accepted"


def test_default_finger_gap_requires_visibly_centered_object():
    result = validate_tcp_object_grasp((0.0, 0.020, -0.025))

    assert not result.accepted
    assert result.reason == "object_outside_finger_gap"


def test_object_outside_grasp_depth_is_refused_when_too_far_below_tcp():
    config = GraspValidationConfig(
        max_center_distance_m=0.200,
        min_z_m=-0.050,
        max_z_m=0.020,
    )

    result = validate_tcp_object_grasp((0.0, 0.0, -0.070), config)

    assert not result.accepted
    assert result.reason == "object_outside_grasp_depth"


def test_object_outside_grasp_depth_is_refused_when_above_tcp_band():
    config = GraspValidationConfig(
        max_center_distance_m=0.200,
        min_z_m=-0.050,
        max_z_m=0.020,
    )

    result = validate_tcp_object_grasp((0.0, 0.0, 0.030), config)

    assert not result.accepted
    assert result.reason == "object_outside_grasp_depth"
