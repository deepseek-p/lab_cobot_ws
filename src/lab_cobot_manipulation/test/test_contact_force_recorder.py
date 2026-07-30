from types import SimpleNamespace

import pytest

from lab_cobot_manipulation.contact_force_recorder import (
    binned_peak_series,
    contact_window,
    finite_nonnegative,
    force_for_target,
    parse_args,
    plot_y_limit,
    vector_length,
)


def _vector(x, y, z):
    return SimpleNamespace(x=x, y=y, z=z)


def _state(collision1, collision2, force):
    return SimpleNamespace(
        collision1_name=collision1,
        collision2_name=collision2,
        total_wrench=SimpleNamespace(force=_vector(*force)),
    )


def test_vector_length_returns_force_magnitude():
    assert vector_length(_vector(3.0, 4.0, 12.0)) == pytest.approx(13.0)


def test_finite_nonnegative_rejects_invalid_force_values():
    assert finite_nonnegative(float("inf")) == pytest.approx(0.0)
    assert finite_nonnegative(float("nan")) == pytest.approx(0.0)
    assert finite_nonnegative(-3.0) == pytest.approx(0.0)
    assert finite_nonnegative(4.5) == pytest.approx(4.5)


def test_plot_y_limit_ignores_extreme_solver_spikes():
    limit = plot_y_limit([0.0, 13.8, 20.0, 33.8, 2_000_000.0])

    assert limit == pytest.approx(45.63)


def test_contact_window_focuses_nonzero_force_region():
    window = contact_window(
        [0.0, 10.0, 20.0, 30.0, 40.0],
        [0.0, 0.0, 12.0, 0.0, 0.0],
        padding_sec=5.0,
    )

    assert window == pytest.approx((15.0, 25.0))


def test_binned_peak_series_keeps_each_bin_peak():
    x_values, left_values, right_values = binned_peak_series(
        [0.0, 0.1, 0.2, 0.8, 0.9, 1.0],
        [0.0, 2.0, 1.0, 3.0, 7.0, 4.0],
        [1.0, 0.0, 5.0, 0.0, 6.0, 2.0],
        max_points=2,
    )

    assert len(x_values) == 2
    assert left_values == pytest.approx([2.0, 7.0])
    assert right_values == pytest.approx([5.0, 6.0])


def test_force_for_target_filters_unrelated_contacts_and_sums_magnitudes():
    message = SimpleNamespace(
        states=[
            _state("lab::left_probe", "aruco_sample::link::collision", (3, 4, 0)),
            _state("aruco_sample::link::collision", "lab::right_probe", (0, 0, 12)),
            _state("lab::left_probe", "station_a_table::link::collision", (50, 0, 0)),
        ]
    )

    assert force_for_target(message, "aruco_sample") == pytest.approx(17.0)


def test_parse_args_ignores_ros_launch_arguments():
    args = parse_args([
        "contact_force_recorder",
        "--duration",
        "3.5",
        "--target-object",
        "sample",
        "--ros-args",
    ])

    assert args.duration == pytest.approx(3.5)
    assert args.target_object == "sample"
