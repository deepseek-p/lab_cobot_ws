from types import SimpleNamespace

import pytest

from lab_cobot_manipulation.contact_force_recorder import (
    force_for_target,
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


def test_force_for_target_filters_unrelated_contacts_and_sums_magnitudes():
    message = SimpleNamespace(
        states=[
            _state("lab::left_probe", "aruco_sample::link::collision", (3, 4, 0)),
            _state("aruco_sample::link::collision", "lab::right_probe", (0, 0, 12)),
            _state("lab::left_probe", "station_a_table::link::collision", (50, 0, 0)),
        ]
    )

    assert force_for_target(message, "aruco_sample") == pytest.approx(17.0)
