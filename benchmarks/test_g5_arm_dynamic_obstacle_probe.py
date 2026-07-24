import importlib.util
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("g5_arm_dynamic_obstacle_probe.py")
SPEC = importlib.util.spec_from_file_location("g5_arm_dynamic_obstacle_probe", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
probe = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(probe)


def test_validate_box_spec_accepts_positive_box():
    spec = probe.BoxSpec(
        object_id="g5_dynamic_box",
        frame_id="base_link",
        center=(0.5, 0.0, 0.6),
        size=(0.2, 0.2, 0.4),
    )

    probe.validate_box_spec(spec)


def test_validate_box_spec_rejects_non_positive_size():
    spec = probe.BoxSpec(
        object_id="g5_dynamic_box",
        frame_id="base_link",
        center=(0.5, 0.0, 0.6),
        size=(0.2, 0.0, 0.4),
    )

    with pytest.raises(ValueError, match="positive"):
        probe.validate_box_spec(spec)


def test_validate_box_spec_rejects_nan_center():
    spec = probe.BoxSpec(
        object_id="g5_dynamic_box",
        frame_id="base_link",
        center=(float("nan"), 0.0, 0.6),
        size=(0.2, 0.2, 0.4),
    )

    with pytest.raises(ValueError, match="finite"):
        probe.validate_box_spec(spec)


def test_parse_triplet_returns_three_floats():
    assert probe._parse_triplet(["1", "2.5", "-3"], "center") == (1.0, 2.5, -3.0)


def test_ros_time_to_float_combines_seconds_and_nanoseconds():
    class Stamp:
        sec = 12
        nanosec = 345000000

    assert probe.ros_time_to_float(Stamp()) == 12.345
