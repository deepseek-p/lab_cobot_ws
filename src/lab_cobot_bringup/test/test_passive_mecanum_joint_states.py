"""Contracts for the mecanum chassis passive joint state source."""

from lab_cobot_bringup.passive_mecanum_joint_states import (
    passive_joint_names,
    shutdown_if_running,
)


def test_passive_joint_source_covers_only_unmeasured_rollers():
    names = passive_joint_names()

    assert len(names) == 60
    assert len(set(names)) == 60
    assert {
        f"{wheel}_barrel_{index}_joint"
        for wheel in ("front_left", "front_right", "back_left", "back_right")
        for index in range(15)
    } <= set(names)
    assert not any(name.endswith("_suspension_joint") for name in names)
    assert not any(name.endswith("_wheel_joint") for name in names)


def test_shutdown_is_skipped_after_launch_already_stopped_context(monkeypatch):
    calls = []
    monkeypatch.setattr(
        "lab_cobot_bringup.passive_mecanum_joint_states.rclpy.ok",
        lambda: False,
    )
    monkeypatch.setattr(
        "lab_cobot_bringup.passive_mecanum_joint_states.rclpy.shutdown",
        lambda: calls.append("shutdown"),
    )

    shutdown_if_running()

    assert calls == []
