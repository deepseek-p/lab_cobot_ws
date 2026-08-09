"""Unit tests for the G4/G5 mission result summary."""

from lab_cobot_bringup.g4g5_result_node import parse_fingers_status, summarize_result


def test_parse_fingers_status_reads_plugin_snapshot():
    assert parse_fingers_status("fingers left=1 right=0") == (True, False)
    assert parse_fingers_status("fingers left=0 right=1") == (False, True)
    assert parse_fingers_status("fingers left=1 right=1") == (True, True)
    assert parse_fingers_status("released none") == (False, False)


def test_summary_marks_g4_ok_after_done_and_both_fingers_touch():
    summary = summarize_result(
        "DONE",
        3,
        1.25,
        2.5,
        3.75,
        4,
        5,
        ["attached aruco_sample", "released aruco_sample"],
        ["ready topic=/arm_dynamic_obstacle_box frame=base_link id=g5_dynamic_arm_obstacle"],
    )

    assert "task_status=DONE" in summary
    assert "g4_touch_ok=True" in summary
    assert "g4_peak_left_n=1.250" in summary
    assert "g4_peak_right_n=2.500" in summary
    assert "g4_peak_sum_n=3.750" in summary
    assert "g5_bridge_ready=True" in summary


def test_summary_counts_g5_dynamic_obstacle_updates():
    summary = summarize_result(
        "DONE",
        0,
        0.0,
        0.0,
        0.0,
        0,
        0,
        [],
        [
            "ready topic=/arm_dynamic_obstacle_box frame=base_link id=g5_dynamic_arm_obstacle",
            "updated center=[0.35, 0.12, 0.5] size=[0.12, 0.12, 0.2]",
            "removed id=g5_dynamic_arm_obstacle",
            "updated center=[0.4, 0.12, 0.5] size=[0.12, 0.12, 0.2]",
        ],
    )

    assert "g4_touch_ok=False" in summary
    assert "g5_dynamic_obstacle_updates=2" in summary
    assert "g5_last_status='updated center=[0.4, 0.12, 0.5] size=[0.12, 0.12, 0.2]'" in summary
