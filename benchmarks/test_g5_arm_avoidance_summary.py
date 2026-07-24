import csv
import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("g5_arm_avoidance_summary.py")
SPEC = importlib.util.spec_from_file_location("g5_arm_avoidance_summary", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
summary = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(summary)


FIELDNAMES = [
    "stamp",
    "case_label",
    "expected_verdict",
    "expected_met",
    "verdict",
    "obstacle_id",
    "obstacle_frame",
    "obstacle_center",
    "obstacle_size",
    "target_frame",
    "target_link",
    "target",
    "start_joints",
    "apply_scene_ok",
    "apply_scene_latency_ms",
    "apply_scene_service_wait_ms",
    "apply_scene_call_latency_ms",
    "planning_ok",
    "moveit_error_code",
    "trajectory_points",
    "planning_latency_ms",
    "total_latency_ms",
    "control_latency_ms",
]


def _write_csv(path, *, stamp, verdict, ok, total_ms):
    row = {
        "stamp": stamp,
        "case_label": verdict.lower(),
        "expected_verdict": verdict,
        "expected_met": "True",
        "verdict": verdict,
        "obstacle_id": "g5_dynamic_box",
        "obstacle_frame": "base_link",
        "obstacle_center": "0.350000 0.120000 0.500000",
        "obstacle_size": "0.120000 0.120000 0.200000",
        "target_frame": "base_link",
        "target_link": "gripper_tcp",
        "target": "0.450000 0.000000 0.620000",
        "start_joints": "0.000000 -1.570800 1.570800 -1.570800 -1.570800 0.000000",
        "apply_scene_ok": "True",
        "apply_scene_latency_ms": "30.000",
        "apply_scene_service_wait_ms": "2.000",
        "apply_scene_call_latency_ms": "28.000",
        "planning_ok": str(ok),
        "moveit_error_code": "1" if ok else "99999",
        "trajectory_points": "10" if ok else "0",
        "planning_latency_ms": "50.000",
        "total_latency_ms": f"{total_ms:.3f}",
        "control_latency_ms": f"{total_ms - 2.0:.3f}",
    }
    with path.open("w", encoding="utf-8", newline="") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=FIELDNAMES)
        writer.writeheader()
        writer.writerow(row)


def test_summarize_counts_success_and_latency_distribution(tmp_path):
    one = tmp_path / "g5_arm_avoidance_1.csv"
    two = tmp_path / "g5_arm_avoidance_2.csv"
    _write_csv(one, stamp="one", verdict="PLAN_OK", ok=True, total_ms=100.0)
    _write_csv(two, stamp="two", verdict="PLAN_FAILED", ok=False, total_ms=250.0)

    rows = summary.read_measurements([one, two])
    stats = summary.summarize(rows)

    assert stats.count == 2
    assert stats.ok_count == 1
    assert stats.failed_count == 1
    assert stats.expected_count == 2
    assert stats.expected_met_count == 2
    assert stats.expected_met_rate == 1.0
    assert stats.plan_ok_rate == 0.5
    assert stats.total_p95_ms == 250.0
    assert stats.total_over_200ms == 1
    assert stats.total_within_200ms == 1
    assert stats.total_within_200ms_rate == 0.5
    assert stats.apply_scene_service_wait_mean_ms == 2.0
    assert stats.apply_scene_call_mean_ms == 28.0
    assert stats.control_within_200ms_rate == 0.5


def test_write_summary_includes_rows_and_distribution(tmp_path):
    one = tmp_path / "g5_arm_avoidance_1.csv"
    _write_csv(one, stamp="one", verdict="PLAN_OK", ok=True, total_ms=100.0)
    rows = summary.read_measurements([one])
    stats = summary.summarize(rows)

    report = summary.write_summary(rows, stats, tmp_path)

    text = report.read_text(encoding="utf-8")
    assert "规划成功率" in text
    assert "预期满足率" in text
    assert "PlanningScene 服务发现均值" in text
    assert "PlanningScene apply 调用均值" in text
    assert "控制响应延时 <=200ms 达标率" in text
    assert "<=200ms 达标率" in text
    assert "P95" in text
    assert "| one | plan_ok | PLAN_OK | True | PLAN_OK |" in text


def test_expand_inputs_ignores_summary_csv(tmp_path):
    measurement = tmp_path / "g5_arm_avoidance_1.csv"
    summary_csv = tmp_path / "g5_arm_avoidance_summary_1.csv"
    measurement.write_text("", encoding="utf-8")
    summary_csv.write_text("", encoding="utf-8")

    assert summary._expand_inputs([tmp_path]) == [measurement]
