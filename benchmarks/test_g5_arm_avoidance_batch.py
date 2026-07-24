import importlib.util
import sys
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("g5_arm_avoidance_batch.py")
SPEC = importlib.util.spec_from_file_location("g5_arm_avoidance_batch", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
batch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(batch)


def test_default_batch_runs_offset_case(tmp_path):
    args = batch.build_parser().parse_args(["--out-dir", str(tmp_path)])

    cmd = batch.build_demo_command(args, "offset")

    assert cmd[0] == sys.executable
    assert "--case-label" in cmd
    assert "offset_obstacle" in cmd
    assert "--expected-verdict" in cmd
    assert "PLAN_OK" in cmd
    assert str(tmp_path) in cmd


def test_blocked_case_expects_plan_failed(tmp_path):
    args = batch.build_parser().parse_args(["--out-dir", str(tmp_path)])

    cmd = batch.build_demo_command(args, "blocked")

    assert "blocked_obstacle" in cmd
    assert "PLAN_FAILED" in cmd
    assert "0.42" in cmd
    assert "0.28" in cmd


def test_summary_command_points_at_output_dir(tmp_path):
    args = batch.build_parser().parse_args(["--out-dir", str(tmp_path)])

    cmd = batch.build_summary_command(args)

    assert cmd[0] == sys.executable
    assert str(batch.SUMMARY) in cmd
    assert str(tmp_path) in cmd
