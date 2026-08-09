import importlib.util
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("g5_arm_avoidance_demo.py")
SPEC = importlib.util.spec_from_file_location("g5_arm_avoidance_demo", MODULE_PATH)
assert SPEC is not None
assert SPEC.loader is not None
demo = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(demo)


def test_parser_defaults_to_plan_only_and_cleanup():
    args = demo.build_parser().parse_args([])

    assert args.execute is False
    assert args.cleanup is True
    assert args.target_link == "gripper_tcp"
    assert args.target_frame == "base_link"
    assert args.out_dir.name == "results"
    assert args.case_label == "manual"
    assert args.expected_verdict == "ANY"


def test_parser_accepts_execute_and_custom_obstacle():
    args = demo.build_parser().parse_args([
        "--execute",
        "--obstacle-center", "0.1", "0.2", "0.3",
        "--obstacle-size", "0.4", "0.5", "0.6",
        "--target", "0.7", "0.8", "0.9",
    ])

    assert args.execute is True
    assert args.obstacle_center == [0.1, 0.2, 0.3]
    assert args.obstacle_size == [0.4, 0.5, 0.6]
    assert args.target == [0.7, 0.8, 0.9]


def test_write_measurement_outputs_csv_and_markdown(tmp_path):
    measurement = demo.RunMeasurement(
        stamp="20260723-141500",
        case_label="offset_obstacle",
        expected_verdict="PLAN_OK",
        expected_met=True,
        verdict="PLAN_OK",
        obstacle_id="g5_dynamic_box",
        obstacle_frame="base_link",
        obstacle_center=(0.35, 0.12, 0.50),
        obstacle_size=(0.12, 0.12, 0.20),
        target_frame="base_link",
        target_link="gripper_tcp",
        target=(0.45, 0.0, 0.62),
        start_joints=tuple(demo.HOME_CONFIG),
        apply_scene_ok=True,
        apply_scene_latency_s=0.0123,
        apply_scene_service_wait_s=0.001,
        apply_scene_call_latency_s=0.0113,
        planning_ok=True,
        moveit_error_code=demo.MOVEIT_SUCCESS,
        trajectory_points=8,
        planning_latency_s=0.4567,
        total_latency_s=0.5001,
    )

    csv_path, md_path = demo._write_measurement(measurement, tmp_path)

    csv_text = csv_path.read_text(encoding="utf-8")
    md_text = md_path.read_text(encoding="utf-8")
    assert "apply_scene_latency_ms" in csv_text
    assert "apply_scene_service_wait_ms" in csv_text
    assert "apply_scene_call_latency_ms" in csv_text
    assert "planning_latency_ms" in csv_text
    assert "control_latency_ms" in csv_text
    assert "PLAN_OK" in csv_text
    assert "expected_met" in csv_text
    assert "预期满足: True" in md_text
    assert "PlanningScene 服务发现" in md_text
    assert "PlanningScene apply 调用" in md_text
    assert "总响应延时" in md_text
    assert "服务已连接后的控制响应延时" in md_text
    assert "trajectory_points=8" in md_text
