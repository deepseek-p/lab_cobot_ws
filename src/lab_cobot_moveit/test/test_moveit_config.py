"""MoveIt configuration regression tests."""
from pathlib import Path

import yaml


def test_ompl_pipeline_declares_planning_plugin():
    config_file = (
        Path(__file__).resolve().parents[1] / "config" / "ompl_planning.yaml"
    )
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))

    assert config["planning_plugin"] == "ompl_interface/OMPLPlanner"


def test_ompl_pipeline_time_parameterizes_trajectories():
    config_file = (
        Path(__file__).resolve().parents[1] / "config" / "ompl_planning.yaml"
    )
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))

    request_adapters = config["request_adapters"]

    assert (
        "default_planner_request_adapters/AddTimeOptimalParameterization"
        in request_adapters
    )
    assert config["start_state_max_bounds_error"] == 0.1


def test_trajectory_execution_tolerates_gazebo_settling_residual():
    config_file = (
        Path(__file__).resolve().parents[1] / "config" / "moveit_controllers.yaml"
    )
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))

    assert config["trajectory_execution"]["allowed_start_tolerance"] == 0.05
    assert config["trajectory_execution"]["allowed_execution_duration_scaling"] == 10.0
    assert config["trajectory_execution"]["allowed_goal_duration_margin"] == 30.0


def test_joint_limits_prevent_multi_turn_grasp_paths():
    config_file = (
        Path(__file__).resolve().parents[1] / "config" / "joint_limits.yaml"
    )
    config = yaml.safe_load(config_file.read_text(encoding="utf-8"))

    limits = config["joint_limits"]
    for joint_name, joint_limits in limits.items():
        assert joint_limits["has_velocity_limits"]
        assert joint_limits["max_velocity"] == 3.141592653589793

    for joint_name in (
        "ur_shoulder_pan_joint",
        "ur_shoulder_lift_joint",
        "ur_wrist_1_joint",
        "ur_wrist_2_joint",
        "ur_wrist_3_joint",
    ):
        assert limits[joint_name]["has_position_limits"]
        assert limits[joint_name]["min_position"] == -3.141592653589793
        assert limits[joint_name]["max_position"] == 3.141592653589793
