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
