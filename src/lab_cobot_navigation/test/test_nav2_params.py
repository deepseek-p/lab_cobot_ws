"""Regression checks for Nav2 tuning."""
from pathlib import Path

import yaml


def test_progress_checker_allows_terminal_settling_in_small_lab():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))

    progress_checker = params["controller_server"]["ros__parameters"][
        "progress_checker"
    ]

    assert progress_checker["required_movement_radius"] <= 0.2
    assert progress_checker["movement_time_allowance"] >= 20.0
