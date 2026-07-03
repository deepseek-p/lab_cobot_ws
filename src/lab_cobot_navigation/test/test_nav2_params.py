"""Regression checks for Nav2 tuning."""
from pathlib import Path

import yaml


def test_progress_checker_allows_terminal_settling_in_small_lab():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))

    progress_checker = params["controller_server"]["ros__parameters"][
        "progress_checker"
    ]

    assert progress_checker["required_movement_radius"] <= 0.05
    assert progress_checker["movement_time_allowance"] >= 60.0


def test_goal_checker_is_precise_enough_for_pick_station():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]

    assert controller["general_goal_checker"]["xy_goal_tolerance"] <= 0.12
    assert controller["FollowPath"]["xy_goal_tolerance"] <= 0.12


def test_controller_can_back_out_of_table_facing_stations():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["min_vel_x"] <= -0.15


def test_rotate_to_goal_waits_for_actual_translation_stop():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["trans_stopped_velocity"] <= 0.05
    assert follow_path["trans_stopped_velocity"] <= follow_path["max_speed_xy"] * 0.25


def test_terminal_rotation_has_fine_symmetric_sampling():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    angular_step = 2 * follow_path["max_vel_theta"] / (follow_path["vtheta_samples"] - 1)

    assert angular_step <= 0.05
    assert follow_path["vtheta_samples"] % 2 == 1


def test_mecanum_navigation_keeps_lateral_velocity_enabled():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert controller["min_y_velocity_threshold"] <= 0.01
    assert follow_path["min_vel_y"] < 0.0
    assert follow_path["max_vel_y"] > 0.0
    assert follow_path["vy_samples"] >= 7
    assert smoother["min_velocity"][1] < 0.0
    assert smoother["max_velocity"][1] > 0.0
    assert smoother["max_accel"][1] > 0.0
    assert smoother["max_decel"][1] < 0.0


def test_mecanum_navigation_does_not_creep_on_long_station_legs():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert follow_path["min_speed_xy"] >= 0.05
    assert follow_path["max_speed_xy"] >= 0.34
    assert follow_path["max_vel_x"] >= 0.34
    assert follow_path["max_vel_y"] >= 0.24
    assert smoother["max_velocity"][0] >= follow_path["max_vel_x"]
    assert smoother["max_velocity"][1] >= follow_path["max_vel_y"]


def test_goal_checker_revalidates_xy_while_turning_at_station():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]

    assert controller["general_goal_checker"]["stateful"] is False
    assert follow_path["stateful"] is False


def test_dwb_sampling_budget_stays_realtime_for_lab_sim():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]

    trajectory_samples = (
        follow_path["vx_samples"]
        * follow_path["vy_samples"]
        * follow_path["vtheta_samples"]
    )

    assert controller["controller_frequency"] <= 10.0
    assert trajectory_samples <= 4000


def test_station_approach_keeps_alignment_strong_enough_for_terminal_docking():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["debug_trajectory_details"] is False
    assert follow_path["PathAlign.scale"] >= 24.0
    assert follow_path["GoalAlign.scale"] >= 20.0
    assert follow_path["PathAlign.forward_point_distance"] >= 0.1
    assert follow_path["GoalAlign.forward_point_distance"] >= 0.1
    assert follow_path["RotateToGoal.scale"] >= 24.0
    assert follow_path["RotateToGoal.slowing_factor"] >= 5.0


def test_bt_navigator_debug_monitoring_is_disabled_for_gui_missions():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    params = yaml.safe_load(params_file.read_text(encoding="utf-8"))
    bt_navigator = params["bt_navigator"]["ros__parameters"]

    assert bt_navigator["enable_groot_monitoring"] is False
