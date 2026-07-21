"""Regression checks for Nav2 tuning."""
from pathlib import Path

import yaml


def _nav2_params():
    params_file = Path(__file__).resolve().parents[1] / "config" / "nav2_params.yaml"
    return yaml.safe_load(params_file.read_text(encoding="utf-8"))


def test_progress_checker_allows_terminal_settling_in_small_lab():
    params = _nav2_params()

    progress_checker = params["controller_server"]["ros__parameters"][
        "progress_checker"
    ]

    assert progress_checker["required_movement_radius"] <= 0.05
    assert progress_checker["movement_time_allowance"] >= 60.0


def test_goal_checker_is_precise_enough_for_pick_station():
    params = _nav2_params()
    controller = params["controller_server"]["ros__parameters"]

    assert controller["general_goal_checker"]["xy_goal_tolerance"] <= 0.15
    assert controller["FollowPath"]["xy_goal_tolerance"] <= 0.15


def test_controller_can_back_out_of_table_facing_stations():
    params = _nav2_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["min_vel_x"] <= -0.15


def test_rotate_to_goal_waits_for_actual_translation_stop():
    params = _nav2_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["trans_stopped_velocity"] <= 0.05
    assert follow_path["trans_stopped_velocity"] <= follow_path["max_speed_xy"] * 0.25


def test_terminal_rotation_has_fine_symmetric_sampling():
    params = _nav2_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    angular_step = 2 * follow_path["max_vel_theta"] / (follow_path["vtheta_samples"] - 1)

    assert angular_step <= 0.05
    assert follow_path["vtheta_samples"] % 2 == 1


def test_mecanum_navigation_keeps_lateral_velocity_enabled():
    params = _nav2_params()
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
    params = _nav2_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]
    smoother = params["velocity_smoother"]["ros__parameters"]

    assert follow_path["min_speed_xy"] >= 0.05
    assert follow_path["max_speed_xy"] >= 0.34
    assert follow_path["max_vel_x"] >= 0.34
    assert follow_path["max_vel_y"] >= 0.24
    assert smoother["max_velocity"][0] >= follow_path["max_vel_x"]
    assert smoother["max_velocity"][1] >= follow_path["max_vel_y"]


def test_goal_checker_revalidates_xy_while_turning_at_station():
    params = _nav2_params()
    controller = params["controller_server"]["ros__parameters"]
    follow_path = controller["FollowPath"]

    assert controller["general_goal_checker"]["stateful"] is False
    assert follow_path["stateful"] is False


def test_dwb_sampling_budget_stays_realtime_for_lab_sim():
    params = _nav2_params()
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
    params = _nav2_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert follow_path["debug_trajectory_details"] is False
    assert follow_path["PathAlign.scale"] >= 24.0
    assert follow_path["GoalAlign.scale"] >= 20.0
    assert follow_path["PathAlign.forward_point_distance"] >= 0.1
    assert follow_path["GoalAlign.forward_point_distance"] >= 0.1
    assert follow_path["RotateToGoal.scale"] >= 24.0
    assert follow_path["RotateToGoal.slowing_factor"] >= 5.0


def test_bt_navigator_debug_monitoring_is_disabled_for_gui_missions():
    params = _nav2_params()
    bt_navigator = params["bt_navigator"]["ros__parameters"]

    assert bt_navigator["enable_groot_monitoring"] is False


def test_amcl_uses_omni_motion_model_for_mecanum_base():
    params = _nav2_params()
    amcl = params["amcl"]["ros__parameters"]

    assert amcl["robot_model_type"] == "nav2_amcl::OmniMotionModel"


def test_amcl_initial_pose_matches_five_zone_home_spawn():
    params = _nav2_params()
    initial_pose = params["amcl"]["ros__parameters"]["initial_pose"]

    assert initial_pose == {
        "x": 4.50,
        "y": -4.20,
        "z": 0.0,
        "yaw": 0.0,
    }


def test_humble_behavior_server_replaces_legacy_recoveries_server():
    params = _nav2_params()

    assert "recoveries_server" not in params
    behavior_server = params["behavior_server"]["ros__parameters"]
    assert behavior_server["behavior_plugins"] == ["spin", "backup", "wait"]
    assert "recovery_plugins" not in behavior_server
    assert behavior_server["spin"]["plugin"] == "nav2_behaviors/Spin"
    assert behavior_server["backup"]["plugin"] == "nav2_behaviors/BackUp"
    assert behavior_server["wait"]["plugin"] == "nav2_behaviors/Wait"
    assert behavior_server["transform_tolerance"] == 0.1
    assert "transform_timeout" not in behavior_server


def test_velocity_smoother_and_smoother_server_use_sim_time():
    params = _nav2_params()

    assert params["velocity_smoother"]["ros__parameters"]["use_sim_time"] is True
    smoother_server = params["smoother_server"]["ros__parameters"]
    assert smoother_server["use_sim_time"] is True
    assert smoother_server["smoother_plugins"] == ["simple_smoother"]
    assert (
        smoother_server["simple_smoother"]["plugin"]
        == "nav2_smoother::SimpleSmoother"
    )


def test_dwb_uses_obstacle_footprint_critic_with_existing_scale():
    params = _nav2_params()
    follow_path = params["controller_server"]["ros__parameters"]["FollowPath"]

    assert "ObstacleFootprint" in follow_path["critics"]
    assert "BaseObstacle" not in follow_path["critics"]
    assert follow_path["ObstacleFootprint.scale"] == 0.02
    assert "BaseObstacle.scale" not in follow_path


def test_local_costmap_has_no_unused_static_layer_block():
    params = _nav2_params()
    local_costmap = params["local_costmap"]["local_costmap"]["ros__parameters"]

    assert local_costmap["plugins"] == ["voxel_layer", "inflation_layer"]
    assert "static_layer" not in local_costmap


def test_costmaps_match_git_main_box_mecanum_footprint():
    params = _nav2_params()
    local = params["local_costmap"]["local_costmap"]["ros__parameters"]
    global_params = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert local["footprint"] == (
        "[ [0.28, 0.31], [0.28, -0.31], [-0.28, -0.31], [-0.28, 0.31] ]"
    )
    assert local["inflation_layer"]["inflation_radius"] == 0.55
    assert global_params["robot_radius"] == 0.42
    assert global_params["inflation_layer"]["inflation_radius"] == 0.55


def test_global_costmap_does_not_block_slam_unknown_cells_in_known_lab():
    params = _nav2_params()
    global_costmap = params["global_costmap"]["global_costmap"]["ros__parameters"]

    assert global_costmap["track_unknown_space"] is False
    assert global_costmap["static_layer"]["track_unknown_space"] is False


def test_ekf_uses_default_rejection_thresholds():
    ekf_file = Path(__file__).resolve().parents[1] / "config" / "ekf.yaml"
    ekf_params = yaml.safe_load(ekf_file.read_text(encoding="utf-8"))
    ros_params = ekf_params["ekf_filter_node"]["ros__parameters"]

    assert not any("rejection_threshold" in key for key in ros_params)
