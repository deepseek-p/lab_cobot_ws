"""Contracts for the independent grasp validation node."""

import inspect
import math

import pytest

import lab_cobot_bringup.grasp_validation_node as grasp_validation_node
from lab_cobot_bringup.grasp_target_config import GRASP_TARGETS
from lab_cobot_bringup.grasp_validation_node import (
    BASE_FOOTPRINT_ENTITY,
    BASE_FOOTPRINT_FRAME,
    DESCEND_MAX_JOINT_DELTA_PER_STEP,
    DESCEND_MAX_TOTAL_JOINT_DELTA,
    FAILURE_STATUSES,
    STATUS_TOPIC,
    SUCCESS_STATUSES,
    TARGET_TOPIC,
    compose_child_world_pose,
    board_base_block_long_axis_world_yaw,
    board_base_block_world_center,
    configured_grasp_base_pose,
    configured_grasp_base_yaw,
    configured_grasp_quat_and_yaw,
    configured_grasp_rpy,
    down_quat_for_yaw,
    grasp_z_debug_values,
    handle_long_axis_world_yaw,
    validate_target_config,
    world_pose_to_base_pose,
    world_to_base_xy,
)


def test_grasp_validation_topics_are_independent_from_mission_topics():
    assert TARGET_TOPIC == "/grasp_validation/target"
    assert STATUS_TOPIC == "/grasp_validation/status"


def test_grasp_validation_status_contract_lists_expected_states():
    for status in (
        "READY",
        "TARGET_CONFIGURED",
        "PLANNING",
        "PRE_GRASP_REACHED",
        "DESCENDING",
        "GRIPPING",
        "CONTACT_OK",
        "ATTACHED",
        "LIFTING",
        "HOLDING",
        "SUCCESS",
    ):
        assert status in SUCCESS_STATUSES

    for status in (
        "FAILED_UNKNOWN_TARGET",
        "FAILED_TARGET_CONFIG",
        "FAILED_TARGET_POSE",
        "FAILED_TRANSFORM",
        "FAILED_PRE_GRASP_PLAN",
        "FAILED_PRE_GRASP_EXEC",
        "FAILED_DESCEND",
        "FAILED_CONTACT",
        "FAILED_ATTACH",
        "FAILED_LIFT",
        "FAILED_HOLD_LOST",
        "FAILED_EXCEPTION",
    ):
        assert status in FAILURE_STATUSES


def test_validation_sets_cartesian_avoid_collisions_for_descend():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._plan_validation_cartesian_descend
    )

    assert "self._set_cartesian_avoid_collisions(True)" in source


def test_validation_descend_plans_once_then_executes_same_trajectory():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._execute_configured_grasp
    )

    assert "descend_trajectory = self._plan_validation_cartesian_descend" in source
    assert "self._execute_validation_descend(descend_trajectory)" in source
    assert "self.pp._move(  # noqa: SLF001\n            tcp_target" not in source


def test_validation_descend_sets_nonzero_cartesian_jump_threshold():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._plan_validation_cartesian_descend
    )

    assert "_set_cartesian_jump_threshold" in source
    assert "DESCEND_CARTESIAN_JUMP_THRESHOLD" in source


def test_validation_descend_logs_state_validity_diagnostics():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._plan_validation_cartesian_descend
    )

    assert "_log_descend_trajectory_extent" in source
    assert "_log_state_validity_for_current_joints" in source
    assert "_log_state_validity_for_trajectory_endpoint" in source
    assert "FIRST_INVALID_COLLISION_PAIR" in inspect.getsource(
        grasp_validation_node.GraspValidationNode._log_state_validity
    )


def test_board_z_adjust_logs_before_after_targets():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._log_world_base_target_geometry
    )
    execute_source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._execute_configured_grasp
    )

    assert "GRASP_LOCAL_POINT_FINAL" in source
    assert "GRASP_WORLD_POINT_FINAL" in source
    assert "FINAL_GRASP_RPY" in source
    assert "_log_descend_tcp_poses" in execute_source
    assert "GRASP_Z_ADJUST" in source
    assert "OLD_BOARD_GRASP_Z" in source
    assert "NEW_BOARD_GRASP_Z" in source
    assert "OLD_TCP_TARGET_Z" in source
    assert "NEW_TCP_TARGET_Z" in source


def test_validation_descend_tcp_pose_logs_start_and_end():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._log_descend_tcp_poses
    )

    assert "DESCEND_START_TCP_POSE" in source
    assert "DESCEND_END_TCP_POSE" in source


def test_coordinate_consistency_diagnostics_do_not_fake_gazebo_base_link():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._gazebo_frame_world_z_no_fallback
    )
    base_source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._get_base_link_world_pose
    )
    footprint_source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._get_base_footprint_world_pose
    )

    assert "_get_entity_world_pose(f\"{ROBOT_ENTITY_NAME}::{frame}\")" in source
    assert "model_state" not in source
    assert "_get_base_footprint_world_pose(config)" in base_source
    assert "_get_base_link_pose_in_base_footprint()" in base_source
    assert "_get_entity_world_pose(BASE_FOOTPRINT_ENTITY)" in footprint_source
    assert "_get_entity_world_pose(BASE_LINK" not in base_source


def test_elbow_branch_flip_exceeds_descend_continuity_limits():
    total = abs(1.2762 - (-0.7501))

    assert total > DESCEND_MAX_TOTAL_JOINT_DELTA["ur_elbow_joint"]
    assert DESCEND_MAX_JOINT_DELTA_PER_STEP < total


def test_world_to_base_uses_full_yaw_transform():
    x_base, y_base = world_to_base_xy(
        world_x=1.0,
        world_y=2.0,
        base_x=1.0,
        base_y=1.0,
        base_yaw=1.57079632679,
    )

    assert x_base == pytest.approx(1.0)
    assert y_base == pytest.approx(0.0, abs=1e-9)


def test_configured_grasp_base_pose_is_finite_for_all_targets():
    for target, config in GRASP_TARGETS.items():
        validate_target_config(target, config)
        pose = configured_grasp_base_pose(config)
        assert len(pose) == 3
        assert all(abs(value) < 5.0 for value in pose)


def test_configured_grasp_yaw_is_relative_to_validation_base():
    config = GRASP_TARGETS["material_spare_igbt"]

    yaw = configured_grasp_base_yaw(config)
    quat = down_quat_for_yaw(yaw)

    assert yaw == pytest.approx(0.38, abs=1e-6)
    assert len(quat) == 4
    assert sum(value * value for value in quat) == pytest.approx(1.0)


def test_material_spare_igbt_uses_caliper_main_ruler_grasp():
    config = GRASP_TARGETS["material_spare_igbt"]
    inferred_yaw = configured_grasp_base_yaw(config)

    quat, final_yaw, rpy = configured_grasp_quat_and_yaw(config, inferred_yaw)

    assert config["grasp_point_local"] == {
        "x": 0.01580000,
        "y": -0.05290000,
        "z": 0.00017000,
    }
    assert config["grasp_region_label"] == "main_ruler_mid_body"
    assert config["gripper_closing_axis_local"] == "model_local_x"
    assert config["expected_grasp_width_mm"] == pytest.approx(23.79)
    assert rpy == pytest.approx((3.141592653589793, 0.0, 0.38))
    assert configured_grasp_rpy(config, inferred_yaw) == pytest.approx(rpy)
    assert final_yaw == pytest.approx(0.38)
    assert quat == pytest.approx(
        [0.9820042351172703, 0.18885889497650052, 0.0, 0.0]
    )


def test_high_voltage_grasp_debug_reports_object_geometry_only():
    config = GRASP_TARGETS["high_voltage_probe_kit"]
    grasp_base = configured_grasp_base_pose(config)
    tactile_tcp_z = grasp_base[2] + 0.018
    values = grasp_z_debug_values(config, tactile_tcp_z)

    assert values["OBJECT_CENTER_Z"] == pytest.approx(0.7889984341574722)
    assert values["OBJECT_TOP_Z"] == pytest.approx(0.8289984341574722)
    assert values["TCP_TARGET_Z"] == pytest.approx(grasp_base[2] + 0.018)
    assert "FINGER_CONTACT_PLANE_Z" not in values


def test_high_voltage_object_grasp_point_then_production_tactile_clearance():
    config = GRASP_TARGETS["high_voltage_probe_kit"]
    object_world_pose = {
        "x": -4.30,
        "y": -2.60,
        "z": 0.7489984341574721,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": -0.15,
    }
    base_world_pose = {
        "x": -4.30,
        "y": -3.21,
        "z": 0.155,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 1.57079632679,
    }

    grasp_base = configured_grasp_base_pose(
        config,
        base_world_pose=base_world_pose,
        object_world_pose=object_world_pose,
    )
    tcp_z = grasp_base[2] + 0.018

    assert config["handle_grasp_local_point"]["x"] == pytest.approx(0.082130309)
    assert config["handle_grasp_local_point"]["y"] == pytest.approx(0.0)
    assert handle_long_axis_world_yaw(config, object_world_pose) == pytest.approx(
        -0.15
    )
    assert grasp_base[0] == pytest.approx(0.5977266000035604)
    assert grasp_base[1] == pytest.approx(-0.08120807416115024)
    assert grasp_base[2] == pytest.approx(0.6339984341574721)
    assert tcp_z == pytest.approx(0.6519984341574721)
    assert tcp_z + base_world_pose["z"] == pytest.approx(0.8069984341574721)


def test_board_fixture_base_block_grasp_point_then_production_tactile_clearance():
    config = GRASP_TARGETS["board_test_fixture"]
    object_world_pose = {
        "x": -4.70,
        "y": -2.60,
        "z": 0.7776,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.05,
    }
    base_world_pose = {
        "x": -4.596977732335393,
        "y": -3.212689253512364,
        "z": 0.155,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 1.57079632679,
    }

    block_world = board_base_block_world_center(config, object_world_pose)
    grasp_base = configured_grasp_base_pose(
        config,
        base_world_pose=base_world_pose,
        object_world_pose=object_world_pose,
    )
    tcp_z = grasp_base[2] + 0.018
    yaw = configured_grasp_base_yaw(
        config,
        base_world_pose=base_world_pose,
        object_world_pose=object_world_pose,
    )

    assert block_world["x"] == pytest.approx(-4.61387160460737)
    assert block_world["y"] == pytest.approx(-2.668743786179849)
    assert block_world["z"] == pytest.approx(0.7775915)
    assert board_base_block_long_axis_world_yaw(
        config,
        object_world_pose,
    ) == pytest.approx(0.05)
    assert grasp_base[0] == pytest.approx(0.5439454673325153)
    assert grasp_base[1] == pytest.approx(0.016893872271976952)
    assert grasp_base[2] == pytest.approx(0.6336)
    assert yaw == pytest.approx(0.05)
    assert tcp_z == pytest.approx(0.6516)
    assert tcp_z + 0.110 == pytest.approx(0.7616)


def test_world_pose_to_base_pose_uses_xyz_and_yaw():
    base_pose = {"x": 1.0, "y": 1.0, "z": 0.155, "yaw": 1.57079632679}
    world_pose = {"x": 1.0, "y": 2.0, "z": 0.790, "yaw": 1.57079632679}

    base_pose_result = world_pose_to_base_pose(world_pose, base_pose)

    assert base_pose_result["x"] == pytest.approx(1.0)
    assert base_pose_result["y"] == pytest.approx(0.0, abs=1e-9)
    assert base_pose_result["z"] == pytest.approx(0.635)
    assert base_pose_result["yaw"] == pytest.approx(0.0)


def test_validation_base_link_world_pose_is_composed_from_base_footprint():
    base_footprint_world = {
        "x": 1.0,
        "y": 2.0,
        "z": 0.0,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 1.57079632679,
    }
    base_link_in_footprint = {
        "x": 0.10,
        "y": 0.20,
        "z": 0.155,
        "roll": 0.0,
        "pitch": 0.0,
        "yaw": 0.0,
    }

    base_link_world = compose_child_world_pose(
        base_footprint_world,
        base_link_in_footprint,
    )

    assert base_link_world["x"] == pytest.approx(0.80)
    assert base_link_world["y"] == pytest.approx(2.10)
    assert base_link_world["z"] == pytest.approx(0.155)
    assert base_link_world["yaw"] == pytest.approx(1.57079632679)


def test_get_base_link_world_pose_keeps_base_footprint_z_offset():
    class FakeNode:
        def _get_base_footprint_world_pose(self, _config):
            return {
                "x": 1.0,
                "y": 2.0,
                "z": 0.0,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 1.57079632679,
            }

        def _get_base_link_pose_in_base_footprint(self):
            return {
                "x": 0.10,
                "y": 0.20,
                "z": 0.155,
                "roll": 0.0,
                "pitch": 0.0,
                "yaw": 0.0,
            }

    pose = grasp_validation_node.GraspValidationNode._get_base_link_world_pose(
        FakeNode(),
        {},
    )

    assert pose["x"] == pytest.approx(0.80)
    assert pose["y"] == pytest.approx(2.10)
    assert pose["z"] == pytest.approx(0.155)
    assert pose["yaw"] == pytest.approx(1.57079632679)


def test_validation_does_not_query_nonexistent_gazebo_base_link_entity():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._get_base_link_world_pose
    )
    footprint_source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._get_base_footprint_world_pose
    )

    assert BASE_FOOTPRINT_FRAME == "base_footprint"
    assert BASE_FOOTPRINT_ENTITY == "lab_cobot::base_footprint"
    assert "lab_cobot::base_link" not in source
    assert "_get_entity_world_pose(BASE_LINK" not in source
    assert "_get_entity_world_pose(BASE_FOOTPRINT_ENTITY)" in footprint_source
    assert "base_link will be derived through TF" in footprint_source


def test_remaining_targets_are_spawned_into_screwdriver_style_workspace():
    def expected_base_xy_from_raw_config(config):
        # Fixture intentionally uses an off-center local grasp point
        # grasp_point_local.y = 0.06; compute the expected base-frame point
        # from raw config fields instead of hard-coding the old center-grasp
        # x=0.65, y=0.0 baseline.
        point = config["grasp_point_local"]
        object_pose = config["world_pose"]
        object_yaw = float(object_pose["yaw"])
        grasp_world_x = (
            float(object_pose["x"])
            + math.cos(object_yaw) * float(point["x"])
            - math.sin(object_yaw) * float(point["y"])
        )
        grasp_world_y = (
            float(object_pose["y"])
            + math.sin(object_yaw) * float(point["x"])
            + math.cos(object_yaw) * float(point["y"])
        )
        base_pose = config["validation_base_pose"]
        dx = grasp_world_x - float(base_pose["x"])
        dy = grasp_world_y - float(base_pose["y"])
        base_yaw = float(base_pose["yaw"])
        return (
            math.cos(base_yaw) * dx + math.sin(base_yaw) * dy,
            -math.sin(base_yaw) * dx + math.cos(base_yaw) * dy,
        )

    expected = {
        "material_spare_igbt": (0.5667341841947278, -0.03429459407405574),
    }

    fixture_config = GRASP_TARGETS["tooling_fixture_box"]
    fixture_expected_x, fixture_expected_y = expected_base_xy_from_raw_config(
        fixture_config
    )
    fixture_grasp_base = configured_grasp_base_pose(fixture_config)
    assert fixture_grasp_base[0] == pytest.approx(fixture_expected_x)
    assert fixture_grasp_base[1] == pytest.approx(fixture_expected_y, abs=1e-8)

    for target, (expected_x, expected_y) in expected.items():
        config = GRASP_TARGETS[target]
        grasp_base = configured_grasp_base_pose(config)

        assert grasp_base[0] == pytest.approx(expected_x)
        assert grasp_base[1] == pytest.approx(expected_y, abs=1e-8)

    hand_tools_config = GRASP_TARGETS["tooling_hand_tools"]
    hand_tools_grasp_base = configured_grasp_base_pose(hand_tools_config)
    assert all(math.isfinite(value) for value in hand_tools_grasp_base)
    assert 0.50 <= hand_tools_grasp_base[0] <= 0.75
    assert abs(hand_tools_grasp_base[1]) <= 0.12
    assert 0.45 <= hand_tools_grasp_base[2] <= 0.85


def test_validation_execution_uses_pickplace_target_conversion():
    source = inspect.getsource(
        grasp_validation_node.GraspValidationNode._execute_configured_grasp
    )

    assert "self.pp._pick_tcp_target(grasp_base)" in source
    assert "self.pp._pick_approach_target(tcp_target)" in source
    assert "VALIDATION_OBJECT_GRASP_POINT_BASE" in inspect.getsource(
        grasp_validation_node.GraspValidationNode._log_validation_pick_targets
    )
