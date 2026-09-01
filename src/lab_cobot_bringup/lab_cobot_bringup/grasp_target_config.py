"""Fixed target configuration for tooling-zone grasp validation."""
from __future__ import annotations

import math


TOOLING_TARGET_NAMES = (
    "tooling_fixture_box",
    "tooling_hand_tools",
    "board_test_fixture",
    "high_voltage_probe_kit",
    "material_spare_igbt",
)


GRASP_TARGETS = {
    "tooling_fixture_box": {
        "station": "tooling_zone",
        "entity_name": "tooling_fixture_box",
        "model_uri": "model://fixture_box_plain",
        "link_name": "link",
        "world_pose": {"x": -3.88, "y": -2.04, "z": 0.758, "yaw": -0.28},
        # North-side validation-only pose; target x is 0.65m to preserve table
        # clearance while keeping the arm in the screwdriver-style workspace.
        "validation_base_pose": {"x": -3.88, "y": -1.39, "yaw": -math.pi / 2.0},
        "grasp_region_label": "central_adjustable_wrench_handle_shank",
        "grasp_point_local": {"x": 0.0, "y": 0.06000000, "z": 0.00007236},
        "grasp_region_size": (0.10411212, 0.31999992, 0.01614453),
        "grasp_long_axis_yaw": math.pi / 2.0,
        "grasp_offset": {"x": 0.0, "y": 0.06000000, "z": 0.00007236},
        "grasp_yaw": math.pi / 2.0,
        "pre_grasp_height": 0.105,
        "tcp_clearance": 0.020,
        "tactile_target_force_n": 2.0,
        "tactile_max_force_n": 18.0,
        "descend_distance": 0.105,
        "lift_distance": 0.115,
        "collision_size": (0.120, 0.340, 0.200),
        "carried_collision_size": (0.120, 0.340, 0.160),
        "attach_envelope": {
            "max_center_distance": 0.220,
            "max_abs_x": 0.125,
            "max_abs_y": 0.105,
            "min_z": -0.060,
            "max_z": 0.220,
        },
    },
    "tooling_hand_tools": {
        "station": "tooling_zone",
        "entity_name": "tooling_hand_tools",
        "model_uri": "model://tooling_hand_tools",
        "link_name": "link",
        "world_pose": {"x": -4.36, "y": -1.96, "z": 0.761, "yaw": 0.12},
        # North-side validation-only pose; the white handle/body region is
        # centered in front of base_link rather than reaching from the south.
        "validation_base_pose": {
            "x": -4.353153509224888,
            "y": -1.3767799668952772,
            "yaw": -math.pi / 2.0,
        },
        "grasp_region_label": "closed_pliers_white_handle_body",
        "grasp_point_local": {
            "x": 0.0,
            "y": -0.08700000,
            "z": -0.00016920,
        },
        "grasp_region_size": (0.10398028, 0.20561742, 0.02166162),
        "grasp_long_axis_yaw": math.pi / 2.0,
        "grasp_offset": {"x": 0.0, "y": -0.08700000, "z": -0.00016920},
        "grasp_yaw": math.pi / 2.0,
        "pre_grasp_height": 0.105,
        "tcp_clearance": 0.018,
        "tactile_target_force_n": 2.0,
        "tactile_max_force_n": 18.0,
        "descend_distance": 0.105,
        "lift_distance": 0.115,
        "collision_size": (0.120, 0.340, 0.048),
        "carried_collision_size": (0.120, 0.340, 0.048),
        "attach_envelope": {
            "max_center_distance": 0.220,
            "max_abs_x": 0.125,
            "max_abs_y": 0.105,
            "min_z": -0.060,
            "max_z": 0.220,
        },
    },
    "board_test_fixture": {
        "station": "tooling_zone",
        "entity_name": "board_test_fixture",
        "model_uri": "model://pcb_test_fixture",
        "link_name": "link",
        "world_pose": {"x": -4.70, "y": -2.60, "z": 0.7776, "yaw": 0.05},
        # validation-only initial estimate, tune from runtime reach result
        "validation_base_pose": {
            "x": -4.596977732335393,
            "y": -3.212689253512364,
            "yaw": math.pi / 2.0,
        },
        # Bottom coarse light/beige rectangular block from pcb_test_fixture
        # drill.dae Acetal_Resin__White + Paek__Beige components, after the
        # DAE node matrix, model.sdf scale 2 2 2, and visual/collision pose
        # z=-0.0276.  This is intentionally not the central ABS__White
        # connector component.
        "base_block_local_center": {
            "x": 0.082585,
            "y": -0.0729625,
            "z": -0.0000085,
        },
        "base_block_size": (0.128356, 0.054419, 0.055183),
        "base_block_local_rpy": (0.0, 0.0, 0.0),
        "base_block_long_axis_yaw": 0.0,
        "grasp_candidate_1": {
            "label": "bottom_light_beige_coarse_rectangular_block",
            "local_center": {
                "x": 0.082585,
                "y": -0.0729625,
                "z": -0.0000085,
            },
            "size": (0.128356, 0.054419, 0.055183),
            "local_rpy": (0.0, 0.0, 0.0),
            "long_axis_yaw": 0.0,
            "grasp_local_point": {
                "x": 0.082585,
                "y": -0.0729625,
                "z": 0.006,
            },
            "grasp_yaw": math.pi / 2.0,
        },
        "grasp_candidate_2": {
            "label": "right_yellow_thick_cylindrical_handle",
            "local_center": {
                "x": 0.09249197,
                "y": 0.07339262,
                "z": -0.00000844,
            },
            "size": (0.11501598, 0.05355908, 0.05355885),
            "long_axis_yaw": 0.0,
            "grasp_local_point": {
                "x": 0.09249197,
                "y": 0.07339262,
                "z": 0.006,
            },
            "grasp_yaw": 0.0,
        },
        "grasp_offset": {"x": 0.082585, "y": -0.0729625, "z": 0.006},
        "grasp_z_adjust": 0.005,
        "grasp_yaw": math.pi / 2.0,
        "pre_grasp_height": 0.110,
        "tcp_clearance": 0.020,
        "tactile_target_force_n": 2.0,
        "tactile_max_force_n": 18.0,
        "descend_distance": 0.110,
        "lift_distance": 0.120,
        "collision_size": (0.320, 0.220, 0.100),
        "carried_collision_size": (0.300, 0.200, 0.090),
        "attach_envelope": {
            "max_center_distance": 0.220,
            "max_abs_x": 0.125,
            "max_abs_y": 0.105,
            "min_z": -0.060,
            "max_z": 0.220,
        },
    },
    "high_voltage_probe_kit": {
        "station": "tooling_zone",
        "entity_name": "high_voltage_probe_kit",
        "model_uri": "model://safety_probe_kit",
        "link_name": "link",
        "world_pose": {
            "x": -4.30,
            "y": -2.60,
            "z": 0.7489984341574721,
            "yaw": -0.15,
        },
        # validation-only initial estimate, tune from runtime reach result
        "validation_base_pose": {"x": -4.30, "y": -3.21, "yaw": math.pi / 2.0},
        "collision_local_pose": {"x": 0.0, "y": 0.0, "z": 0.040, "yaw": 0.0},
        # Red handle thick-body center from safety_probe_kit screwdriver.dae,
        # after the model.sdf mesh scale of 2 2 2.  Keep z on the existing
        # validation chain for this pass; only move XY to the handle body.
        "handle_grasp_local_point": {
            "x": 0.082130309,
            "y": 0.0,
            "z": 0.02671527,
        },
        "handle_long_axis_yaw": 0.0,
        "grasp_offset": {"x": 0.082130309, "y": 0.0, "z": 0.040},
        "grasp_yaw": 0.0,
        "pre_grasp_height": 0.100,
        "tcp_clearance": 0.000,
        "tactile_target_force_n": 2.0,
        "tactile_max_force_n": 18.0,
        "descend_distance": 0.100,
        "lift_distance": 0.110,
        # Validation-only final TCP adjustment for the safety-probe handle.
        # This lowers only the final grasp pose; grasp XY/yaw and the object
        # grasp point remain unchanged.
        "validation_grasp_tcp_z_adjust": -0.010,
        "preserve_pre_grasp_z_on_tcp_z_adjust": True,
        # The current handle grasp section is about 48.58 mm wide.  With the
        # parallel gripper geometry gap = 0.092 - 2q, first contact is around
        # q=0.02171 m.  Keep this target-specific tactile sweep below the URDF
        # mechanical limit while allowing enough margin to find real contact.
        "tactile_max_position": 0.025,
        "collision_size": (0.340, 0.070, 0.080),
        "carried_collision_size": (0.320, 0.070, 0.070),
        "attach_envelope": {
            "max_center_distance": 0.220,
            "max_abs_x": 0.125,
            "max_abs_y": 0.105,
            "min_z": -0.060,
            "max_z": 0.220,
        },
    },
    "material_spare_igbt": {
        "station": "tooling_zone",
        "entity_name": "material_spare_igbt",
        "model_uri": "model://igbt_module_plain",
        "link_name": "link",
        "world_pose": {"x": -3.62, "y": -2.60, "z": 0.753, "yaw": 0.38},
        # South-side validation-only pose matching the screwdriver-style
        # workspace while staying outside the tooling table footprint.
        "validation_base_pose": {"x": -3.62, "y": -3.21, "yaw": math.pi / 2.0},
        "grasp_region_label": "main_ruler_mid_body",
        "grasp_point_local": {"x": 0.01580000, "y": -0.05290000, "z": 0.00017000},
        "grasp_region_size": (0.02378844, 0.06000000, 0.00475767),
        "grasp_long_axis_yaw": math.pi / 2.0,
        "grasp_offset": {"x": 0.01580000, "y": -0.05290000, "z": 0.00017000},
        "grasp_yaw": math.pi / 2.0,
        "gripper_closing_axis_local": "model_local_x",
        "expected_grasp_width_mm": 23.79,
        "pre_grasp_position": 0.03010,
        "preopen_gap_mm": 31.80,
        "expected_contact_q": 0.03411,
        "tactile_start_position": 0.03010,
        "tactile_max_position": 0.03480,
        "grasp_z_adjust": 0.0,
        "pre_grasp_height": 0.080,
        "tcp_clearance": 0.013,
        "tactile_target_force_n": 2.0,
        "tactile_max_force_n": 18.0,
        "descend_distance": 0.080,
        "lift_distance": 0.115,
        "collision_size": (0.140, 0.380, 0.120),
        "carried_collision_size": (0.140, 0.360, 0.100),
        "attach_envelope": {
            "max_center_distance": 0.220,
            "max_abs_x": 0.125,
            "max_abs_y": 0.105,
            "min_z": -0.060,
            "max_z": 0.220,
        },
    },
}


def target_names() -> tuple[str, ...]:
    return tuple(GRASP_TARGETS)


def get_target_config(target: str) -> dict | None:
    return GRASP_TARGETS.get(str(target))


def validation_base_pose_for_target(target: str) -> dict:
    config = get_target_config(target)
    if config is None:
        raise ValueError(f"unknown validation_target: {target}")
    return dict(config["validation_base_pose"])
