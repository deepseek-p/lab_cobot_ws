"""Mission navigation handoff policy tests."""
import math

import pytest

from lab_cobot_bringup import mission_node


def _policy(name):
    assert hasattr(mission_node, name), f"{name} policy is missing"
    return getattr(mission_node, name)


def test_pick_navigation_can_handoff_when_visual_docking_can_finish_alignment():
    pick_navigation_handoff_ready = _policy("pick_navigation_handoff_ready")

    assert pick_navigation_handoff_ready([0.80, 0.10, 0.63])


def test_pick_navigation_keeps_nav2_for_gui_failed_lateral_offset():
    pick_navigation_handoff_ready = _policy("pick_navigation_handoff_ready")

    assert not pick_navigation_handoff_ready([0.793, 0.235, 0.63])


def test_pick_navigation_keeps_nav2_when_object_is_too_lateral_for_docking():
    pick_navigation_handoff_ready = _policy("pick_navigation_handoff_ready")

    assert not pick_navigation_handoff_ready([0.87, 0.52, 0.63])


def test_place_navigation_can_handoff_when_tcp_target_is_on_station_b_table():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.95, 0.36, math.radians(80.0))

    assert place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_navigation_can_handoff_when_base_is_close_enough_for_local_docking():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.80, 0.47, math.radians(147.0))

    assert place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_navigation_keeps_nav2_until_place_target_reaches_table_front():
    place_navigation_handoff_ready = _policy("place_navigation_handoff_ready")
    base_pose = (-1.71, 0.22, math.radians(79.0))

    assert not place_navigation_handoff_ready(base_pose, mission_node.DEFAULT_PLACE_POSE)


def test_place_dock_velocity_drives_forward_and_rotates_toward_station_b_pose():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.00, 0.35, math.radians(106.0)))

    assert not done
    assert cmd.linear.x > 0.0
    assert cmd.angular.z < 0.0


def test_place_dock_velocity_stops_when_base_is_aligned_for_table_place():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.01, 0.46, math.radians(88.0)))

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


def test_place_dock_velocity_accepts_gui_verified_table_pose():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-1.995, 0.454, math.radians(94.7)))

    assert done
    assert cmd.linear.x == pytest.approx(0.0)
    assert cmd.linear.y == pytest.approx(0.0)
    assert cmd.angular.z == pytest.approx(0.0)


def test_place_dock_velocity_keeps_moving_when_drop_point_is_before_table():
    place_dock_velocity_for_base = _policy("place_dock_velocity_for_base")

    done, cmd = place_dock_velocity_for_base((-2.00, 0.40, math.radians(100.0)))

    assert not done
    assert cmd.linear.x > 0.0


def test_home_navigation_can_handoff_when_map_pose_is_close_enough():
    home_navigation_handoff_ready = _policy("home_navigation_handoff_ready")

    assert home_navigation_handoff_ready((-0.02, 0.16, math.radians(17.6)))


def test_home_navigation_keeps_nav2_when_pose_is_still_far_from_home():
    home_navigation_handoff_ready = _policy("home_navigation_handoff_ready")

    assert not home_navigation_handoff_ready((0.09, 0.38, math.radians(23.0)))
