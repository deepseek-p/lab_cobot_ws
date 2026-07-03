"""Contracts for the simulated gripper attach bridge."""
from pathlib import Path

import pytest

from lab_cobot_bringup import gripper_attach_bridge
from gazebo_msgs.srv import GetLinkProperties
from geometry_msgs.msg import Pose


BRINGUP = Path(__file__).resolve().parents[1]


def test_gripper_attach_bridge_declares_expected_topics_and_tcp():
    bridge = (
        BRINGUP / "lab_cobot_bringup" / "gripper_attach_bridge.py"
    ).read_text(encoding="utf-8")

    assert "/gripper/attach/aruco_sample" in bridge
    assert "/gripper/detach/aruco_sample" in bridge
    assert "gripper_tcp" in bridge
    assert "/gazebo/model_states" in bridge
    assert "SetEntityState" in bridge
    assert 'TF_REFERENCE_FRAME = "odom"' in bridge
    assert 'GAZEBO_REFERENCE_FRAME = "world"' in bridge
    assert 'declare_parameter("tf_reference_frame", TF_REFERENCE_FRAME)' in bridge
    assert (
        'declare_parameter("gazebo_reference_frame", GAZEBO_REFERENCE_FRAME)'
        in bridge
    )


def test_gripper_attach_bridge_uses_grasp_validator_before_attaching():
    bridge = (
        BRINGUP / "lab_cobot_bringup" / "gripper_attach_bridge.py"
    ).read_text(encoding="utf-8")

    assert "GraspValidationConfig" in bridge
    assert "validate_tcp_object_grasp" in bridge
    assert "self._grasp_config" in bridge
    assert (
        "validation = validate_tcp_object_grasp(offset_tcp, self._grasp_config)"
        in bridge
    )
    assert "refusing attach" in bridge
    assert "validation.reason" in bridge


def test_gripper_attach_bridge_declares_grasp_validation_parameters():
    bridge = (
        BRINGUP / "lab_cobot_bringup" / "gripper_attach_bridge.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("grasp.max_center_distance_m", 0.080)' in bridge
    assert 'declare_parameter("grasp.max_abs_x_m", 0.040)' in bridge
    assert 'declare_parameter("grasp.max_abs_y_m", 0.018)' in bridge
    assert 'declare_parameter("grasp.min_z_m", -0.060)' in bridge
    assert 'declare_parameter("grasp.max_z_m", 0.025)' in bridge


def test_gripper_attach_bridge_updates_fast_enough_to_reduce_attached_jitter():
    bridge = (
        BRINGUP / "lab_cobot_bringup" / "gripper_attach_bridge.py"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter("update_rate", 120.0)' in bridge


def test_gripper_attach_bridge_publishes_attach_status():
    bridge = (
        BRINGUP / "lab_cobot_bringup" / "gripper_attach_bridge.py"
    ).read_text(encoding="utf-8")

    assert 'ATTACH_STATUS_TOPIC = "/gripper/attach/status"' in bridge
    assert "String" in bridge
    assert "self._attach_status_pub" in bridge
    assert 'self._publish_attach_status("attached")' in bridge
    assert 'self._publish_attach_status("refused", validation.reason)' in bridge
    assert (
        'self._publish_attach_status("refused", "missing_tcp_or_model_pose")'
        in bridge
    )


def test_gripper_attach_bridge_is_installed_with_gazebo_dependency():
    cmake = (BRINGUP / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (BRINGUP / "package.xml").read_text(encoding="utf-8")

    assert "gripper_attach_bridge.py" in cmake
    assert "RENAME gripper_attach_bridge" in cmake
    assert "<exec_depend>gazebo_msgs</exec_depend>" in package


def test_gazebo_world_loads_state_service_plugin():
    world = (
        BRINGUP.parent / "lab_cobot_gazebo" / "worlds" / "lab.world"
    ).read_text(encoding="utf-8")

    assert "libgazebo_ros_state.so" in world


def test_gazebo_world_loads_properties_plugin_for_attach_physics():
    world = (
        BRINGUP.parent / "lab_cobot_gazebo" / "worlds" / "lab.world"
    ).read_text(encoding="utf-8")

    assert "libgazebo_ros_properties.so" in world
    assert "<namespace>/gazebo</namespace>" in world


def test_gazebo_launch_spawns_gripper_controller():
    launch_file = (
        BRINGUP.parent / "lab_cobot_gazebo" / "launch" / "world.launch.py"
    ).read_text(encoding="utf-8")

    assert "gripper_position_controller" in launch_file
    assert (
        'arguments=["gripper_position_controller", "-c", "/controller_manager"]'
        in launch_file
    )


def test_attach_bridge_holds_valid_grasp_at_visual_tcp_center():
    assert hasattr(gripper_attach_bridge, "hold_offset_for_validated_grasp")

    offset = gripper_attach_bridge.hold_offset_for_validated_grasp(
        (0.030, -0.009, 0.020)
    )

    assert offset == pytest.approx((0.0, 0.0, 0.0))


def test_attach_bridge_entity_state_zeroes_twist_to_reduce_jitter():
    assert hasattr(gripper_attach_bridge, "make_attached_entity_state")

    pose = Pose()
    pose.position.x = 1.0
    pose.position.y = 2.0
    pose.position.z = 3.0
    pose.orientation.w = 1.0

    state = gripper_attach_bridge.make_attached_entity_state(
        name="aruco_sample",
        pose=pose,
        reference_frame="world",
    )

    assert state.name == "aruco_sample"
    assert state.reference_frame == "world"
    assert state.pose.position.x == pytest.approx(1.0)
    assert state.pose.position.y == pytest.approx(2.0)
    assert state.pose.position.z == pytest.approx(3.0)
    assert state.twist.linear.x == pytest.approx(0.0)
    assert state.twist.linear.y == pytest.approx(0.0)
    assert state.twist.linear.z == pytest.approx(0.0)
    assert state.twist.angular.x == pytest.approx(0.0)
    assert state.twist.angular.y == pytest.approx(0.0)
    assert state.twist.angular.z == pytest.approx(0.0)


def test_attach_bridge_preserves_link_properties_when_disabling_gravity():
    assert hasattr(gripper_attach_bridge, "capture_link_properties")
    assert hasattr(gripper_attach_bridge, "make_link_properties_request")

    response = GetLinkProperties.Response()
    response.success = True
    response.com.position.x = 0.01
    response.com.position.y = 0.02
    response.com.position.z = 0.03
    response.com.orientation.w = 1.0
    response.gravity_mode = True
    response.mass = 0.05
    response.ixx = 4.0e-5
    response.ixy = 1.0e-6
    response.ixz = 2.0e-6
    response.iyy = 5.0e-5
    response.iyz = 3.0e-6
    response.izz = 6.0e-5

    snapshot = gripper_attach_bridge.capture_link_properties(response)
    request = gripper_attach_bridge.make_link_properties_request(
        "aruco_sample::link",
        snapshot,
        gravity_mode=False,
    )

    assert snapshot.gravity_mode is True
    assert request.link_name == "aruco_sample::link"
    assert request.gravity_mode is False
    assert request.com.position.x == pytest.approx(0.01)
    assert request.com.position.y == pytest.approx(0.02)
    assert request.com.position.z == pytest.approx(0.03)
    assert request.mass == pytest.approx(0.05)
    assert request.ixx == pytest.approx(4.0e-5)
    assert request.ixy == pytest.approx(1.0e-6)
    assert request.ixz == pytest.approx(2.0e-6)
    assert request.iyy == pytest.approx(5.0e-5)
    assert request.iyz == pytest.approx(3.0e-6)
    assert request.izz == pytest.approx(6.0e-5)


def test_attach_bridge_ignores_failed_link_property_reads():
    response = GetLinkProperties.Response()
    response.success = False

    assert gripper_attach_bridge.capture_link_properties(response) is None


def test_attach_bridge_waits_until_gazebo_services_are_ready():
    assert hasattr(gripper_attach_bridge, "wait_until_gazebo_ready")

    attempts = []

    class FakeLogger:
        def warn(self, message):
            attempts.append(message)

    class FakeNode:
        def __init__(self):
            self.results = iter([False, False, True])

        def wait_for_gazebo(self, timeout_sec):
            attempts.append(timeout_sec)
            return next(self.results)

        def get_logger(self):
            return FakeLogger()

    assert gripper_attach_bridge.wait_until_gazebo_ready(
        FakeNode(),
        ok=lambda: True,
        retry_timeout_sec=0.1,
    )
    assert attempts.count(0.1) == 3
