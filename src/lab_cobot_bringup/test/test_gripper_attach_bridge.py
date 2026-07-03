"""Contracts for the simulated gripper attach bridge."""
import importlib.util
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest

from lab_cobot_bringup import gripper_attach_bridge
from lab_cobot_bringup.grasp_validator import (
    GraspValidationConfig,
    GraspValidationResult,
    validate_tcp_object_grasp,
)
from gazebo_msgs.srv import GetLinkProperties, SetLinkProperties
from geometry_msgs.msg import Pose, Quaternion


BRINGUP = Path(__file__).resolve().parents[1]
SRC = BRINGUP.parent


def _load_launch_module(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _package_share(pkg: str) -> str:
    if pkg.startswith("lab_cobot_"):
        return str(SRC / pkg)
    return f"/opt/ros/humble/share/{pkg}"


def _launch_entities(entity):
    yield entity
    actions = getattr(entity, "actions", None)
    if actions:
        for action in actions:
            yield from _launch_entities(action)

    handler = getattr(entity, "_RegisterEventHandler__event_handler", None)
    if handler is not None:
        on_event = getattr(handler, "_OnActionEventBase__actions_on_event", [])
        for action in on_event:
            yield from _launch_entities(action)


def _all_launch_entities(launch_description):
    for entity in launch_description.entities:
        yield from _launch_entities(entity)


def _node_arguments(node):
    return list(getattr(node, "_Node__arguments", []) or [])


class _Future:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done

    def done(self):
        return self._done

    def result(self):
        return self._result


class _Logger:
    def __init__(self):
        self.warnings = []

    def debug(self, *args, **kwargs):
        pass

    def info(self, *args, **kwargs):
        pass

    def warn(self, message, **kwargs):
        self.warning(message, **kwargs)

    def warning(self, message, **kwargs):
        self.warnings.append((message, kwargs))


def _identity_pose():
    pose = Pose()
    pose.orientation.w = 1.0
    return pose


def _link_properties_response(gravity_mode: bool):
    response = GetLinkProperties.Response()
    response.success = True
    response.com.orientation.w = 1.0
    response.gravity_mode = gravity_mode
    response.mass = 0.05
    response.ixx = 4.0e-5
    response.iyy = 5.0e-5
    response.izz = 6.0e-5
    return response


def _set_link_properties_response(success: bool):
    response = SetLinkProperties.Response()
    response.success = success
    return response


def test_gripper_attach_bridge_default_contract_constants_are_coherent():
    assert gripper_attach_bridge.OBJECT_NAME == "aruco_sample"
    assert gripper_attach_bridge.ATTACH_TOPIC.endswith("/aruco_sample")
    assert gripper_attach_bridge.DETACH_TOPIC.endswith("/aruco_sample")
    assert gripper_attach_bridge.ATTACH_STATUS_TOPIC == "/gripper/attach/status"
    assert gripper_attach_bridge.MODEL_STATES_TOPIC == "/gazebo/model_states"
    assert gripper_attach_bridge.TF_REFERENCE_FRAME == "odom"
    assert gripper_attach_bridge.GAZEBO_REFERENCE_FRAME == "world"
    assert gripper_attach_bridge.TCP_FRAME == "gripper_tcp"
    assert gripper_attach_bridge.OBJECT_LINK_NAME == "aruco_sample::link"


def test_quaternion_rotation_round_trip_preserves_tcp_offset():
    yaw_90 = (0.0, 0.0, math.sin(math.pi / 4.0), math.cos(math.pi / 4.0))
    offset_tcp = (0.030, -0.009, 0.020)

    offset_world = gripper_attach_bridge.rotate_vector(yaw_90, offset_tcp)
    recovered = gripper_attach_bridge.rotate_vector(
        gripper_attach_bridge.quat_conjugate(yaw_90),
        offset_world,
    )

    assert recovered == pytest.approx(offset_tcp)


def test_grasp_validator_accepts_boundary_values_and_rejects_overruns():
    config = GraspValidationConfig()

    assert validate_tcp_object_grasp((0.040, 0.018, 0.025), config).accepted
    assert not validate_tcp_object_grasp((0.041, 0.0, 0.0), config).accepted
    assert not validate_tcp_object_grasp((0.0, 0.019, 0.0), config).accepted
    assert not validate_tcp_object_grasp((0.0, 0.0, 0.026), config).accepted


def test_attach_converts_world_offset_to_tcp_frame_before_validation(monkeypatch):
    bridge = object.__new__(gripper_attach_bridge.GripperAttachBridge)
    tcp = Pose()
    tcp.position.x = 1.0
    tcp.position.y = 2.0
    tcp.position.z = 0.5
    tcp.orientation.z = math.sin(math.pi / 4.0)
    tcp.orientation.w = math.cos(math.pi / 4.0)

    expected_offset_tcp = (0.030, -0.009, 0.020)
    offset_world = gripper_attach_bridge.rotate_vector(
        gripper_attach_bridge.quat_tuple(tcp.orientation),
        expected_offset_tcp,
    )
    obj = Pose()
    obj.position.x = tcp.position.x + offset_world[0]
    obj.position.y = tcp.position.y + offset_world[1]
    obj.position.z = tcp.position.z + offset_world[2]
    obj.orientation = tcp.orientation

    seen = {}

    def fake_validate(offset_tcp, config):
        seen["offset_tcp"] = offset_tcp
        seen["config"] = config
        return GraspValidationResult(True, "accepted", offset_tcp, 0.0)

    monkeypatch.setattr(
        gripper_attach_bridge,
        "validate_tcp_object_grasp",
        fake_validate,
    )

    bridge._tcp_pose = lambda: tcp
    bridge._model_pose = obj
    bridge._grasp_config = GraspValidationConfig()
    bridge._attachment = None
    bridge._attachment_generation = 0
    bridge._object_name = "aruco_sample"
    bridge._tcp_frame = "gripper_tcp"
    bridge._request_object_gravity_disabled = lambda generation: None
    bridge._publish_attach_status = lambda status, reason="": None
    bridge.get_logger = lambda: type(
        "Logger",
        (),
        {"info": lambda *args, **kwargs: None, "warn": lambda *args, **kwargs: None},
    )()

    bridge._attach(None)

    assert seen["offset_tcp"] == pytest.approx(expected_offset_tcp)
    assert seen["config"] is bridge._grasp_config
    assert bridge._attachment.offset_tcp == pytest.approx((0.0, 0.0, 0.0))


def test_duplicate_attach_preserves_original_gravity_baseline():
    bridge = object.__new__(gripper_attach_bridge.GripperAttachBridge)
    tcp = _identity_pose()
    obj = _identity_pose()
    logger = _Logger()
    set_gravity_calls = []
    gravity_reads = [
        _link_properties_response(gravity_mode=True),
        _link_properties_response(gravity_mode=False),
    ]

    def request_object_gravity_disabled(generation):
        bridge._on_object_link_properties(_Future(gravity_reads.pop(0)), generation)

    def set_object_gravity(
        properties,
        gravity_mode,
        clear_properties_on_success=False,
    ):
        set_gravity_calls.append((properties.gravity_mode, gravity_mode))
        if clear_properties_on_success:
            bridge._object_link_properties = None

    bridge._tcp_pose = lambda: tcp
    bridge._model_pose = obj
    bridge._grasp_config = GraspValidationConfig()
    bridge._attachment = None
    bridge._attachment_generation = 0
    bridge._object_link_properties = None
    bridge._object_name = "aruco_sample"
    bridge._object_link_name = "aruco_sample::link"
    bridge._tcp_frame = "gripper_tcp"
    bridge._request_object_gravity_disabled = request_object_gravity_disabled
    bridge._set_object_gravity = set_object_gravity
    bridge._publish_attach_status = lambda status, reason="": None
    bridge.get_logger = lambda: logger

    bridge._attach(None)
    bridge._attach(None)
    bridge._detach(None)

    assert set_gravity_calls[-1] == (True, True)


def test_failed_gravity_restore_keeps_saved_link_properties():
    bridge = object.__new__(gripper_attach_bridge.GripperAttachBridge)
    logger = _Logger()
    properties = gripper_attach_bridge.capture_link_properties(
        _link_properties_response(gravity_mode=True)
    )

    def set_object_gravity(
        properties,
        gravity_mode,
        clear_properties_on_success=False,
    ):
        bridge._on_set_object_gravity(
            _Future(_set_link_properties_response(success=False)),
            gravity_mode,
            clear_properties_on_success,
        )

    bridge._object_link_properties = properties
    bridge._object_link_name = "aruco_sample::link"
    bridge._set_object_gravity = set_object_gravity
    bridge.get_logger = lambda: logger

    bridge._restore_object_gravity()

    assert bridge._object_link_properties is properties


def test_tick_resends_stale_pending_set_entity_state(monkeypatch):
    bridge = object.__new__(gripper_attach_bridge.GripperAttachBridge)
    logger = _Logger()
    tcp = _identity_pose()
    new_future = _Future(done=False)
    calls = []

    class FakeSetEntityStateClient:
        def call_async(self, request):
            calls.append(request)
            return new_future

    class FakeTime:
        @staticmethod
        def monotonic():
            return 12.25

    monkeypatch.setattr(gripper_attach_bridge, "time", FakeTime, raising=False)

    bridge._attachment = gripper_attach_bridge.Attachment(
        offset_tcp=(0.0, 0.0, 0.0),
        relative_orientation=Quaternion(w=1.0),
    )
    bridge._pending = _Future(done=False)
    bridge._pending_started_at = 11.0
    bridge._tcp_pose = lambda: tcp
    bridge._object_name = "aruco_sample"
    bridge._gazebo_reference_frame = "world"
    bridge._set_entity_state = FakeSetEntityStateClient()
    bridge.get_logger = lambda: logger

    bridge._tick()

    assert len(calls) == 1
    assert bridge._pending is new_future
    assert logger.warnings[-1][1] == {"once": True}


def test_gazebo_world_loads_state_service_plugin():
    world = ET.parse(SRC / "lab_cobot_gazebo" / "worlds" / "lab.world")
    plugins = {
        plugin.attrib["filename"]: plugin
        for plugin in world.findall("./world/plugin")
    }

    assert "libgazebo_ros_state.so" in plugins
    namespace = plugins["libgazebo_ros_state.so"].findtext("./ros/namespace")
    assert namespace == "/gazebo"


def test_gazebo_world_loads_properties_plugin_for_attach_physics():
    world = ET.parse(SRC / "lab_cobot_gazebo" / "worlds" / "lab.world")
    plugins = {
        plugin.attrib["filename"]: plugin
        for plugin in world.findall("./world/plugin")
    }

    assert "libgazebo_ros_properties.so" in plugins
    namespace = plugins["libgazebo_ros_properties.so"].findtext("./ros/namespace")
    assert namespace == "/gazebo"


def test_gazebo_launch_spawns_gripper_controller(monkeypatch):
    module = _load_launch_module(
        SRC / "lab_cobot_gazebo" / "launch" / "world.launch.py",
        "lab_cobot_gazebo_world_launch_test",
    )
    monkeypatch.setattr(module, "get_package_share_directory", _package_share)

    launch_description = module.generate_launch_description()
    controller_nodes = [
        entity
        for entity in _all_launch_entities(launch_description)
        if getattr(entity, "node_package", None) == "controller_manager"
    ]

    assert any(
        _node_arguments(node) == [
            "gripper_position_controller",
            "-c",
            "/controller_manager",
        ]
        for node in controller_nodes
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
