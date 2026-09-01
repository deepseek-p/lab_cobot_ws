"""Static first-phase checks for the independent yellow cube slot validation task."""
from __future__ import annotations

import importlib.util
import ast
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

from lab_cobot_manipulation import yellow_cube_slot_config as cfg


BRINGUP = Path(__file__).resolve().parents[1]
SRC = BRINGUP.parent
WS_SRC = SRC


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _load_launch(monkeypatch):
    monkeypatch.setenv("ROS_LOG_DIR", "/tmp/lab_cobot_pytest_ros_log")
    launch_file = BRINGUP / "launch" / "lab_cobot.launch.py"
    spec = importlib.util.spec_from_file_location("yellow_slot_launch_test", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, "get_package_share_directory", lambda pkg: str(SRC / pkg))
    return module, module.generate_launch_description()


def _declared_defaults(launch_description):
    defaults = {}
    context = LaunchContext()
    for entity in launch_description.entities:
        if entity.__class__.__name__ != "DeclareLaunchArgument":
            continue
        value = getattr(entity, "_DeclareLaunchArgument__default_value")
        if isinstance(value, tuple):
            value = list(value)
        if isinstance(value, list):
            defaults[entity.name] = perform_substitutions(context, value)
        else:
            defaults[entity.name] = str(value)
    return defaults


def _entities(launch_description, context):
    def walk(entity):
        yield entity
        if isinstance(entity, OpaqueFunction):
            for child in entity.execute(context):
                yield from walk(child)
        if isinstance(entity, TimerAction):
            for action in entity.actions:
                yield from walk(action)

    for entity in launch_description.entities:
        yield from walk(entity)


def _active_nodes(launch_description, overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    context.launch_configurations.update(overrides or {})
    return [
        entity
        for entity in _entities(launch_description, context)
        if isinstance(entity, Node)
        and (entity.condition is None or entity.condition.evaluate(context))
    ]


def _active_includes(launch_description, overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    context.launch_configurations.update(overrides or {})
    return [
        entity
        for entity in _entities(launch_description, context)
        if isinstance(entity, IncludeLaunchDescription)
        and (entity.condition is None or entity.condition.evaluate(context))
    ]


def _include_args(include):
    return {name: value for name, value in include.launch_arguments}


def _pose_from_text(text: str) -> tuple[float, float, float, float, float, float]:
    values = [float(value) for value in text.split()]
    values += [0.0] * (6 - len(values))
    return tuple(values[:6])


def _world_root() -> ET.Element:
    return ET.parse(WS_SRC / "lab_cobot_gazebo/worlds/lab.world").getroot()


def _model_root(model: str) -> ET.Element:
    return ET.parse(WS_SRC / f"lab_cobot_gazebo/models/{model}/model.sdf").getroot()


def test_protected_runtime_files_do_not_contain_yellow_validation_logic():
    protected = [
        WS_SRC / "lab_cobot_bringup/lab_cobot_bringup/mission_node.py",
        WS_SRC / "lab_cobot_navigation/launch/navigation.launch.py",
        WS_SRC / "lab_cobot_navigation/config/nav2_params.yaml",
        WS_SRC / "lab_cobot_navigation/lab_cobot_navigation/waypoints.py",
        WS_SRC / "lab_cobot_bringup/lab_cobot_bringup/tube_insert_validation_node.py",
        WS_SRC / "lab_cobot_perception/lab_cobot_perception/object_detector.py",
    ]
    forbidden = (
        "yellow_cube_slot_validation",
        "/yellow_cube_slot_validation",
        "YELLOW_CUBE_SLOT_SUCCESS",
    )
    for path in protected:
        text = _text(path)
        for marker in forbidden:
            assert marker not in text, path


def test_launch_default_disabled_and_uses_independent_node(monkeypatch):
    _module, launch_description = _load_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    assert defaults["launch_yellow_cube_slot_validation"] == "false"

    default_executables = {
        node.node_executable for node in _active_nodes(launch_description)
    }
    assert "yellow_cube_slot_validation_node" not in default_executables

    yellow_executables = {
        node.node_executable
        for node in _active_nodes(
            launch_description,
            {"launch_yellow_cube_slot_validation": "true"},
        )
    }
    assert "yellow_cube_slot_validation_node" in yellow_executables
    assert "mission_node" not in yellow_executables
    assert "aruco_detector" not in yellow_executables
    assert "object_detector" not in yellow_executables


def test_yellow_spawn_pose_only_in_yellow_mode(monkeypatch):
    module, launch_description = _load_launch(monkeypatch)
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    assert module._validation_spawn_value("x", "4.50", context) == "4.50"
    assert module._validation_spawn_value("y", "-4.20", context) == "-4.20"

    context.launch_configurations["launch_yellow_cube_slot_validation"] = "true"
    assert module._validation_spawn_value("x", "4.50", context) == "-4.300000"
    assert module._validation_spawn_value("y", "-4.20", context) == "2.745000"
    assert module._validation_spawn_value("yaw", "0.0", context) == "1.570796"

    world = next(
        include
        for include in _active_includes(
            launch_description,
            {"launch_yellow_cube_slot_validation": "true"},
        )
        if "robot_spawn_x" in _include_args(include)
    )
    args = _include_args(world)
    assert args["robot_spawn_x"] == "-4.300000"
    assert args["robot_spawn_y"] == "2.745000"
    assert args["robot_spawn_yaw"] == "1.570796"

    navigation = next(
        include
        for include in _active_includes(
            launch_description,
            {"launch_yellow_cube_slot_validation": "true"},
        )
        if "params_file" in _include_args(include)
    )
    nav_args = _include_args(navigation)
    assert perform_substitutions(context, [nav_args["amcl_initial_pose_x"]]) == "-4.300000"
    assert perform_substitutions(context, [nav_args["amcl_initial_pose_y"]]) == "2.745000"
    assert perform_substitutions(context, [nav_args["amcl_initial_pose_yaw"]]) == "1.570796"


def test_status_chain_is_independent_and_skips_initial_and_home_navigation():
    assert cfg.TARGET_TOPIC == "/yellow_cube_slot_validation/target"
    assert cfg.STATUS_TOPIC == "/yellow_cube_slot_validation/status"
    assert "NAV_AGING_ZONE" in cfg.STATUS_SEQUENCE
    assert "NAV_STATION_A" not in cfg.STATUS_SEQUENCE
    assert "NAV_HOME" not in cfg.STATUS_SEQUENCE
    assert "RETURN_HOME" not in cfg.STATUS_SEQUENCE
    assert "SLOT_ALIGNMENT_ROTATE_START" in cfg.STATUS_SEQUENCE
    assert "SLOT_ALIGNMENT_ROTATE_DONE" in cfg.STATUS_SEQUENCE
    assert "INSERT_STAGE1" in cfg.STATUS_SEQUENCE
    assert "INSERT_STAGE2" in cfg.STATUS_SEQUENCE
    assert "INSERT_FINAL" in cfg.STATUS_SEQUENCE
    assert "INSERT_SHALLOW" not in cfg.STATUS_SEQUENCE
    assert "ARM_TRANSPORT_SAFE" in cfg.STATUS_SEQUENCE
    assert "STATION_A_RETREAT_START" in cfg.STATUS_SEQUENCE
    assert "STATION_A_RETREAT_SAFE" in cfg.STATUS_SEQUENCE

    node_source = _text(BRINGUP / "lab_cobot_bringup/yellow_cube_slot_validation_node.py")
    assert "navigation_goals_for_station(\"aging_zone\")" in node_source
    assert "navigation_goals_for_station(\"station_a\")" not in node_source
    assert "navigation_goals_for_station(\"home\")" not in node_source
    assert "STATION_SPECS[\"aging_zone\"].nav_pose" not in node_source
    assert "YELLOW_DIRECT_AGING_NAV_START" not in node_source
    assert "NAV_GOAL_COUNT=1" not in node_source
    assert "USING_PUBLIC_AGING_NAV_LEGS=NO" not in node_source
    assert "PUBLIC_SOUTH_ENTRY_VISITED=NO" not in node_source
    assert "DIRECT_NAV_GOAL" not in node_source
    assert "YELLOW_AGING_NAV_MODE=PUBLIC_AGING_NAV_LEGS" in node_source
    assert "NAV_GOAL_COUNT=%d" in node_source
    assert "NAV_LEG_%d_START" in node_source
    assert "NAV_LEG_%d_DONE" in node_source
    assert "stop_after_nav_aging_zone" in node_source
    assert "YELLOW_NAV_AGING_ZONE_SUCCESS" in node_source
    assert node_source.index("YELLOW_NAV_AGING_ZONE_SUCCESS") < node_source.index(
        "if not self._aging_station_dock():"
    )
    assert "_world_point_to_base(pre_grasp_high_tcp_world())" in node_source
    assert "_world_point_to_base(pre_slot_high_tcp_world(), base_pose)" in node_source
    assert "STATION_A_FINE_DOCK_START" in node_source
    assert "STATION_A_FINE_DOCK_DONE" in node_source
    assert node_source.index("STATION_A_FINE_DOCK_DONE") < node_source.index(
        "_world_point_to_base(pre_grasp_high_tcp_world())"
    )
    assert "SLOT_ALIGNMENT_QUAT_XYZW" in node_source
    tree = ast.parse(node_source)
    mission_imports = {
        alias.name
        for node in tree.body
        if isinstance(node, ast.ImportFrom)
        and node.module == "lab_cobot_bringup.mission_node"
        for alias in node.names
    }
    assert "NAV_ACTIVE_TIMEOUT_SEC" not in mission_imports
    assert "NAV_TF_READY_TIMEOUT_SEC" not in mission_imports
    assert "STATION_DOCK_MAX_SEC" not in mission_imports
    assert "STATION_DOCK_SETTLE_SEC" not in mission_imports
    assert "YELLOW_NAV_ACTIVE_TIMEOUT_SEC" in node_source
    assert "YELLOW_NAV_TF_READY_TIMEOUT_SEC" in node_source
    assert "def _fine_dock_station_a_for_cube" in node_source
    assert "_fine_dock_station_a_for_cube()" in node_source
    assert "BASE_FOOTPRINT_ENTITY = \"lab_cobot::base_footprint\"" in node_source
    assert "def _gazebo_base_pose" in node_source
    assert "stop_after_station_a_fine_dock" in node_source
    assert "stop_after_yellow_lift" in node_source
    assert "YELLOW_GRASP_ONLY_SUCCESS" in node_source
    assert node_source.index("YELLOW_GRASP_ONLY_SUCCESS") < node_source.index(
        "FAILED_NAV_AGING_ZONE"
    )
    assert "stop_after_nav_leg1_start" in node_source
    assert "YELLOW_NAV_LEG_1_STARTED" in node_source
    assert "stop_after_aging_fine_dock" in node_source
    assert "YELLOW_AGING_FINE_DOCK_ONLY_SUCCESS" in node_source
    assert "stop_after_pre_slot_high" in node_source
    assert "YELLOW_PRE_SLOT_HIGH_SUCCESS" in node_source
    assert node_source.index("YELLOW_PRE_SLOT_HIGH_SUCCESS") < node_source.index(
        "SLOT_ALIGNMENT_ROTATE_START"
    )
    assert "YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_X" in node_source
    assert "station_dock_velocity_for_base" in node_source
    assert "_fine_dock(\"station_a\")" not in node_source
    assert "for goal_index, (goal_name, waypoint, _handoff_station) in enumerate(goals, start=1)" in node_source
    assert "leg[\"x\"]" not in node_source
    assert "leg.get(\"dock_station\")" not in node_source
    assert "msg.pose.orientation = yaw_to_quat" not in node_source
    assert "msg.pose.orientation.w = qw" in node_source
    assert "TaskResult.SUCCEEDED" in node_source
    assert "NAV_TIMEOUT_SEC" not in node_source
    assert "deadline = time.monotonic() + NAV_TIMEOUT_SEC" not in node_source
    assert "GoalStatus.STATUS_SUCCEEDED" not in node_source
    assert "_move_cartesian(\"INSERT_STAGE1\"" in node_source
    assert "_move_cartesian(\"INSERT_STAGE2\"" in node_source
    assert "_move_cartesian(\"INSERT_FINAL\"" in node_source
    assert "_move_cartesian(\"INSERT_SHALLOW\"" not in node_source


def test_yellow_post_grasp_exit_reuses_aging_station_a_retreat_before_nav():
    node_source = _text(BRINGUP / "lab_cobot_bringup/yellow_cube_slot_validation_node.py")

    lift = node_source.index('self._move_cartesian("LIFT", lift, DOWN_QUAT)')
    grasp_stop = node_source.index('get_parameter("stop_after_yellow_lift")', lift)
    transport = node_source.index("if not self._transport_safe():", grasp_stop)
    retreat = node_source.index("if not self._retreat_from_station_a_table():", transport)
    nav = node_source.index("if not self._navigate_aging_zone():", retreat)

    assert lift < grasp_stop < transport < retreat < nav
    assert "def _transport_safe(self) -> bool:" in node_source
    assert "self._publish_status(\"ARM_TRANSPORT_SAFE\")" in node_source
    assert "ok = bool(self.go_home())" in node_source
    assert "def _retreat_from_station_a_table(self) -> bool:" in node_source
    assert "STATION_A_RETREAT_TARGET_CLEARANCE = 0.600" in node_source
    assert "STATION_A_RETREAT_MAX_LINEAR_SPEED = 0.080" in node_source
    assert "world_vy = -abs(speed)" in node_source
    assert "cmd.linear.x = math.cos(yaw) * world_vx + math.sin(yaw) * world_vy" in node_source
    assert "cmd.linear.y = -math.sin(yaw) * world_vx + math.cos(yaw) * world_vy" in node_source
    assert "def _station_a_retreat_plan_check(self) -> bool:" in node_source
    assert "path = self._nav.getPath(start, goal, use_start=False)" in node_source
    assert "POST_RETREAT_GLOBAL_PLAN_AVAILABLE=true" in node_source


def test_yellow_aging_route_uses_public_nav_legs_and_public_route_is_unchanged():
    from lab_cobot_bringup.mission_node import navigation_goals_for_station

    node_source = _text(BRINGUP / "lab_cobot_bringup/yellow_cube_slot_validation_node.py")
    public_goals = navigation_goals_for_station("aging_zone")
    assert [goal[0] for goal in public_goals] == [
        "aging_zone_south_entry",
        "aging_zone_east_corridor",
        "aging_zone",
    ]
    assert public_goals[0][1] == {"x": 2.00, "y": -5.05, "yaw": math.pi / 2.0}
    assert public_goals[1][1] == {"x": 2.00, "y": 2.97, "yaw": math.pi / 2.0}
    assert public_goals[2][1] == {"x": 0.20, "y": 2.97, "yaw": math.pi / 2.0}

    assert "goals = navigation_goals_for_station(\"aging_zone\")" in node_source
    assert "STATION_SPECS[\"aging_zone\"].nav_pose" not in node_source
    assert "goal = _pose_stamped(waypoint[\"x\"], waypoint[\"y\"], waypoint[\"yaw\"])" in node_source
    assert node_source.count("self._nav.goToPose(goal)") == 1


def test_yellow_post_nav_docks_and_fine_docks_before_pre_slot_high():
    node_source = _text(BRINGUP / "lab_cobot_bringup/yellow_cube_slot_validation_node.py")

    nav = node_source.index("if not self._navigate_aging_zone():")
    nav_stop = node_source.index('get_parameter("stop_after_nav_aging_zone")', nav)
    station_dock = node_source.index("if not self._aging_station_dock():", nav_stop)
    fine_dock = node_source.index("if not self._aging_place_fine_dock():", station_dock)
    fine_stop = node_source.index('get_parameter("stop_after_aging_fine_dock")', fine_dock)
    recompute = node_source.index("targets = self._placement_targets_after_aging_fine_dock()", fine_stop)
    pre_slot = node_source.index('self._move_ompl("PRE_SLOT_HIGH"', recompute)

    assert nav < nav_stop < station_dock < fine_dock < fine_stop < recompute < pre_slot
    assert "YELLOW_AGING_STATION_DOCK_START" in node_source
    assert "YELLOW_AGING_STATION_DOCK_DONE" in node_source
    assert "YELLOW_AGING_FINE_DOCK_START" in node_source
    assert "YELLOW_AGING_FINE_DOCK_SUCCESS" in node_source
    assert "YELLOW_AGING_FINE_DOCK_FINAL_SLOT_BASE" in node_source
    assert "PRE_SLOT_HIGH_TARGET_AFTER_FINE_DOCK" in node_source
    assert "YELLOW_AGING_FINE_DOCK_ONLY_SUCCESS" in node_source
    assert node_source.index("YELLOW_AGING_FINE_DOCK_ONLY_SUCCESS") < pre_slot


class _FakeLogger:
    def __init__(self):
        self.infos = []
        self.warnings = []

    def info(self, msg):
        self.infos.append(str(msg))

    def warn(self, msg):
        self.warnings.append(str(msg))


class _FakeNav:
    def __init__(self, complete_sequence, result):
        self._complete_sequence = list(complete_sequence)
        self._result = result
        self.cancel_count = 0

    def isTaskComplete(self):
        if self._complete_sequence:
            return self._complete_sequence.pop(0)
        return False

    def getResult(self):
        return self._result

    def cancelTask(self):
        self.cancel_count += 1


class _FakeOrientation:
    def __init__(self, yaw: float):
        self.x = 0.0
        self.y = 0.0
        self.z = math.sin(float(yaw) / 2.0)
        self.w = math.cos(float(yaw) / 2.0)


class _FakePosition:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z


class _FakePose:
    def __init__(self, x: float, y: float, z: float, yaw: float):
        self.position = _FakePosition(x, y, z)
        self.orientation = _FakeOrientation(yaw)


class _FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


def _load_yellow_node_module():
    path = BRINGUP / "lab_cobot_bringup/yellow_cube_slot_validation_node.py"
    spec = importlib.util.spec_from_file_location("yellow_node_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _node_for_nav_wait(module, nav):
    node = object.__new__(module.YellowCubeSlotValidation)
    logger = _FakeLogger()
    node._nav = nav
    node._holding_is_healthy = lambda: True
    node._handle_hold_lost = lambda: None
    node.get_logger = lambda: logger
    return node, logger


def test_yellow_aging_fine_dock_command_slot_x_direction():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)

    done_high, cmd_high, _error_x, _error_y = node._aging_fine_dock_command(
        {"x": 0.800, "y": 0.0, "z": 0.653}
    )
    done_low, cmd_low, _error_x, _error_y = node._aging_fine_dock_command(
        {"x": 0.700, "y": 0.0, "z": 0.653}
    )

    assert done_high is False
    assert done_low is False
    assert cmd_high.linear.x > 0.0
    assert cmd_low.linear.x < 0.0


def test_yellow_aging_fine_dock_command_slot_y_correction_and_success_window():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)

    done_y, cmd_y, _error_x, _error_y = node._aging_fine_dock_command(
        {"x": 0.735, "y": 0.060, "z": 0.653}
    )
    done_ok, cmd_ok, _error_x, _error_y = node._aging_fine_dock_command(
        {"x": 0.735, "y": 0.020, "z": 0.653}
    )

    assert done_y is False
    assert cmd_y.linear.y > 0.0
    assert done_ok is True
    assert cmd_ok.linear.x == 0.0
    assert cmd_ok.linear.y == 0.0
    assert module.AGING_PLACE_SLOT_BASE_X_MIN == pytest.approx(0.720)
    assert module.AGING_PLACE_SLOT_BASE_X_MAX == pytest.approx(0.750)
    assert module.AGING_PLACE_SLOT_BASE_Y_TOLERANCE == pytest.approx(0.040)


def test_yellow_placement_targets_are_recomputed_after_latest_aging_base_pose():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)
    logger = _FakeLogger()
    node.get_logger = lambda: logger
    node._publish_status = lambda _status: None
    node._get_actual_base_pose_for_aging_place = lambda: {
        "x": 0.200,
        "y": 3.145,
        "z": cfg.BASE_LINK_WORLD_Z,
        "yaw": math.pi / 2.0,
        "source": module.BASE_FOOTPRINT_ENTITY,
    }

    targets = node._placement_targets_after_aging_fine_dock()

    assert targets is not None
    assert targets["pre_slot_high"] == pytest.approx([0.735, 0.0, 0.851])
    assert targets["pre_slot"] == pytest.approx([0.735, 0.0, 0.791])
    assert targets["stage1"] == pytest.approx([0.735, 0.0, 0.731])
    assert targets["stage2"] == pytest.approx([0.735, 0.0, 0.696])
    assert targets["final"] == pytest.approx([0.735, 0.0, 0.671])
    assert any("PRE_SLOT_HIGH_TARGET_AFTER_FINE_DOCK" in msg for msg in logger.infos)


def test_yellow_actual_base_pose_for_aging_place_uses_gazebo_base_pose():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)
    node._gazebo_base_pose = lambda: {"x": 0.34, "y": 2.90, "yaw": 1.815}

    pose = node._get_actual_base_pose_for_aging_place()

    assert pose["x"] == pytest.approx(0.34)
    assert pose["y"] == pytest.approx(2.90)
    assert pose["z"] == pytest.approx(cfg.BASE_LINK_WORLD_Z)
    assert pose["yaw"] == pytest.approx(1.815)
    assert pose["source"] == module.BASE_FOOTPRINT_ENTITY


def test_yellow_aging_fine_dock_stops_when_holding_is_lost():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)
    publisher = _FakePublisher()
    statuses = []
    handled = []
    node._cmd_pub = publisher
    node._holding_is_healthy = lambda: False
    node._handle_hold_lost = lambda: handled.append(True)
    node._publish_status = lambda status: statuses.append(status)

    assert node._aging_place_fine_dock() is False
    assert handled == [True]
    assert "FAILED_HOLDING" in statuses
    assert publisher.messages == []


@pytest.mark.parametrize(
    ("actual_yaw", "expected_error", "valid"),
    [
        (-1.5214, 0.0493963268, True),
        (-math.pi / 2.0, 0.0, True),
        (0.0, 0.0, True),
        (math.pi / 2.0, 0.0, True),
        (math.pi, 0.0, True),
        (-math.pi / 4.0, math.pi / 4.0, False),
    ],
)
def test_yellow_place_yaw_validation_uses_square_symmetry(
    actual_yaw,
    expected_error,
    valid,
):
    module = _load_yellow_node_module()

    error = module._symmetric_yaw_error(actual_yaw, module.SLOT_ALIGNMENT_YAW)

    assert error == pytest.approx(expected_error)
    assert (error <= module.PLACE_VALIDATION_YAW_TOLERANCE) is valid


def test_yellow_place_validation_keeps_xyz_and_slot_containment_requirements():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)
    logger = _FakeLogger()
    statuses = []
    target = cfg.middle_slot_center_world()
    node.get_logger = lambda: logger
    node._publish_status = lambda status: statuses.append(status)
    node._gazebo_cube_pose = lambda: _FakePose(
        target[0],
        target[1],
        target[2] + module.PLACE_VALIDATION_Z_TOLERANCE + 0.001,
        module.SLOT_ALIGNMENT_YAW,
    )

    assert node._validate_place() is False
    assert statuses == []
    assert any("actual_yaw=-1.5708" in msg for msg in logger.infos)
    assert any("expected_yaw=-1.5708" in msg for msg in logger.infos)
    assert any("yaw_tolerance=0.1500" in msg for msg in logger.infos)


def test_yellow_place_validation_accepts_real_rotated_sample():
    module = _load_yellow_node_module()
    node = object.__new__(module.YellowCubeSlotValidation)
    logger = _FakeLogger()
    statuses = []
    target = cfg.middle_slot_center_world()
    node.get_logger = lambda: logger
    node._publish_status = lambda status: statuses.append(status)
    node._gazebo_cube_pose = lambda: _FakePose(
        target[0] + 0.0002,
        target[1],
        target[2] - 0.0031,
        -1.5214,
    )

    assert node._validate_place() is True
    assert statuses == ["YELLOW_CUBE_SLOT_PLACE_VALID"]
    assert any("yaw_error=0.0494" in msg for msg in logger.infos)
    assert any("inside=True" in msg for msg in logger.infos)


def test_yellow_nav_running_past_240_wall_seconds_does_not_cancel(monkeypatch):
    module = _load_yellow_node_module()
    nav = _FakeNav([False, False, False], module.TaskResult.SUCCEEDED)
    node, logger = _node_for_nav_wait(module, nav)
    times = iter([0.0, 241.0, 300.0])
    ok_values = iter([True, True, False])
    monkeypatch.setattr(module.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: next(ok_values))

    assert node._wait_for_nav_leg_result(1, "aging_zone_south_entry") is False
    assert nav.cancel_count == 0
    assert not any("FAILED_NAV_AGING_ZONE" in msg for msg in logger.infos)
    assert any("elapsed_wall=241.0" in msg for msg in logger.infos)


def test_yellow_nav_running_past_240_then_succeeded_passes_without_cancel(monkeypatch):
    module = _load_yellow_node_module()
    nav = _FakeNav([False, False, True], module.TaskResult.SUCCEEDED)
    node, _logger = _node_for_nav_wait(module, nav)
    times = iter([0.0, 241.0, 300.0])
    monkeypatch.setattr(module.time, "monotonic", lambda: next(times))
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)

    assert node._wait_for_nav_leg_result(2, "aging_zone_east_corridor") is True
    assert nav.cancel_count == 0


@pytest.mark.parametrize("result", ["FAILED", "CANCELED"])
def test_yellow_nav_terminal_non_success_results_fail_without_timeout_cancel(
    monkeypatch,
    result,
):
    module = _load_yellow_node_module()
    task_result = getattr(module.TaskResult, result)
    nav = _FakeNav([True], task_result)
    node, logger = _node_for_nav_wait(module, nav)
    monkeypatch.setattr(module.time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: True)

    assert node._wait_for_nav_leg_result(3, "aging_zone") is False
    assert nav.cancel_count == 0
    assert any(f"result={task_result}" in msg for msg in logger.warnings)


def test_yellow_nav_wait_exits_on_ros_shutdown(monkeypatch):
    module = _load_yellow_node_module()
    nav = _FakeNav([False], module.TaskResult.SUCCEEDED)
    node, logger = _node_for_nav_wait(module, nav)
    monkeypatch.setattr(module.time, "monotonic", lambda: 0.0)
    monkeypatch.setattr(module.time, "sleep", lambda _seconds: None)
    monkeypatch.setattr(module.rclpy, "ok", lambda: False)

    assert node._wait_for_nav_leg_result(1, "aging_zone_south_entry") is False
    assert nav.cancel_count == 0
    assert any("reason=ROS_SHUTDOWN" in msg for msg in logger.warnings)


def test_material_cube_geometry_matches_world_and_sdf():
    world = _world_root()
    include = next(
        elem for elem in world.findall(".//include")
        if elem.findtext("name") == cfg.TARGET_OBJECT
    )
    model_pose = _pose_from_text(include.findtext("pose"))
    assert model_pose == (
        cfg.MATERIAL_CUBE_YELLOW_WORLD_POSE.x,
        cfg.MATERIAL_CUBE_YELLOW_WORLD_POSE.y,
        cfg.MATERIAL_CUBE_YELLOW_WORLD_POSE.z,
        0.0,
        0.0,
        0.0,
    )

    root = _model_root("material_cube_yellow")
    collision = root.find(".//collision[@name='collision']")
    size = tuple(float(value) for value in collision.findtext(".//box/size").split())
    assert size == cfg.MATERIAL_CUBE_YELLOW_COLLISION_SIZE
    assert cfg.cube_center_world() == (-4.30, 3.52, 0.785)
    assert math.isclose(cfg.cube_top_world_z(), 0.820, abs_tol=1e-9)


def test_middle_slot_geometry_matches_rack_sdf():
    world = _world_root()
    include = next(
        elem for elem in world.findall(".//include")
        if elem.findtext("name") == "aging_rack"
    )
    rack_pose = _pose_from_text(include.findtext("pose"))
    assert rack_pose == (0.20, 3.88, 0.80, 0.0, 0.0, 0.0)

    root = _model_root("aging_rack")
    collisions = {
        elem.attrib["name"]: (
            _pose_from_text(elem.findtext("pose")),
            tuple(float(value) for value in elem.findtext(".//box/size").split()),
        )
        for elem in root.findall(".//collision")
    }
    left_divider_pose, left_divider_size = collisions["collision_divider_left"]
    right_divider_pose, right_divider_size = collisions["collision_divider_right"]
    front_pose, front_size = collisions["collision_wall_front"]
    back_pose, back_size = collisions["collision_wall_back"]
    bottom_pose, bottom_size = collisions["collision_bottom"]

    left_boundary = left_divider_pose[0] + left_divider_size[0] / 2.0
    right_boundary = right_divider_pose[0] - right_divider_size[0] / 2.0
    back_boundary = back_pose[1] + back_size[1] / 2.0
    front_boundary = front_pose[1] - front_size[1] / 2.0
    bottom_top = bottom_pose[2] + bottom_size[2] / 2.0

    assert cfg.MIDDLE_SLOT_INDEX == 1
    assert math.isclose(left_boundary, cfg.MIDDLE_SLOT_LEFT_BOUNDARY_LOCAL_X)
    assert math.isclose(right_boundary, cfg.MIDDLE_SLOT_RIGHT_BOUNDARY_LOCAL_X)
    assert math.isclose(back_boundary, cfg.MIDDLE_SLOT_BACK_BOUNDARY_LOCAL_Y)
    assert math.isclose(front_boundary, cfg.MIDDLE_SLOT_FRONT_BOUNDARY_LOCAL_Y)
    assert math.isclose(bottom_top, cfg.MIDDLE_SLOT_BOTTOM_LOCAL_Z)
    assert math.isclose(cfg.MIDDLE_SLOT_WIDTH, 0.090)
    assert math.isclose(cfg.MIDDLE_SLOT_DEPTH, 0.180)
    assert math.isclose(cfg.MIDDLE_SLOT_HEIGHT, 0.080)
    assert cfg.cube_slot_clearance() == pytest.approx({
        "width": 0.01999999999999999,
        "depth": 0.10999999999999999,
        "height": 0.010000000000000009,
    })


def test_insert_targets_are_derived_from_slot_and_cube_geometry():
    center = cfg.middle_slot_center_world()
    assert center == pytest.approx((0.20, 3.88, 0.808))
    assert cfg.insert_stage1_tcp_world() == pytest.approx((0.20, 3.88, 0.886))
    assert cfg.insert_stage2_tcp_world() == pytest.approx((0.20, 3.88, 0.851))
    assert cfg.insert_final_tcp_world() == pytest.approx((0.20, 3.88, 0.826))
    assert cfg.vertical_retreat_tcp_world() == pytest.approx((0.20, 3.88, 0.946))


def test_station_a_world_targets_are_converted_to_base_frame():
    base = cfg.YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE
    assert cfg.world_to_base(cfg.cube_center_world(), base) == pytest.approx([
        0.775,
        0.0,
        0.630,
    ])
    assert cfg.world_to_base(cfg.grasp_tcp_world(), base) == pytest.approx([
        0.775,
        0.0,
        0.648,
    ])
    assert cfg.world_to_base(cfg.pre_grasp_tcp_world(), base) == pytest.approx([
        0.775,
        0.0,
        0.743,
    ])
    assert cfg.world_to_base(cfg.pre_grasp_high_tcp_world(), base) == pytest.approx([
        0.775,
        0.0,
        0.823,
    ])
    assert cfg.grasp_tcp_world() != tuple(
        cfg.world_to_base(cfg.grasp_tcp_world(), base)
    )


def test_aging_slot_world_targets_are_converted_to_base_frame():
    aging_base = {"x": 0.20, "y": 3.345, "yaw": math.pi / 2.0}
    assert cfg.world_to_base(cfg.middle_slot_center_world(), aging_base) == pytest.approx([
        0.535,
        0.0,
        0.653,
    ])
    assert cfg.world_to_base(cfg.pre_slot_tcp_world(), aging_base) == pytest.approx([
        0.535,
        0.0,
        0.791,
    ])
    assert cfg.world_to_base(cfg.insert_shallow_tcp_world(), aging_base) == pytest.approx([
        0.535,
        0.0,
        0.750,
    ])


def test_clearance_tolerance_and_gripper_width_select_shallow_strategy():
    total = cfg.cube_slot_clearance()
    per_side = cfg.cube_slot_per_side_clearance()
    assert total["width"] == pytest.approx(0.020)
    assert per_side["width"] == pytest.approx(0.010)
    assert total["depth"] == pytest.approx(0.110)
    assert per_side["depth"] == pytest.approx(0.055)
    assert cfg.PLACE_VALIDATION_XY_TOLERANCE <= per_side["width"] - 0.001
    assert cfg.PLACE_VALIDATION_XY_TOLERANCE == pytest.approx(
        cfg.recommended_place_xy_tolerance()
    )

    assert cfg.gripper_finger_position_for_cube() == pytest.approx(0.011)
    assert cfg.gripper_outer_width_while_holding_cube() == pytest.approx(0.094)
    assert cfg.gripper_outer_width_while_holding_cube() > cfg.MIDDLE_SLOT_WIDTH
    assert cfg.gripper_cube_unrotated_slot_width_extent() == pytest.approx(0.094)
    assert cfg.gripper_cube_unrotated_slot_depth_extent() == pytest.approx(0.070)
    assert cfg.gripper_cube_rotated_slot_width_extent() == pytest.approx(0.070)
    assert cfg.gripper_cube_rotated_slot_depth_extent() == pytest.approx(0.094)
    assert cfg.rotated_deep_insert_clearance() == pytest.approx({
        "width_total": 0.020,
        "width_per_side": 0.010,
        "depth_total": 0.086,
        "depth_per_side": 0.043,
    })
    assert cfg.SLOT_ALIGNMENT_YAW == pytest.approx(-math.pi / 2.0)
    assert cfg.SLOT_ALIGNMENT_QUAT_XYZW == pytest.approx([
        math.sqrt(0.5),
        -math.sqrt(0.5),
        0.0,
        0.0,
    ])
    assert cfg.deep_insert_physically_possible()
    assert cfg.PLACEMENT_STRATEGY == "ROTATED_DEEP_INSERT"


def test_yellow_startup_constants_are_local_and_baseline_targets_unchanged():
    assert cfg.YELLOW_NAV_ACTIVE_TIMEOUT_SEC == pytest.approx(20.0)
    assert cfg.YELLOW_NAV_TF_READY_TIMEOUT_SEC == pytest.approx(12.0)
    assert cfg.YELLOW_STATION_DOCK_MAX_SEC == pytest.approx(80.0)
    assert cfg.YELLOW_STATION_DOCK_SETTLE_SEC == pytest.approx(0.3)
    assert cfg.YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_X == pytest.approx(0.720)
    assert cfg.YELLOW_STATION_A_FINE_DOCK_TARGET_CUBE_BASE_Y == pytest.approx(0.000)
    assert cfg.YELLOW_STATION_A_FINE_DOCK_X_TOLERANCE == pytest.approx(0.020)
    assert cfg.YELLOW_STATION_A_FINE_DOCK_Y_TOLERANCE == pytest.approx(0.040)
    assert cfg.YELLOW_CUBE_SLOT_VALIDATION_BASE_POSE == pytest.approx({
        "x": -4.300,
        "y": 2.745,
        "yaw": math.pi / 2.0,
    })
    assert cfg.TARGET_OBJECT == "material_cube_yellow"
    assert cfg.MIDDLE_SLOT_INDEX == 1
    assert cfg.grasp_tcp_world() == pytest.approx((-4.300, 3.520, 0.803))
    assert cfg.pre_grasp_tcp_world() == pytest.approx((-4.300, 3.520, 0.898))
    assert cfg.pre_grasp_high_tcp_world() == pytest.approx((-4.300, 3.520, 0.978))
    assert cfg.insert_stage1_tcp_world() == pytest.approx((0.200, 3.880, 0.886))
    assert cfg.insert_stage2_tcp_world() == pytest.approx((0.200, 3.880, 0.851))
    assert cfg.insert_final_tcp_world() == pytest.approx((0.200, 3.880, 0.826))


def test_deep_insert_vertical_envelope_clears_slot_bottom():
    assert cfg.slot_inner_bottom_world_z() == pytest.approx(0.770)
    assert cfg.desired_cube_bottom_world_z() == pytest.approx(0.773)
    assert cfg.gripper_finger_world_z_span_at_deep_final() == pytest.approx((
        0.814,
        0.889,
    ))
    assert cfg.gripper_tactile_probe_world_z_span_at_deep_final() == pytest.approx((
        0.7915,
        0.9115,
    ))
    assert cfg.gripper_body_visual_world_z_span_at_deep_final()[0] > cfg.slot_top_world_z()
    for stage in ("INSERT_STAGE1", "INSERT_STAGE2", "INSERT_FINAL"):
        result = cfg.stage_collision_check(stage)
        assert result["footprint_ok"]
        assert result["cube_bottom_above_inner_bottom"]
        assert result["probe_bottom_above_inner_bottom"]
