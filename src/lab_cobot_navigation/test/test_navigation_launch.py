"""Regression checks for Nav2 launch wiring."""
import importlib.util
from pathlib import Path

from ament_index_python.packages import get_package_share_directory as real_share
from launch import LaunchContext
from launch.actions import IncludeLaunchDescription
from launch.substitutions import SubstitutionFailure
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


NAVIGATION = Path(__file__).resolve().parents[1]
SRC = NAVIGATION.parent


def _load_navigation_launch(monkeypatch):
    launch_file = NAVIGATION / "launch" / "navigation.launch.py"
    spec = importlib.util.spec_from_file_location("navigation_launch_test", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda pkg: str(SRC / pkg) if pkg == "lab_cobot_navigation" else real_share(pkg),
    )
    return module.generate_launch_description()


def _all_actions(launch_description):
    actions = []

    def _walk(entity):
        actions.append(entity)
        if isinstance(entity, IncludeLaunchDescription):
            child = entity.launch_description_source.try_get_launch_description_without_context()
            if child is not None:
                for child_entity in child.entities:
                    _walk(child_entity)
        for child in getattr(entity, "_GroupAction__actions", []) or []:
            _walk(child)
        # TimerAction 子动作(lifecycle manager 延后启动后藏在这里)
        for child in getattr(entity, "_TimerAction__actions", []) or []:
            _walk(child)

    for entity in launch_description.entities:
        _walk(entity)
    return actions


def _text(value):
    context = LaunchContext()
    if isinstance(value, tuple):
        value = list(value)
    if isinstance(value, list) and value and all(
        isinstance(item, (list, tuple)) for item in value
    ):
        return [_text(item) for item in value]
    if isinstance(value, list) and value and hasattr(value[0], "perform"):
        try:
            result = perform_substitutions(context, value)
            return result[:-5] if result.endswith("\n...\n") else result
        except SubstitutionFailure:
            return value
    if hasattr(value, "perform"):
        try:
            result = value.perform(context)
            return result[:-5] if result.endswith("\n...\n") else result
        except SubstitutionFailure:
            return value
    return value


def _nodes(launch_description):
    return [
        entity
        for entity in _all_actions(launch_description)
        if isinstance(entity, Node)
    ]


def _node_name(node):
    name = getattr(node, "_Node__node_name")
    if name is None:
        return None
    return _text(name)


def _node_parameters(node):
    result = {}
    for parameter_set in getattr(node, "_Node__parameters", []):
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            result[_text(key)] = _text(value)
    return result


def test_navigation_launch_omits_unused_waypoint_follower(monkeypatch):
    launch_description = _load_navigation_launch(monkeypatch)
    launched = {
        (node.node_package, node.node_executable, _node_name(node))
        for node in _nodes(launch_description)
    }

    assert (
        "nav2_waypoint_follower",
        "waypoint_follower",
        "waypoint_follower",
    ) not in launched


def test_lifecycle_manager_only_manages_used_navigation_nodes(monkeypatch):
    launch_description = _load_navigation_launch(monkeypatch)
    lifecycle = next(
        node for node in _nodes(launch_description)
        if _node_name(node) == "lifecycle_manager_navigation"
    )
    params = _node_parameters(lifecycle)

    assert params["node_names"] == [
        "controller_server",
        "smoother_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
    ]
    assert "waypoint_follower" not in params["node_names"]


def test_navigation_launch_keeps_required_nav2_runtime_nodes(monkeypatch):
    launch_description = _load_navigation_launch(monkeypatch)
    executables = {
        (node.node_package, node.node_executable)
        for node in _nodes(launch_description)
    }

    assert ("nav2_controller", "controller_server") in executables
    assert ("nav2_smoother", "smoother_server") in executables
    assert ("nav2_planner", "planner_server") in executables
    assert ("nav2_behaviors", "behavior_server") in executables
    assert ("nav2_bt_navigator", "bt_navigator") in executables
    assert ("nav2_velocity_smoother", "velocity_smoother") in executables
    assert ("nav2_lifecycle_manager", "lifecycle_manager") in executables


def test_navigation_nodes_respawn_for_gui_load_resilience(monkeypatch):
    """Lock the startup-resilience knobs added after the GUI demo hang."""
    # 2026-07-10 GUI 演示实测:gzclient 高负载下 lifecycle 编排超时挂死整栈。
    # respawn=True + bond 放宽是自愈防线,本测试防止其被再次写死为 False。
    launch_description = _load_navigation_launch(monkeypatch)
    own_executables = {
        "controller_server", "smoother_server", "planner_server",
        "behavior_server", "bt_navigator", "velocity_smoother",
    }
    # 只检查本 launch 手写的节点;localization include 属官方 launch 不管
    nav_nodes = [
        node for node in _nodes(launch_description)
        if node.node_package.startswith("nav2_")
        and node.node_executable in own_executables
    ]
    nav_nodes.append(next(
        node for node in _nodes(launch_description)
        if _node_name(node) == "lifecycle_manager_navigation"
    ))

    assert len(nav_nodes) == 7
    for node in nav_nodes:
        # respawn 实际存于 ExecuteLocal 基类(实测属性名)
        respawn = node._ExecuteLocal__respawn
        assert respawn is True, f"{node.node_executable} respawn off"

    lifecycle = next(
        node for node in _nodes(launch_description)
        if _node_name(node) == "lifecycle_manager_navigation"
    )
    params = _node_parameters(lifecycle)
    assert params["bond_timeout"] == 10.0
    assert params["attempt_respawn_reconnection"] is True
    assert params["bond_respawn_max_duration"] == 30.0


def test_lifecycle_manager_startup_is_delayed_past_boot_race(monkeypatch):
    """Lock the manager TimerAction delay that kills the boot race."""
    # GUI 实测:manager 在 launch 后 ~1s 编排,首次服务调用失败即弃栈。
    # 延后启动是根治手段,本测试防止 TimerAction 被去掉或缩短。
    from launch.actions import TimerAction as _Timer
    launch_description = _load_navigation_launch(monkeypatch)
    timers = [
        a for a in _all_actions(launch_description)
        if isinstance(a, _Timer)
        and any(
            _node_name(c) == "lifecycle_manager_navigation"
            for c in a._TimerAction__actions
        )
    ]

    assert len(timers) == 1
    assert float(timers[0]._TimerAction__period) >= 15.0
