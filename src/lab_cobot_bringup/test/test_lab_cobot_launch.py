"""Regression checks for the integrated bringup launch file."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node


BRINGUP = Path(__file__).resolve().parents[1]
SRC = BRINGUP.parent


def _load_bringup_launch(monkeypatch):
    launch_file = BRINGUP / "launch" / "lab_cobot.launch.py"
    spec = importlib.util.spec_from_file_location("lab_cobot_launch_test", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(
        module,
        "get_package_share_directory",
        lambda pkg: str(SRC / pkg),
    )
    return module.generate_launch_description()


def _entities(launch_description):
    for entity in launch_description.entities:
        yield entity
        if isinstance(entity, TimerAction):
            for action in entity.actions:
                yield action


def _text(value):
    context = LaunchContext()
    if isinstance(value, tuple):
        value = list(value)
    if isinstance(value, list) and value and hasattr(value[0], "perform"):
        value = perform_substitutions(context, value)
        if isinstance(value, str) and value.endswith("\n...\n"):
            return value[:-5]
        return value
    if hasattr(value, "perform"):
        value = value.perform(context)
        if isinstance(value, str) and value.endswith("\n...\n"):
            return value[:-5]
        return value
    return value


def _declared_defaults(launch_description):
    defaults = {}
    for entity in launch_description.entities:
        if hasattr(entity, "name") and entity.__class__.__name__ == "DeclareLaunchArgument":
            default = getattr(entity, "_DeclareLaunchArgument__default_value")
            defaults[entity.name] = _text(default)
    return defaults


def _include_arguments(include):
    return {name: value for name, value in include.launch_arguments}


def _nodes(launch_description):
    return [entity for entity in _entities(launch_description) if isinstance(entity, Node)]


def _node_parameters(node):
    result = {}
    for parameter_set in getattr(node, "_Node__parameters", []):
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            result[_text(key)] = _text(value)
    return result


def _node(package, executable, launch_description):
    for node in _nodes(launch_description):
        if node.node_package == package and node.node_executable == executable:
            return node
    raise AssertionError(f"missing node {package}/{executable}")


def test_navigation_include_pins_map_and_params_file(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    includes = [
        entity
        for entity in _entities(launch_description)
        if isinstance(entity, IncludeLaunchDescription)
    ]

    navigation = next(
        include for include in includes
        if "params_file" in _include_arguments(include)
    )
    args = _include_arguments(navigation)

    assert args["map"].endswith("lab_cobot_navigation/maps/map.yaml")
    assert args["params_file"].endswith("lab_cobot_navigation/config/nav2_params.yaml")


def test_bringup_disables_nav2_rviz_by_default_for_low_memory_runs(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    navigation = next(
        entity
        for entity in _entities(launch_description)
        if isinstance(entity, IncludeLaunchDescription)
        and "use_rviz" in _include_arguments(entity)
    )

    assert defaults["use_rviz"] == "false"
    assert isinstance(_include_arguments(navigation)["use_rviz"], LaunchConfiguration)


def test_mission_launch_does_not_globally_remap_internal_node_names(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    mission = _node("lab_cobot_bringup", "mission_node", launch_description)

    assert getattr(mission, "_Node__node_name") is None


def test_bringup_drives_mecanum_wheel_visuals_from_cmd_vel(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    executables = {node.node_executable for node in _nodes(launch_description)}

    assert "mecanum_wheel_visualizer" in executables
    assert "wheel_joint_state_publisher" not in executables


def test_bringup_launches_gripper_attach_bridge(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)

    assert _node("lab_cobot_bringup", "gripper_attach_bridge", launch_description)


def test_bringup_uses_odom_frame_for_simulated_object_truth_tf(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)

    aruco = _node("lab_cobot_perception", "aruco_detector", launch_description)
    aruco_params = _node_parameters(aruco)
    gripper = _node("lab_cobot_bringup", "gripper_attach_bridge", launch_description)
    gripper_params = _node_parameters(gripper)

    assert aruco_params["use_gazebo_model_pose"] is True
    assert aruco_params["gazebo_model_name"] == "aruco_sample"
    assert aruco_params["gazebo_reference_frame"] == "odom"
    assert gripper_params["tf_reference_frame"] == "odom"


def test_bringup_disables_gazebo_remote_model_database_for_gui_runs(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    env = {
        _text(getattr(entity, "_SetEnvironmentVariable__name")):
        _text(getattr(entity, "_SetEnvironmentVariable__value"))
        for entity in launch_description.entities
        if isinstance(entity, SetEnvironmentVariable)
    }

    assert env["GAZEBO_MODEL_DATABASE_URI"] == ""
