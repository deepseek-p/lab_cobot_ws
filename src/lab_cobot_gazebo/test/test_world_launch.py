"""Behavioural regressions for world.launch.py via launch introspection."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import ExecuteProcess, TimerAction
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

GAZEBO = Path(__file__).resolve().parents[1]


def _load_world_launch():
    launch_file = GAZEBO / "launch" / "world.launch.py"
    spec = importlib.util.spec_from_file_location("world_launch_test", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def _all_actions(launch_description):
    """Recursively expand timers and exit handlers into a flat action list."""
    actions = []

    def _walk(entity):
        actions.append(entity)
        if isinstance(entity, TimerAction):
            for child in entity.actions:
                _walk(child)
        handler = getattr(entity, "handler", None) or getattr(
            entity, "_RegisterEventHandler__event_handler", None
        )
        on_exit = getattr(handler, "_OnProcessExit__actions", None) or getattr(
            handler, "_OnActionEventBase__actions_on_event", None
        )
        for child in on_exit or []:
            _walk(child)

    for entity in launch_description.entities:
        _walk(entity)
    return actions


def _text_list(values):
    context = LaunchContext()
    out = []
    for value in values or []:
        if isinstance(value, (list, tuple)):
            out.append(perform_substitutions(context, list(value)))
        elif hasattr(value, "perform"):
            out.append(value.perform(context))
        else:
            out.append(str(value))
    return out


def _nodes(actions):
    return [action for action in actions if isinstance(action, Node)]


def _declared_defaults(launch_description):
    defaults = {}
    context = LaunchContext()
    for entity in launch_description.entities:
        if entity.__class__.__name__ != "DeclareLaunchArgument":
            continue
        value = getattr(entity, "_DeclareLaunchArgument__default_value")
        defaults[entity.name] = perform_substitutions(context, list(value))
    return defaults


def _robot_description_command(launch_description):
    actions = _all_actions(launch_description)
    publisher = next(
        node
        for node in _nodes(actions)
        if node.node_executable == "robot_state_publisher"
    )
    for parameter_set in getattr(publisher, "_Node__parameters", []):
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            key_text = perform_substitutions(LaunchContext(), list(key))
            if key_text == "robot_description":
                return value[0]
    raise AssertionError("missing robot_description Command")


def _robot_description_command_text(overrides=None):
    launch_description = _load_world_launch()
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    context.launch_configurations.update(overrides or {})
    command = _robot_description_command(launch_description)
    return perform_substitutions(context, command.command)


def _spawn_entity_args():
    actions = _all_actions(_load_world_launch())
    spawns = [
        node for node in _nodes(actions)
        if getattr(node, "node_executable", "") == "spawn_entity.py"
    ]
    assert len(spawns) == 1
    return _text_list(spawns[0]._Node__arguments)


def test_spawn_entity_waits_for_slow_gazebo_factory_startup():
    args = _spawn_entity_args()
    assert "-timeout" in args
    assert float(args[args.index("-timeout") + 1]) >= 90.0


def test_spawn_entity_places_base_footprint_on_ground():
    args = _spawn_entity_args()
    assert "-z" in args
    assert float(args[args.index("-z") + 1]) == 0.0


def test_world_launch_spawns_wheel_velocity_controller():
    actions = _all_actions(_load_world_launch())
    spawner_args = [
        _text_list(node._Node__arguments)
        for node in _nodes(actions)
        if getattr(node, "node_executable", "") == "spawner"
    ]
    assert any("wheel_velocity_controller" in args for args in spawner_args)


def test_world_launch_has_no_default_set_entity_state_calls():
    """Default path must not contain SetEntityState pose interventions."""
    # 原 12s 复位死代码已删(位姿驱动下一 tick 即被插件覆盖,自相矛盾)
    actions = _all_actions(_load_world_launch())
    for action in actions:
        if isinstance(action, ExecuteProcess):
            cmd = " ".join(str(part) for part in _text_list(action.process_description.cmd))
            assert "set_entity_state" not in cmd


def test_world_launch_starts_gzclient_with_sanitized_model_path():
    actions = _all_actions(_load_world_launch())
    gzclients = []
    for action in actions:
        if isinstance(action, ExecuteProcess):
            cmd = _text_list(action.process_description.cmd)
            if any("gzclient" in str(part) for part in cmd):
                gzclients.append((action, cmd))
    assert len(gzclients) == 1
    action, cmd = gzclients[0]
    assert any(
        "--gui-client-plugin=libgazebo_ros_eol_gui.so" in str(part) for part in cmd
    )
    env_pairs = list(action.process_description.env or []) + list(
        getattr(action.process_description, "additional_env", None) or []
    )
    env_keys = set()
    for key, _value in env_pairs:
        if isinstance(key, str):
            env_keys.add(key)
        else:
            env_keys.add(perform_substitutions(LaunchContext(), list(key)))
    assert "GAZEBO_MODEL_PATH" in env_keys


def test_world_robot_description_disables_wrist_camera_by_default():
    command = _robot_description_command_text()

    assert "wrist_refine_camera:=false" in command


def test_world_robot_description_enables_wrist_camera_from_shared_switch():
    command = _robot_description_command_text({"use_refine_detect": "true"})

    assert "wrist_refine_camera:=true" in command
