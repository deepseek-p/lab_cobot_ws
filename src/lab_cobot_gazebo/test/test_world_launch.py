"""Behavioural regressions for world.launch.py via launch introspection."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import EmitEvent, ExecuteProcess, OpaqueFunction, TimerAction
from launch.events import Shutdown
from launch.utilities import perform_substitutions
from launch_ros.actions import Node

GAZEBO = Path(__file__).resolve().parents[1]


def _load_world_launch():
    launch_file = GAZEBO / "launch" / "world.launch.py"
    spec = importlib.util.spec_from_file_location("world_launch_test", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description(), module


def test_world_launch_provides_offline_gazebo_resources():
    source = (GAZEBO / "launch" / "world.launch.py").read_text(encoding="utf-8")
    assert '"GAZEBO_RESOURCE_PATH", "/usr/share/gazebo-11"' in source
    assert '"GAZEBO_MODEL_PATH", "/usr/share/gazebo-11/models"' in source
    assert 'SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")' in source
    assert '"GAZEBO_MODEL_PATH", os.path.join(gz_pkg, "models")' in source


def _all_actions(launch_description):
    actions = []
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))

    def _walk(entity):
        actions.append(entity)
        if isinstance(entity, TimerAction):
            for child in entity.actions:
                _walk(child)
        if isinstance(entity, OpaqueFunction):
            for child in entity.execute(context):
                _walk(child)
        handler = getattr(entity, "handler", None) or getattr(
            entity, "_RegisterEventHandler__event_handler", None
        )
        on_exit = getattr(handler, "_OnProcessExit__actions", None) or getattr(
            handler, "_OnActionEventBase__actions_on_event", None
        )
        callback = getattr(handler, "_OnActionEventBase__on_event", None)
        if callback is not None:
            on_exit = callback(
                type("Event", (), {"returncode": 0})(), context
            )
        for child in on_exit or []:
            _walk(child)

    for entity in launch_description.entities:
        _walk(entity)
    return actions


def _text_list(values, context=None):
    context = context or LaunchContext()
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
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    publisher = next(
        node
        for node in _nodes(actions)
        if node.node_executable == "robot_state_publisher"
    )
    for parameter_set in getattr(publisher, "_Node__parameters", []):
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            key_text = perform_substitutions(context, list(key))
            if key_text == "robot_description":
                return value[0]
    raise AssertionError("missing robot_description Command")


def _robot_description_command_text(overrides=None):
    launch_description, _module = _load_world_launch()
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    context.launch_configurations.update(overrides or {})
    command = _robot_description_command(launch_description)
    return perform_substitutions(context, command.command)


def _spawn_entity_args():
    launch_description, _module = _load_world_launch()
    actions = _all_actions(launch_description)
    spawns = [
        node for node in _nodes(actions)
        if getattr(node, "node_executable", "") == "spawn_entity.py"
    ]
    assert len(spawns) == 1
    return _text_list(spawns[0]._Node__arguments)


def test_world_filename_mapper_supports_three_light_profiles_and_actor_switch():
    _launch_description, module = _load_world_launch()
    assert module._world_filename_from_profile("normal", False) == "lab.world"
    assert module._world_filename_from_profile("normal", True) == "lab_actor.world"
    assert module._world_filename_from_profile("dark", False) == "lab_dark.world"
    assert module._world_filename_from_profile("dark", True) == "lab_dark_actor.world"
    assert module._world_filename_from_profile("reflective", False) == "lab_reflective.world"
    assert module._world_filename_from_profile("reflective", True) == "lab_reflective_actor.world"


def test_world_launch_declares_new_environment_switches():
    launch_description, _module = _load_world_launch()
    defaults = _declared_defaults(launch_description)
    assert defaults["lighting_profile"] == "normal"
    assert defaults["enable_actor"] == "false"


def test_spawn_entity_waits_for_slow_gazebo_factory_startup():
    args = _spawn_entity_args()
    assert "-timeout" in args
    assert float(args[args.index("-timeout") + 1]) >= 90.0


def test_spawn_entity_places_main_base_footprint_on_ground():
    args = _spawn_entity_args()
    assert "-x" in args
    assert float(args[args.index("-x") + 1]) == 2.25
    assert "-y" in args
    assert float(args[args.index("-y") + 1]) == -2.10
    assert "-z" in args
    assert float(args[args.index("-z") + 1]) == 0.0


def test_world_launch_spawns_wheel_velocity_controller():
    launch_description, _module = _load_world_launch()
    actions = _all_actions(launch_description)
    spawner_args = [
        _text_list(node._Node__arguments)
        for node in _nodes(actions)
        if getattr(node, "node_executable", "") == "spawner"
    ]
    assert any("wheel_velocity_controller" in args for args in spawner_args)


def test_world_launch_does_not_start_asynchronous_pose_service_driver():
    launch_description, _module = _load_world_launch()
    actions = _all_actions(launch_description)
    assert not any(
        getattr(node, "node_executable", "") == "mecanum_gazebo_kinematic_drive"
        for node in _nodes(actions)
    )


def test_controller_chain_continues_only_after_success():
    _launch_description, loaded = _load_world_launch()

    sentinel = object()
    success = loaded._continue_on_success(
        type("Event", (), {"returncode": 0})(), [sentinel], "wheel"
    )
    assert success == [sentinel]
    failure = loaded._continue_on_success(
        type("Event", (), {"returncode": 1})(), [sentinel], "wheel"
    )
    assert len(failure) == 1
    assert isinstance(failure[0], EmitEvent)
    assert isinstance(failure[0].event, Shutdown)


def test_world_launch_has_no_default_set_entity_state_calls():
    launch_description, _module = _load_world_launch()
    actions = _all_actions(launch_description)
    for action in actions:
        if isinstance(action, ExecuteProcess):
            cmd = " ".join(str(part) for part in _text_list(action.process_description.cmd))
            assert "set_entity_state" not in cmd


def test_world_launch_gzclient_inherits_complete_model_path():
    launch_description, _module = _load_world_launch()
    actions = _all_actions(launch_description)
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    gzclients = []
    for action in actions:
        if isinstance(action, ExecuteProcess):
            cmd = _text_list(action.process_description.cmd, context)
            if any("gzclient" in str(part) for part in cmd):
                gzclients.append((action, cmd))
    assert len(gzclients) == 1
    action, cmd = gzclients[0]
    assert any(
        "--gui-client-plugin=libgazebo_ros_eol_gui.so" in str(part) for part in cmd
    )
    env_pairs = list(
        getattr(action.process_description, "additional_env", None) or []
    )
    env_keys = set()
    for key, _value in env_pairs:
        if isinstance(key, str):
            env_keys.add(key)
        else:
            env_keys.add(perform_substitutions(context, list(key)))
    assert "GAZEBO_MODEL_PATH" not in env_keys


def test_world_robot_description_disables_wrist_camera_by_default():
    command = _robot_description_command_text()
    assert "wrist_refine_camera:=false" in command


def test_world_robot_description_enables_wrist_camera_from_shared_switch():
    command = _robot_description_command_text({"use_refine_detect": "true"})
    assert "wrist_refine_camera:=true" in command


def test_world_robot_description_enables_wrist_camera_for_wrist_detect():
    command = _robot_description_command_text({"use_wrist_detect": "true"})
    assert "wrist_refine_camera:=true" in command
