"""Regression checks for the integrated bringup launch file."""
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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


def _node_name(node):
    value = getattr(node, "_Node__node_name", None)
    return None if value is None else _text(value)


def _active_nodes(launch_description, overrides=None):
    context = LaunchContext()
    context.launch_configurations.update(_declared_defaults(launch_description))
    context.launch_configurations.update(overrides or {})
    return [
        node
        for node in _nodes(launch_description)
        if node.condition is None or node.condition.evaluate(context)
    ]


def _node_parameters(node):
    result = {}
    for parameter_set in getattr(node, "_Node__parameters", []):
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            result[_text(key)] = _text(value)
    return result


def _node_parameters_raw(node):
    result = {}
    for parameter_set in getattr(node, "_Node__parameters", []):
        if not isinstance(parameter_set, dict):
            continue
        for key, value in parameter_set.items():
            if isinstance(value, tuple) and len(value) == 1:
                value = value[0]
            result[_text(key)] = value if isinstance(value, LaunchConfiguration) else _text(value)
    return result


def _parameter_value_launch_configuration(value):
    if isinstance(value, ParameterValue):
        inner = value.value
        assert len(inner) == 1
        return inner[0]
    return value


def _node(package, executable, launch_description):
    for node in _nodes(launch_description):
        if node.node_package == package and node.node_executable == executable:
            return node
    raise AssertionError(f"missing node {package}/{executable}")


def test_navigation_include_pins_map_and_params_file(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
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

    assert defaults["map"].endswith("lab_cobot_navigation/maps/map.yaml")
    assert isinstance(args["map"], LaunchConfiguration)
    assert _text(args["map"].variable_name) == "map"
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


def test_mission_launch_is_guarded_by_launch_mission_argument(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    mission = _node("lab_cobot_bringup", "mission_node", launch_description)

    assert isinstance(mission.condition, IfCondition)
    predicate = getattr(mission.condition, "_IfCondition__predicate_expression")
    assert len(predicate) == 1
    assert isinstance(predicate[0], LaunchConfiguration)
    assert _text(predicate[0].variable_name) == "launch_mission"


def test_bringup_owns_single_relay_and_delegates_drive_to_world(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    executables = [node.node_executable for node in _nodes(launch_description)]
    includes = [
        entity
        for entity in _entities(launch_description)
        if isinstance(entity, IncludeLaunchDescription)
    ]
    worlds = [
        include
        for include in includes
        if _text(getattr(
            include.launch_description_source,
            "_LaunchDescriptionSource__location",
        )).endswith("world.launch.py")
    ]

    assert len(worlds) == 1
    assert "mecanum_wheel_visualizer" not in executables
    assert "wheel_joint_state_publisher" not in executables
    assert list(executables).count("rover_twist_relay") == 1
    assert "mecanum_gazebo_kinematic_drive" not in executables
    assert "gazebo_odom_bridge" not in executables

    relay = _node("lab_cobot_bringup", "rover_twist_relay", launch_description)
    assert _node_parameters(relay) == {
        "use_sim_time": True,
        "rover": "mecanum3",
        "mecanum3.wheel_radius": 0.07,
        "mecanum3.wheel_separation_width": 0.24,
        "mecanum3.wheel_separation_length": 0.175,
        "max_vx": 0.5,
        "max_vy": 0.3,
        "max_wz": 1.2,
        "max_accel_xy": 0.5,
        "max_accel_wz": 1.5,
        "command_timeout": 0.25,
    }


def test_bringup_registers_worktables_in_existing_stage2(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    stage2 = next(
        entity
        for entity in launch_description.entities
        if isinstance(entity, TimerAction) and entity.period == 10.0
    )
    table_initializer = _node(
        "lab_cobot_moveit", "table_scene_initializer", launch_description
    )

    assert table_initializer in stage2.actions
    assert _node_parameters(table_initializer) == {
        "use_sim_time": True,
        "world_frame": "odom",
    }


def test_bringup_publishes_passive_mecanum_joint_states(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    nodes = [
        node for node in _nodes(launch_description)
        if node.node_executable == "passive_mecanum_joint_states"
    ]

    assert len(nodes) == 1
    assert nodes[0].node_package == "lab_cobot_bringup"
    assert _node_parameters(nodes[0])["use_sim_time"] is True


def test_bringup_keeps_sim_attach_bridge_as_explicit_debug_option(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    bridge = _node("lab_cobot_bringup", "gripper_attach_bridge", launch_description)

    assert defaults["use_sim_attach"] == "false"
    assert isinstance(bridge.condition, IfCondition)
    predicate = getattr(bridge.condition, "_IfCondition__predicate_expression")
    assert len(predicate) == 1
    assert isinstance(predicate[0], LaunchConfiguration)
    assert _text(predicate[0].variable_name) == "use_sim_attach"


def test_bringup_defaults_to_camera_aruco_with_truth_as_debug_option(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)

    aruco = _node("lab_cobot_perception", "aruco_detector", launch_description)
    aruco_params = _node_parameters_raw(aruco)
    gripper = _node("lab_cobot_bringup", "gripper_attach_bridge", launch_description)
    gripper_params = _node_parameters(gripper)

    assert defaults["use_truth_pose"] == "false"
    assert isinstance(aruco_params["use_gazebo_model_pose"], LaunchConfiguration)
    assert _text(aruco_params["use_gazebo_model_pose"].variable_name) == "use_truth_pose"
    assert aruco_params["gazebo_model_name"] == "aruco_sample"
    assert aruco_params["gazebo_reference_frame"] == "odom"
    assert aruco_params["rgb_topic"] == "/bench_camera/image_raw"
    assert aruco_params["depth_topic"] == "/bench_camera/depth/image_raw"
    assert aruco_params["info_topic"] == "/bench_camera/camera_info"
    assert aruco_params["optical_frame"] == "camera_optical_frame"
    assert aruco_params["target_frame"] == "base_link"
    assert aruco_params["marker_size_m"] == 0.07 * (240.0 / 312.0)
    assert gripper_params["tf_reference_frame"] == "odom"


def test_bringup_launches_dl_object_detector_by_default(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)

    detector = _node("lab_cobot_perception", "object_detector", launch_description)
    params = _node_parameters_raw(detector)

    assert defaults["use_dl_perception"] == "true"
    assert defaults["dl_device"] == "auto"
    assert defaults["dl_imgsz"] == "1280"
    assert isinstance(detector.condition, IfCondition)
    predicate = getattr(detector.condition, "_IfCondition__predicate_expression")
    assert len(predicate) == 1
    assert isinstance(predicate[0], LaunchConfiguration)
    assert _text(predicate[0].variable_name) == "use_dl_perception"
    assert isinstance(params["device"], LaunchConfiguration)
    assert _text(params["device"].variable_name) == "dl_device"
    assert isinstance(params["imgsz"], LaunchConfiguration)
    assert _text(params["imgsz"].variable_name) == "dl_imgsz"
    assert params["rgb_topic"] == "/bench_camera/image_raw"
    assert params["depth_topic"] == "/bench_camera/depth/image_raw"
    assert params["info_topic"] == "/bench_camera/camera_info"
    assert params["optical_frame"] == "camera_optical_frame"
    assert params["target_frame"] == "base_link"


def test_bringup_passes_target_object_to_mission(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    mission = _node("lab_cobot_bringup", "mission_node", launch_description)
    params = _node_parameters_raw(mission)

    assert defaults["target_object"] == "aruco_sample"
    assert isinstance(params["target_object"], LaunchConfiguration)
    assert _text(params["target_object"].variable_name) == "target_object"


def test_bringup_declares_tactile_grasp_enabled_by_default(monkeypatch):
    # 2026-07-10 T-5 翻默认:触觉步进闭合+双指接触门控为默认;两开关必须一起为 true
    # (只开门控不开触觉时固定闭合 0.009 永不接触,正常抓取全失败)。
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    mission = _node("lab_cobot_bringup", "mission_node", launch_description)
    params = _node_parameters_raw(mission)

    assert defaults["use_tactile_grasp"] == "true"
    assert defaults["require_finger_contact"] == "true"
    use_tactile = _parameter_value_launch_configuration(params["use_tactile_grasp"])
    assert isinstance(use_tactile, LaunchConfiguration)
    assert _text(use_tactile.variable_name) == "use_tactile_grasp"


def test_bringup_disables_wrist_refine_pipeline_by_default(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    mission = _node("lab_cobot_bringup", "mission_node", launch_description)
    mission_params = _node_parameters_raw(mission)
    world = next(
        entity
        for entity in _entities(launch_description)
        if isinstance(entity, IncludeLaunchDescription)
        and "require_finger_contact" in _include_arguments(entity)
    )
    world_args = _include_arguments(world)

    assert defaults["use_refine_detect"] == "false"
    assert "wrist_aruco_detector" not in {
        _node_name(node) for node in _active_nodes(launch_description)
    }
    mission_refine = _parameter_value_launch_configuration(
        mission_params["use_refine_detect"]
    )
    assert isinstance(mission_refine, LaunchConfiguration)
    assert _text(mission_refine.variable_name) == "use_refine_detect"
    assert isinstance(world_args["use_refine_detect"], LaunchConfiguration)
    assert _text(world_args["use_refine_detect"].variable_name) == (
        "use_refine_detect"
    )


def test_bringup_enables_configured_wrist_aruco_instance(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    active = _active_nodes(
        launch_description,
        {"use_refine_detect": "true"},
    )
    wrist = next(
        node for node in active if _node_name(node) == "wrist_aruco_detector"
    )
    params = _node_parameters_raw(wrist)

    assert wrist.node_package == "lab_cobot_perception"
    assert wrist.node_executable == "aruco_detector"
    assert params["use_sim_time"] is True
    assert params["topic_namespace"] == "/perception/wrist"
    assert params["publish_tf"] is False
    assert params["rgb_topic"] == "/wrist_camera/image_raw"
    assert params["depth_topic"] == "/wrist_camera/depth/image_raw"
    assert params["info_topic"] == "/wrist_camera/camera_info"
    assert params["optical_frame"] == "wrist_camera_optical_frame"
    assert params["target_frame"] == "base_link"
    assert params["marker_size_m"] == 0.07 * (240.0 / 312.0)
    assert "use_gazebo_model_pose" not in params


def test_bringup_disables_voice_entry_by_default(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    defaults = _declared_defaults(launch_description)
    voice = _node("lab_cobot_bringup", "voice_node", launch_description)
    params = _node_parameters_raw(voice)

    assert defaults["launch_voice"] == "false"
    assert defaults["voice_audio_file"] == ""
    assert isinstance(voice.condition, IfCondition)
    predicate = getattr(voice.condition, "_IfCondition__predicate_expression")
    assert len(predicate) == 1
    assert isinstance(predicate[0], LaunchConfiguration)
    assert _text(predicate[0].variable_name) == "launch_voice"
    assert isinstance(params["audio_file"], LaunchConfiguration)
    assert _text(params["audio_file"].variable_name) == "voice_audio_file"


def test_bringup_disables_gazebo_remote_model_database_for_gui_runs(monkeypatch):
    launch_description = _load_bringup_launch(monkeypatch)
    env = {
        _text(getattr(entity, "_SetEnvironmentVariable__name")):
        _text(getattr(entity, "_SetEnvironmentVariable__value"))
        for entity in launch_description.entities
        if isinstance(entity, SetEnvironmentVariable)
    }

    assert env["GAZEBO_MODEL_DATABASE_URI"] == ""
