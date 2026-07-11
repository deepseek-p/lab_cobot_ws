from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
GAZEBO = ROOT / "lab_cobot_gazebo"
BRINGUP = ROOT / "lab_cobot_bringup"
NAV = ROOT / "lab_cobot_navigation"


def text(path):
    return path.read_text(encoding="utf-8")


def test_runtime_executables_are_built_and_installed():
    cmake = text(GAZEBO / "CMakeLists.txt")
    for executable in ("mecanum_gazebo_kinematic_drive", "gazebo_odom_bridge"):
        assert f"add_executable({executable}" in cmake
        assert executable in cmake.split("install(TARGETS", 1)[1]


def test_drive_preserves_confirmed_runtime_contract():
    source = text(GAZEBO / "src/mecanum_gazebo_kinematic_drive.cpp")
    for token in (
        '"/rover_twist"', '"/cmd_vel"',
        '"model_states_topic", "/model_states"',
        '"/set_entity_state"', '"model_name", "mecanum3"',
        '"max_vx", 0.5', '"max_vy", 0.3', '"max_wz", 1.2',
        '"max_accel_xy", 0.5', '"max_accel_wz", 1.5',
        '"command_timeout", 0.3', '"update_rate", 50.0',
        "cos_yaw * vx - sin_yaw * vy",
        "sin_yaw * vx + cos_yaw * vy",
    ):
        assert token in source


def test_drive_parameterizes_model_states_without_changing_source_defaults():
    source = text(GAZEBO / "src/mecanum_gazebo_kinematic_drive.cpp")
    assert '"model_states_topic", "/model_states"' in source
    subscription = (
        "create_subscription<gazebo_msgs::msg::ModelStates>"
        "(\n      model_states_topic_"
    )
    assert subscription in source


def test_bridge_is_parameterized_and_publishes_odom_tf():
    source = text(GAZEBO / "src/gazebo_odom_bridge.cpp")
    for token in (
        '"link_states_topic", "/gazebo/link_states"',
        '"odom_topic", "/odom"',
        '"target_link_name", "lab_cobot::base_footprint"',
        '"odom_frame", "odom"',
        '"base_frame", "base_footprint"',
        "create_subscription<gazebo_msgs::msg::LinkStates>",
        "create_publisher<nav_msgs::msg::Odometry>",
        "TransformBroadcaster",
        "sendTransform",
        '"fallback_link_name", "lab_cobot::base_link"',
        "matchesTargetLink",
    ):
        assert token in source
    assert '"fallback_link_name", "lab_cobot::base_link"' in source
    assert "findTargetLinkIndex" in source
    assert "model_prefix" in source


def test_world_launch_starts_gazebo_runtime_after_wheels():
    launch = text(GAZEBO / "launch/world.launch.py")
    assert 'executable="rover_twist_relay"' not in launch
    assert launch.count('executable="mecanum_gazebo_kinematic_drive"') == 1
    assert launch.count('executable="gazebo_odom_bridge"') == 1
    assert '"model_name": "lab_cobot"' in launch
    assert '"model_states_topic": "/gazebo/model_states"' in launch
    assert '"service_name": "/gazebo/set_entity_state"' in launch
    assert '"target_link_name": "lab_cobot::base_footprint"' in launch
    assert '"fallback_link_name": "lab_cobot::base_link"' in launch
    assert '"link_states_topic": "/gazebo/link_states"' in launch
    assert '"use_sim_time": True' in launch
    assert "target_action=wheel_velocity_controller" in launch
    wheel_event = launch.index("delay_wheel_velocity")
    assert "mecanum_kinematic_drive" in launch[wheel_event:]
    assert "/home/lenovo/mecanum_ws" not in launch


def test_integrated_launch_has_no_legacy_visualizer_or_duplicate_runtime():
    launch = text(BRINGUP / "launch/lab_cobot.launch.py")
    assert "mecanum_wheel_visualizer" not in launch
    assert "mecanum_gazebo_kinematic_drive" not in launch
    assert "gazebo_odom_bridge" not in launch
    assert launch.count('executable="rover_twist_relay"') == 1
    assert "/home/lenovo/mecanum_ws" not in launch


def test_gazebo_does_not_create_dependency_cycle_with_bringup():
    package = text(GAZEBO / "package.xml")
    # bringup already exec-depends on gazebo, so the reverse dependency makes
    # colcon's package graph impossible to order.
    assert "<exec_depend>lab_cobot_bringup</exec_depend>" not in package


def test_ekf_does_not_duplicate_odom_transform():
    config = text(NAV / "config/ekf.yaml")
    assert "publish_tf: false" in config


def test_drive_pauses_without_sim_time_and_limits_service_backlog():
    source = text(GAZEBO / "src/mecanum_gazebo_kinematic_drive.cpp")
    assert "if (dt <= 0.0) {\n      return;" in source
    assert "std::atomic_bool request_in_flight_" in source
    assert "request_in_flight_.exchange(true)" in source
    assert "request_in_flight_.store(false)" in source
    assert "rotateBaseToWorld" in source


def test_bridge_converts_world_twist_and_documents_safe_fallback():
    source = text(GAZEBO / "src/gazebo_odom_bridge.cpp")
    assert "rotateWorldToBase" in source
    assert "base_footprint and base_link have an identity fixed transform" in source


def test_mecanum3_fallback_link_has_identity_fixed_transform():
    xacro_path = (
        ROOT / "lab_cobot_description" / "urdf" / "inc" / "mecanum3_base.xacro"
    )
    xacro = text(xacro_path)
    joint = xacro.split('<joint name="base_joint" type="fixed">', 1)[1].split(
        "</joint>", 1
    )[0]
    assert '<parent link="base_footprint"/>' in joint
    assert '<child link="base_link"/>' in joint
    assert '<origin xyz="0 0 0" rpy="0 0 0"/>' in joint
