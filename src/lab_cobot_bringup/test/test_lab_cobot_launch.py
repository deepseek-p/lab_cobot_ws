"""Regression checks for the integrated bringup launch file."""
from pathlib import Path


def test_navigation_include_pins_map_and_params_file():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert '"map": os.path.join(nav, "maps", "map.yaml")' in text
    assert '"params_file": os.path.join(nav, "config", "nav2_params.yaml")' in text


def test_bringup_disables_nav2_rviz_by_default_for_low_memory_runs():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("use_rviz", default_value="false"' in text
    assert '"use_rviz": use_rviz' in text


def test_mission_launch_does_not_globally_remap_internal_node_names():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert 'name="mission_node"' not in text


def test_bringup_drives_mecanum_wheel_visuals_from_cmd_vel():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert 'executable="mecanum_wheel_visualizer"' in text
    assert 'executable="wheel_joint_state_publisher"' not in text


def test_bringup_launches_gripper_attach_bridge():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert 'executable="gripper_attach_bridge"' in text


def test_bringup_uses_odom_frame_for_simulated_object_truth_tf():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert '"use_gazebo_model_pose": True' in text
    assert '"gazebo_model_name": "aruco_sample"' in text
    assert '"gazebo_reference_frame": "odom"' in text
    assert '"tf_reference_frame": "odom"' in text


def test_bringup_disables_gazebo_remote_model_database_for_gui_runs():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "lab_cobot.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert 'SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", "")' in text
