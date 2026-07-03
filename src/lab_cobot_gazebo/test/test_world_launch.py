"""world.launch.py regression checks."""
from pathlib import Path
import re


def test_spawn_entity_waits_for_slow_gazebo_factory_startup():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "world.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    spawn_block = re.search(
        r"spawn_entity\s*=\s*Node\((?P<body>.*?)\n\s*\)",
        text,
        flags=re.DOTALL,
    )

    assert spawn_block is not None
    assert 'executable="spawn_entity.py"' in spawn_block.group("body")
    timeout_arg = re.search(
        r'"-timeout",\s*"(?P<seconds>\d+(?:\.\d+)?)"',
        spawn_block.group("body"),
    )
    assert timeout_arg is not None
    assert float(timeout_arg.group("seconds")) >= 90.0


def test_world_launch_spawns_wheel_velocity_controller():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "world.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert "wheel_velocity_controller" in text
    assert 'arguments=["wheel_velocity_controller", "-c", "/controller_manager"]' in text


def test_world_launch_starts_gzclient_with_sanitized_model_path():
    launch_file = Path(__file__).resolve().parents[1] / "launch" / "world.launch.py"
    text = launch_file.read_text(encoding="utf-8")

    assert "gzclient.launch.py" not in text
    assert "ExecuteProcess(" in text
    assert '"gzclient"' in text
    assert '"--gui-client-plugin=libgazebo_ros_eol_gui.so"' in text
    assert '"GAZEBO_MODEL_PATH": os.path.join(gz_pkg, "models")' in text
