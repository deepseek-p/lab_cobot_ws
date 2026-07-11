"""Honest E2E diagnostics unit tests."""
import importlib.util
from pathlib import Path


BRINGUP = Path(__file__).resolve().parents[1]


def _load_honest_e2e_module():
    test_file = BRINGUP / "test" / "test_honest_e2e_launch.py"
    spec = importlib.util.spec_from_file_location("honest_e2e_diagnostics", test_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_gazebo_residue_error_uses_exact_process_cleanup():
    """Keep Gazebo residue cleanup guidance off pkill -f."""
    module = _load_honest_e2e_module()

    message = module._gazebo_residue_error("123\n456\n")

    assert "pid: 123 456" in message
    assert "pkill -9 -x gzserver" in message
    assert "pkill -9 -x gzclient" in message
    assert "pkill -9 -f" not in message
