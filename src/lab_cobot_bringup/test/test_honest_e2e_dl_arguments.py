"""Honest E2E DL perception launch argument tests."""
import importlib.util
from pathlib import Path


def _load_honest_e2e_module():
    test_file = Path(__file__).resolve().parent / "test_honest_e2e_launch.py"
    spec = importlib.util.spec_from_file_location("honest_e2e_launch_unit", test_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_honest_e2e_enables_dl_perception_when_model_exists(tmp_path):
    module = _load_honest_e2e_module()
    model = tmp_path / "yolo_world_lab_slim.pt"
    model.write_bytes(b"fake")
    module.DL_MODEL_PATH = model

    args = module._dl_launch_arguments()

    assert args["use_dl_perception"] == "true"
    assert args["dl_device"] == "cpu"
    assert args["dl_imgsz"] == "640"


def test_honest_e2e_disables_dl_perception_when_model_is_missing(tmp_path):
    module = _load_honest_e2e_module()
    module.DL_MODEL_PATH = tmp_path / "missing.pt"

    args = module._dl_launch_arguments()

    assert args["use_dl_perception"] == "false"
    assert args["dl_device"] == "cpu"
    assert args["dl_imgsz"] == "640"
