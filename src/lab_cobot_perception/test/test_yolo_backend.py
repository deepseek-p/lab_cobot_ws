"""YOLO backend tests."""
import ast
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lab_cobot_perception import yolo_backend  # noqa: E402


class FakeTensor:
    def __init__(self, value):
        self.value = value

    def item(self):
        return self.value

    def tolist(self):
        return self.value


class FakeBox:
    def __init__(self, cls, conf, xyxy):
        self.cls = FakeTensor(cls)
        self.conf = FakeTensor(conf)
        self.xyxy = FakeTensor([xyxy])


class FakeResult:
    names = {0: "white box", 1: "blue cylinder"}

    def __init__(self):
        self.boxes = [
            FakeBox(1, 0.42, [10.0, 20.0, 30.0, 40.0]),
            FakeBox(0, 0.18, [50.0, 60.0, 70.0, 80.0]),
        ]


class FakeModel:
    def __init__(self, result):
        self.result = result
        self.calls = []

    def predict(self, *args, **kwargs):
        self.calls.append((args, kwargs))
        return [self.result]


def test_results_to_detections_converts_duck_typed_result():
    detections = yolo_backend.results_to_detections(FakeResult())

    assert detections == [
        yolo_backend.Detection2D(
            class_name="blue cylinder",
            conf=0.42,
            xyxy=(10.0, 20.0, 30.0, 40.0),
        ),
        yolo_backend.Detection2D(
            class_name="white box",
            conf=0.18,
            xyxy=(50.0, 60.0, 70.0, 80.0),
        ),
    ]


def test_infer_calls_model_predict_and_converts_first_result():
    result = FakeResult()
    model = FakeModel(result)
    image = object()

    detections = yolo_backend.infer(
        model,
        image,
        conf=0.05,
        imgsz=640,
        device="cpu",
    )

    assert detections[0].class_name == "blue cylinder"
    assert model.calls == [((image,), {
        "conf": 0.05,
        "imgsz": 640,
        "device": "cpu",
        "half": False,
        "verbose": False,
    })]


def test_infer_enables_half_precision_on_cuda_device():
    result = FakeResult()
    model = FakeModel(result)

    yolo_backend.infer(
        model,
        object(),
        conf=0.05,
        imgsz=1280,
        device=0,
    )

    assert model.calls[0][1]["half"] is True


def test_module_top_level_has_no_heavy_imports():
    source = Path(yolo_backend.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add((node.module or "").split(".")[0])

    assert imports <= {"dataclasses", "typing"}
