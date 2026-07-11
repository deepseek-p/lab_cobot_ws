"""YOLO-World backend wrapper."""
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class Detection2D:
    """A 2D detector output."""

    class_name: str
    conf: float
    xyxy: tuple[float, float, float, float]


def load_model(model_path, device):
    """Load a YOLO model and remember its resolved device."""
    from ultralytics import YOLO

    model = YOLO(str(model_path))
    model._lab_cobot_device = _resolve_device(device)
    return model


def infer(model, bgr, conf=0.05, imgsz=1280, device="auto"):
    """Run YOLO inference and convert the first result."""
    resolved_device = _resolve_device(device)
    results = model.predict(
        bgr,
        conf=float(conf),
        imgsz=int(imgsz),
        device=resolved_device,
        half=_uses_cuda_device(resolved_device),
        verbose=False,
    )
    if not results:
        return []
    return results_to_detections(results[0])


def results_to_detections(result):
    """Convert a YOLO result object to Detection2D values."""
    detections = []
    names = getattr(result, "names", {})
    for box in getattr(result, "boxes", []) or []:
        class_index = int(_scalar(getattr(box, "cls")))
        class_name = names.get(class_index, str(class_index))
        detections.append(Detection2D(
            class_name=str(class_name),
            conf=float(_scalar(getattr(box, "conf"))),
            xyxy=_xyxy(getattr(box, "xyxy")),
        ))
    return detections


def _resolve_device(device):
    if device != "auto":
        return device
    import torch

    return 0 if torch.cuda.is_available() else "cpu"


def _uses_cuda_device(device):
    return str(device).lower() not in {"cpu", "none"}


def _scalar(value: Any):
    if hasattr(value, "item"):
        return value.item()
    if isinstance(value, (list, tuple)) and len(value) == 1:
        return _scalar(value[0])
    return value


def _xyxy(value: Any) -> tuple[float, float, float, float]:
    if hasattr(value, "tolist"):
        value = value.tolist()
    if isinstance(value, (list, tuple)) and len(value) == 1:
        first = value[0]
        if isinstance(first, (list, tuple)):
            value = first
    if len(value) != 4:
        raise ValueError(f"expected xyxy with four values, got {value}")
    return tuple(float(coord) for coord in value)
