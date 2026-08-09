#!/usr/bin/env python3
"""ROS-independent wrapper for the project's trained YOLO detector."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Detection:
    """A semantic label, confidence, and pixel-space bounding box."""

    label: str
    confidence: float
    box: tuple[int, int, int, int]


def box_iou(first, second) -> float:
    """Return the intersection-over-union of two ``xyxy`` boxes."""
    ax1, ay1, ax2, ay2 = [float(value) for value in first]
    bx1, by1, bx2, by2 = [float(value) for value in second]
    left, top = max(ax1, bx1), max(ay1, by1)
    right, bottom = min(ax2, bx2), min(ay2, by2)
    intersection = max(0.0, right - left) * max(0.0, bottom - top)
    first_area = max(0.0, ax2 - ax1) * max(0.0, ay2 - ay1)
    second_area = max(0.0, bx2 - bx1) * max(0.0, by2 - by1)
    union = first_area + second_area - intersection
    return intersection / union if union > 0.0 else 0.0


def deduplicate_detections(
    detections: list[Detection],
    iou_threshold: float = 0.65,
) -> list[Detection]:
    """Keep the highest-confidence overlapping box for each label."""
    kept = []
    for detection in sorted(detections, key=lambda item: item.confidence, reverse=True):
        overlaps_same_label = any(
            detection.label == existing.label
            and box_iou(detection.box, existing.box) >= iou_threshold
            for existing in kept
        )
        if not overlaps_same_label:
            kept.append(detection)
    return kept


def select_best_candidate(candidates, target_label: str = ""):
    """Select one deterministic candidate matching the requested label."""
    wanted = str(target_label).strip().lower()
    eligible = [
        candidate for candidate in candidates
        if not wanted or str(candidate["label"]).strip().lower() == wanted
    ]
    if not eligible:
        return None
    return max(
        enumerate(eligible),
        key=lambda indexed: (float(indexed[1]["confidence"]), -indexed[0]),
    )[1]


def resolve_model_path(model_path: str) -> str:
    """Resolve an external model path without downloading model data."""
    path = Path(model_path).expanduser()
    if path.is_file():
        return str(path)
    raise FileNotFoundError(
        f"YOLO weight file not found: {model_path}. "
        "Copy the eight-class weight into ~/lab_cobot_models/."
    )


class YoloDetector:
    """Run a fixed-class Ultralytics YOLO model trained for this project."""

    def __init__(
        self,
        model_path: str,
        device: str,
    ):
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "The trained detector requires ultralytics, torch, and numpy"
            ) from exc
        self.model = YOLO(resolve_model_path(model_path))
        # Keep device selection explicit; do not modify torch's CUDA probes.
        # The previous implementation forcibly disabled CUDA for the process.
        if device == "auto":
            cuda = torch.cuda
            cuda_available = cuda.is_available()
            self.device = 0 if cuda_available else "cpu"
        else:
            self.device = device

    def infer(self, image: np.ndarray, confidence: float, nms_iou: float,
              imgsz: int) -> list[Detection]:
        result = self.model.predict(
            source=image,
            conf=confidence,
            iou=nms_iou,
            imgsz=imgsz,
            device=self.device,
            verbose=False,
        )[0]
        if result.boxes is None:
            return []
        return [
            Detection(
                str(result.names[int(box.cls[0].item())]),
                float(box.conf[0].item()),
                tuple(
                    int(value) for value in box.xyxy[0].cpu().numpy().round()
                ),
            )
            for box in result.boxes
        ]
