#!/usr/bin/env python3
"""ROS-independent YOLO-World inference wrapper."""
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: tuple[int, int, int, int]


class YoloWorldDetector:
    def __init__(self, model_path: str, target_classes: Sequence[str],
                 device: str):
        try:
            import torch
            from ultralytics import YOLOWorld
        except ImportError as exc:
            raise RuntimeError(
                "YOLO-World requires recent ultralytics with YOLOWorld"
            ) from exc
        prompts = [value.strip() for value in target_classes if value.strip()]
        if not prompts:
            raise ValueError(
                "target_classes must contain at least one text prompt"
            )
        self.model = YOLOWorld(model_path)
        self.model.set_classes(prompts)
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
