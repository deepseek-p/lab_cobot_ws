#!/usr/bin/env python3
"""ROS-independent wrapper for the project's trained YOLO detector."""
from dataclasses import dataclass
from pathlib import Path

import numpy as np


_GEOMETRY_FALLBACK_ASPECT_RANGES = {
    # These bounds describe the rendered object silhouette, not Gazebo truth.
    # They are deliberately loose enough for an oblique wrist view while
    # rejecting table edges and isolated highlights.
    "tooling_fixture_box": (1.0, 6.0),
    "tooling_hand_tools": (1.3, 8.0),
    "board_test_fixture": (1.0, 6.0),
    "high_voltage_probe_kit": (2.0, 9.0),
    "material_spare_igbt": (1.2, 5.0),
    "aging_rack": (1.0, 3.5),
    "pcb_board": (1.0, 4.0),
    "test_tube_rack": (1.5, 8.0),
    "test_tube": (1.5, 10.0),
    "beaker": (1.0, 4.0),
    "erlenmeyer_flask": (1.0, 4.0),
    "graduated_cylinder": (1.5, 10.0),
}


@dataclass(frozen=True)
class Detection:
    label: str
    confidence: float
    box: tuple[int, int, int, int]
    source: str = "yolo"


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
    """Keep the highest-confidence overlapping box for each semantic label.

    The trained detector and the deterministic ArUco fallback can both report
    the same marked sample.  Publishing both would create duplicate 3D
    objects and make a one-target grasp interface non-deterministic.
    """
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
    """Select one deterministic target candidate from 3D pose candidates.

    ``target_label`` is optional.  When empty, the most confident candidate
    is selected; otherwise only the exact case-insensitive semantic label is
    eligible.  A tie retains the earlier detector result.
    """
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


def select_centered_target_detection(
    detections: list[Detection],
    target_label: str,
    image_width: int,
    image_height: int,
    max_center_fraction: float = 0.38,
) -> Detection | None:
    """Associate the aimed wrist view with one same-class image instance.

    During fine inspection the cruise controller has already pointed the
    optical axis at exactly one Gazebo item.  Confidence alone is therefore a
    poor association key: repeated lab glassware and neighbouring tools can
    have a stronger box near an image edge.  Keep only the requested class,
    reject boxes outside a bounded central gate, then choose the box whose
    centre is nearest the optical axis.  Confidence is used only as a tie
    breaker and remains available to downstream audit/reporting.
    """
    wanted = str(target_label).strip().lower()
    if not wanted or image_width <= 0 or image_height <= 0:
        return None
    centre_x = float(image_width) / 2.0
    centre_y = float(image_height) / 2.0
    radius = max(1.0, float(max_center_fraction) * min(
        float(image_width), float(image_height)))
    eligible = []
    for index, detection in enumerate(detections):
        if str(detection.label).strip().lower() != wanted:
            continue
        x1, y1, x2, y2 = (float(value) for value in detection.box)
        if x2 <= x1 or y2 <= y1:
            continue
        distance = ((x1 + x2) / 2.0 - centre_x) ** 2 + \
            ((y1 + y2) / 2.0 - centre_y) ** 2
        if distance <= radius * radius:
            eligible.append((distance, -float(detection.confidence), index,
                             detection))
    if not eligible:
        return None
    return min(eligible, key=lambda value: value[:3])[3]


def detect_center_foreground_geometry(
    image: np.ndarray,
    target_label: str,
    color_distance_threshold: float = 18.0,
    min_area_fraction: float = 0.008,
    max_area_fraction: float = 0.55,
) -> Detection | None:
    """Return a centre-near foreground silhouette for weak YOLO classes.

    The aimed wrist images place one target on a nearly uniform work surface.
    When the trained network produces no usable same-class box, segmenting the
    colour difference from that surface yields an independently measured 2-D
    region.  This is intentionally labelled ``geometry_foreground`` so it can
    never be reported as a YOLO detection.  No Gazebo pose, projection, model
    name, or truth coordinate enters this calculation.
    """
    wanted = str(target_label).strip().lower()
    aspect_limits = _GEOMETRY_FALLBACK_ASPECT_RANGES.get(wanted)
    if aspect_limits is None or image is None or image.ndim != 3:
        return None
    import cv2

    height, width = image.shape[:2]
    if height < 32 or width < 32:
        return None
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB).astype(np.float32)
    # Four patches halfway between the optical axis and the image border are
    # much less likely than the corners to contain walls or the robot itself.
    patches = []
    radius = max(6, min(height, width) // 40)
    for row, column in (
        (height // 4, width // 2),
        (3 * height // 4, width // 2),
        (height // 2, width // 4),
        (height // 2, 3 * width // 4),
    ):
        patches.append(lab[
            max(0, row - radius):min(height, row + radius + 1),
            max(0, column - radius):min(width, column + radius + 1),
        ].reshape(-1, 3))
    background = np.median(np.concatenate(patches), axis=0)
    distance = np.linalg.norm(lab - background, axis=2)
    mask = (distance >= float(color_distance_threshold)).astype(np.uint8) * 255
    border = max(6, min(height, width) // 60)
    mask[:border, :] = 0
    mask[-border:, :] = 0
    mask[:, :border] = 0
    mask[:, -border:] = 0
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8), iterations=2)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    count, _, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
    image_area = float(width * height)
    centre = np.asarray((width / 2.0, height / 2.0), dtype=np.float64)
    candidates = []
    for index in range(1, count):
        left, top, box_width, box_height, area = (
            int(value) for value in stats[index])
        if box_width <= 0 or box_height <= 0:
            continue
        area_fraction = float(area) / image_area
        if not (float(min_area_fraction) <= area_fraction
                <= float(max_area_fraction)):
            continue
        if (left <= border or top <= border
                or left + box_width >= width - border
                or top + box_height >= height - border):
            continue
        aspect = max(
            float(box_width) / float(box_height),
            float(box_height) / float(box_width),
        )
        if not (aspect_limits[0] <= aspect <= aspect_limits[1]):
            continue
        component_centre = np.asarray(centroids[index], dtype=np.float64)
        centre_distance = float(np.linalg.norm(component_centre - centre))
        # Prefer the component on the optical axis; area breaks close ties so
        # a small highlight cannot displace the complete object silhouette.
        candidates.append((centre_distance, -float(area), left, top,
                           box_width, box_height, area_fraction))
    if not candidates:
        return None
    _, _, left, top, box_width, box_height, area_fraction = min(candidates)
    quality = max(0.0, min(1.0, area_fraction / 0.15))
    return Detection(
        wanted,
        quality,
        (left, top, left + box_width, top + box_height),
        "geometry_foreground",
    )


def resolve_model_path(model_path: str, package_share: str | None = None) -> str:
    """Resolve a model path for both source and installed ROS workspaces.

    A relative path first works from the current directory, then from the
    installed ``image_pkg`` share directory.  This lets the launch file use
    ``models/best.pt`` without depending on the terminal's working directory.
    """
    path = Path(model_path).expanduser()
    if path.is_file():
        return str(path)
    if package_share:
        packaged_path = Path(package_share) / path
        if packaged_path.is_file():
            return str(packaged_path)
    raise FileNotFoundError(
        f"YOLO weight file not found: {model_path}. "
        "Expected image_pkg/models/best.pt after installation."
    )


class YoloDetector:
    """Run a fixed-class Ultralytics YOLO model trained for this project."""

    def __init__(self, model_path: str, device: str, package_share: str | None = None):
        try:
            import torch
            from ultralytics import YOLO
        except ImportError as exc:
            raise RuntimeError(
                "The trained detector requires ultralytics, torch, and numpy"
            ) from exc
        self.model = YOLO(resolve_model_path(model_path, package_share))
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

    def infer_tiled(
        self,
        image: np.ndarray,
        confidence: float,
        nms_iou: float,
        imgsz: int,
        tile_fraction: float = 0.65,
    ) -> list[Detection]:
        """Recover small objects from four overlapping high-resolution crops."""
        height, width = image.shape[:2]
        fraction = max(0.50, min(0.90, float(tile_fraction)))
        tile_width = max(32, min(width, int(round(width * fraction))))
        tile_height = max(32, min(height, int(round(height * fraction))))
        origins = {
            (0, 0),
            (max(0, width - tile_width), 0),
            (0, max(0, height - tile_height)),
            (max(0, width - tile_width), max(0, height - tile_height)),
        }
        detections = []
        for left, top in sorted(origins):
            crop = image[top:top + tile_height, left:left + tile_width]
            for detection in self.infer(
                    crop, confidence, nms_iou, imgsz):
                x1, y1, x2, y2 = detection.box
                detections.append(Detection(
                    detection.label,
                    detection.confidence,
                    (x1 + left, y1 + top, x2 + left, y2 + top),
                ))
        return deduplicate_detections(detections)

    def infer_center_region(
        self,
        image: np.ndarray,
        confidence: float,
        nms_iou: float,
        imgsz: int,
        region_fraction: float = 0.70,
    ) -> list[Detection]:
        """Run a high-resolution pass on the optical-axis image region.

        This is deliberately an inference crop, not a synthetic label or a
        truth-derived bounding box.  It gives small centred objects more input
        pixels while retaining their measured YOLO class and confidence.
        """
        height, width = image.shape[:2]
        fraction = max(0.40, min(1.0, float(region_fraction)))
        crop_width = max(32, min(width, int(round(width * fraction))))
        crop_height = max(32, min(height, int(round(height * fraction))))
        left = max(0, (width - crop_width) // 2)
        top = max(0, (height - crop_height) // 2)
        crop = image[top:top + crop_height, left:left + crop_width]
        values = []
        for detection in self.infer(crop, confidence, nms_iou, imgsz):
            x1, y1, x2, y2 = detection.box
            values.append(Detection(
                detection.label,
                detection.confidence,
                (x1 + left, y1 + top, x2 + left, y2 + top),
                "target_roi_recovery",
            ))
        return values


# Keep the import name compatible with older scripts.  It now loads normal
# fixed-class YOLO weights, not an open-vocabulary YOLO-World model.
YoloWorldDetector = YoloDetector
