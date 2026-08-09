"""Offline tests for the eight-class perception core."""
import ast
from pathlib import Path

import numpy as np
import pytest

from image_pkg.pcl_node import centroid_from_box
from image_pkg.yolo_world_detector import Detection
from image_pkg.yolo_world_detector import deduplicate_detections
from image_pkg.yolo_world_detector import resolve_model_path
from image_pkg.yolo_world_detector import select_best_candidate


def test_centroid_from_box_rejects_background_outlier():
    points = np.full((4, 4, 3), [0.1, 0.2, 0.8], dtype=np.float32)
    points[0, 0] = [4.0, 4.0, 4.0]

    centroid = centroid_from_box(
        points, width=4, height=4, x1=0, y1=0, x2=4, y2=4, min_points=4
    )

    np.testing.assert_allclose(centroid, [0.1, 0.2, 0.8], atol=1e-6)


def test_deduplicate_keeps_highest_confidence_box_per_label():
    detections = [
        Detection("aruco_sample", 0.6, (0, 0, 10, 10)),
        Detection("aruco_sample", 0.9, (1, 1, 11, 11)),
        Detection("aging_rack", 0.7, (1, 1, 11, 11)),
    ]

    kept = deduplicate_detections(detections, iou_threshold=0.5)

    assert [(item.label, item.confidence) for item in kept] == [
        ("aruco_sample", 0.9),
        ("aging_rack", 0.7),
    ]


def test_select_best_candidate_honors_target_label():
    candidates = [
        {"label": "aging_rack", "confidence": 0.9},
        {"label": "aruco_sample", "confidence": 0.6},
    ]

    assert select_best_candidate(candidates, "aruco_sample") == candidates[1]
    assert select_best_candidate(candidates, "missing") is None


def test_model_path_resolves_external_weight_and_never_downloads(tmp_path):
    model = tmp_path / "eight_class.pt"
    model.write_bytes(b"offline-test-weight")

    assert resolve_model_path(str(model)) == str(model)
    with pytest.raises(FileNotFoundError):
        resolve_model_path(str(tmp_path / "missing.pt"))


def test_detector_module_has_no_top_level_dl_imports():
    module_path = Path(__file__).resolve().parents[1] / "image_pkg"
    source = (module_path / "yolo_world_detector.py").read_text(encoding="utf-8")
    tree = ast.parse(source)
    imported = {
        alias.name.split(".")[0]
        for statement in tree.body
        if isinstance(statement, ast.Import)
        for alias in statement.names
    }
    imported.update(
        statement.module.split(".")[0]
        for statement in tree.body
        if isinstance(statement, ast.ImportFrom) and statement.module
    )

    assert imported.isdisjoint({"torch", "ultralytics"})
