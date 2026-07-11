"""Point cloud operation tests."""
import ast
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lab_cobot_perception import pointcloud_ops  # noqa: E402


def test_module_top_level_imports_only_numpy():
    source = Path(pointcloud_ops.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add((node.module or "").split(".")[0])

    assert imports <= {"numpy"}


def test_depth_to_points_filters_inf_and_range():
    depth = np.array(
        [
            [np.inf, 0.3, 0.5],
            [0.0, 1.0, 1.5],
            [np.nan, 0.8, 2.0],
        ],
        dtype=np.float32,
    )

    points = pointcloud_ops.depth_to_points(
        depth,
        fx=2.0,
        fy=4.0,
        cx=1.0,
        cy=1.0,
        z_min=0.4,
        z_max=1.4,
    )

    np.testing.assert_allclose(
        points,
        np.array([
            [0.25, -0.125, 0.5],
            [0.0, 0.0, 1.0],
            [0.0, 0.2, 0.8],
        ]),
        atol=1e-7,
    )


def _cube_points(center, side=0.07, samples=15):
    offsets = np.linspace(-side / 2.0, side / 2.0, samples)
    grid = np.array(np.meshgrid(offsets, offsets, offsets), dtype=np.float64)
    return grid.reshape(3, -1).T + np.asarray(center, dtype=np.float64)


def test_segment_objects_finds_two_cubes_on_plane():
    xs = np.linspace(-0.45, 0.45, 40)
    ys = np.linspace(-0.28, 0.28, 28)
    plane = np.array([[x, y, 0.8] for x in xs for y in ys], dtype=np.float64)
    cube_a = _cube_points((-0.22, -0.04, 0.835))
    cube_b = _cube_points((0.24, 0.08, 0.835))
    points = np.vstack([plane, cube_a, cube_b])

    clusters = pointcloud_ops.segment_objects(
        points,
        voxel_size=0.004,
        plane_dist=0.003,
        eps=0.04,
        min_points=20,
    )

    assert len(clusters) == 2
    clusters = sorted(clusters, key=lambda cluster: cluster["centroid"][0])
    assert clusters[0]["centroid"] == pytest.approx((-0.22, -0.04, 0.835), abs=0.005)
    assert clusters[1]["centroid"] == pytest.approx((0.24, 0.08, 0.835), abs=0.005)
    for cluster in clusters:
        assert cluster["extent"] == pytest.approx((0.07, 0.07, 0.07), abs=0.012)
        assert cluster["n_points"] >= 20


def test_associate_assigns_class_by_reprojection():
    clusters = [
        {"centroid": np.array([0.1, 0.0, 1.0])},
        {"centroid": np.array([-0.5, 0.0, 1.0])},
    ]
    dets = [
        {"xyxy": [105.0, 95.0, 115.0, 105.0], "conf": 0.5},
    ]

    matches = pointcloud_ops.associate(
        clusters,
        dets,
        fx=100.0,
        fy=100.0,
        cx=100.0,
        cy=100.0,
    )

    assert matches == [0, None]


def test_associate_uses_highest_confidence_when_boxes_overlap():
    clusters = [{"centroid": np.array([0.0, 0.0, 1.0])}]
    dets = [
        {"xyxy": [90.0, 90.0, 110.0, 110.0], "conf": 0.2},
        {"xyxy": [95.0, 95.0, 105.0, 105.0], "conf": 0.8},
    ]

    assert pointcloud_ops.associate(
        clusters,
        dets,
        fx=100.0,
        fy=100.0,
        cx=100.0,
        cy=100.0,
    ) == [1]


def test_match_aruco_within_gate():
    clusters = [
        {"centroid": np.array([0.0, 0.0, 1.0])},
        {"centroid": np.array([0.3, 0.0, 1.0])},
    ]

    assert pointcloud_ops.match_aruco(
        clusters,
        aruco_xyz=np.array([0.05, 0.0, 1.0]),
        gate_m=0.06,
    ) == 0
    assert pointcloud_ops.match_aruco(
        clusters,
        aruco_xyz=np.array([0.10, 0.0, 1.0]),
        gate_m=0.06,
    ) is None
