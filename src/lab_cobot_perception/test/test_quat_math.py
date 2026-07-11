"""Quaternion math helper tests."""
import ast
from pathlib import Path
import math
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from lab_cobot_perception import quat_math  # noqa: E402


def test_module_top_level_has_no_dl_heavy_imports():
    source = Path(quat_math.__file__).read_text(encoding="utf-8")
    tree = ast.parse(source)
    imports = set()
    for node in tree.body:
        if isinstance(node, ast.Import):
            imports.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom):
            imports.add((node.module or "").split(".")[0])

    assert imports.isdisjoint({"ultralytics", "open3d", "torch"})


def test_identity_rotation_round_trips_point():
    point = (0.2, -0.1, 1.5)

    rotated = quat_math.quat_rotate((0.0, 0.0, 0.0, 1.0), point)

    assert rotated == pytest.approx(point)


def test_quaternion_rotation_and_inverse_round_trip():
    angle = math.pi / 2.0
    q = (0.0, 0.0, math.sin(angle / 2.0), math.cos(angle / 2.0))
    point = (0.4, 0.0, 1.2)

    rotated = quat_math.quat_rotate(q, point)
    restored = quat_math.quat_rotate(quat_math.quat_conjugate(q), rotated)

    assert rotated == pytest.approx((0.0, 0.4, 1.2), abs=1e-9)
    assert restored == pytest.approx(point, abs=1e-9)


def test_normalize_zero_vector_falls_back_to_identity():
    assert quat_math.quat_normalize((0.0, 0.0, 0.0, 0.0)) == (
        0.0,
        0.0,
        0.0,
        1.0,
    )
