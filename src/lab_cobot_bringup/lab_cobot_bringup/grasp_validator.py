"""Pure geometry checks for simulated parallel-gripper attach requests."""
from __future__ import annotations

import math
from dataclasses import dataclass


ACCEPTED = "accepted"
TCP_TOO_FAR = "tcp_too_far"
OBJECT_OUTSIDE_PALM_WIDTH = "object_outside_palm_width"
OBJECT_OUTSIDE_FINGER_GAP = "object_outside_finger_gap"
OBJECT_OUTSIDE_GRASP_DEPTH = "object_outside_grasp_depth"


@dataclass(frozen=True)
class GraspValidationConfig:
    max_center_distance_m: float = 0.080
    max_abs_x_m: float = 0.040
    max_abs_y_m: float = 0.018
    min_z_m: float = -0.060
    max_z_m: float = 0.025


@dataclass(frozen=True)
class GraspValidationResult:
    accepted: bool
    reason: str
    offset_tcp: tuple[float, float, float]
    center_distance_m: float


def validate_tcp_object_grasp(
    offset_tcp: tuple[float, float, float],
    config: GraspValidationConfig | None = None,
) -> GraspValidationResult:
    """Return whether an object center is inside the TCP-frame grasp envelope."""
    cfg = config or GraspValidationConfig()
    x, y, z = (float(offset_tcp[0]), float(offset_tcp[1]), float(offset_tcp[2]))
    distance = math.sqrt(x * x + y * y + z * z)

    if distance > cfg.max_center_distance_m:
        return GraspValidationResult(False, TCP_TOO_FAR, (x, y, z), distance)
    if abs(x) > cfg.max_abs_x_m:
        return GraspValidationResult(
            False,
            OBJECT_OUTSIDE_PALM_WIDTH,
            (x, y, z),
            distance,
        )
    if abs(y) > cfg.max_abs_y_m:
        return GraspValidationResult(
            False,
            OBJECT_OUTSIDE_FINGER_GAP,
            (x, y, z),
            distance,
        )
    if z < cfg.min_z_m or z > cfg.max_z_m:
        return GraspValidationResult(
            False,
            OBJECT_OUTSIDE_GRASP_DEPTH,
            (x, y, z),
            distance,
        )
    return GraspValidationResult(True, ACCEPTED, (x, y, z), distance)
