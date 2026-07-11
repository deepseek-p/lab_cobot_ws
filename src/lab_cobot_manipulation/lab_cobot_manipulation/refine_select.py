"""Select a safe refined object position."""
import math


REFINE_MAX_CORRECTION_M = 0.05


def select_refined_position(
    coarse_xyz: list[float],
    refined_xyz: list[float] | None,
    max_correction_m: float = REFINE_MAX_CORRECTION_M,
) -> tuple[list[float], bool, str]:
    """Select a refined position when its correction is within the safety gate."""
    coarse = [float(value) for value in coarse_xyz]
    if refined_xyz is None:
        return coarse, False, "no_fresh_sample"

    refined = [float(value) for value in refined_xyz]
    if math.dist(coarse, refined) > max_correction_m:
        return coarse, False, "correction_exceeds_gate"
    return refined, True, "ok"
