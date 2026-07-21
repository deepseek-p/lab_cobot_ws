"""
Generate a synthetic PGM map covering the full 2x lab environment (14x14m).

The map is rendered from the known world geometry: exterior walls, four
worktables, and the high-voltage-zone fence.  Interior space is free (254),
obstacles are occupied (0).
"""
from __future__ import annotations

from pathlib import Path

MAP_RESOLUTION = 0.05          # m / pixel
MAP_MARGIN = 0.5               # margin outside walls (m)
WORLD_HALF = 7.0               # walls at x/y = +/-7.0
WALL_THICKNESS = 0.1           # m

# bounding box
LO = -WORLD_HALF - MAP_MARGIN
HI = WORLD_HALF + MAP_MARGIN
SPAN = HI - LO                       # 15.0 m
PX = int(SPAN / MAP_RESOLUTION)      # 300 px


def _world_to_pixel(wx: float, wy: float) -> tuple[int, int]:
    """Convert world (x,y) to PGM (col,row). Row 0 = top (max y)."""
    col = int((wx - LO) / MAP_RESOLUTION)
    row = int((HI - wy) / MAP_RESOLUTION)
    return col, row


def _fill_rect(grid: list[list[int]], cx: float, cy: float,
               sx: float, sy: float) -> None:
    """Fill a world-aligned rectangle centred at (cx,cy) with size (sx,sy)."""
    x0, y0 = _world_to_pixel(cx - sx / 2.0, cy + sy / 2.0)
    x1, y1 = _world_to_pixel(cx + sx / 2.0, cy - sy / 2.0)
    x0 = max(0, x0)
    y0 = max(0, y0)
    x1 = min(PX - 1, x1)
    y1 = min(PX - 1, y1)
    for row in range(y0, y1 + 1):
        for col in range(x0, x1 + 1):
            grid[row][col] = 0


def main() -> None:
    # all free (254) initially
    grid = [[254 for _ in range(PX)] for _ in range(PX)]

    # ---- exterior walls --------------------------------------------------
    # North wall (y = +7)
    _fill_rect(grid, 0.0, WORLD_HALF, 2.0 * WORLD_HALF, WALL_THICKNESS)
    # South wall (y = -7)
    _fill_rect(grid, 0.0, -WORLD_HALF, 2.0 * WORLD_HALF, WALL_THICKNESS)
    # East wall (x = +7)
    _fill_rect(grid, WORLD_HALF, 0.0, WALL_THICKNESS, 2.0 * WORLD_HALF)
    # West wall (x = -7)
    _fill_rect(grid, -WORLD_HALF, 0.0, WALL_THICKNESS, 2.0 * WORLD_HALF)

    # ---- worktables (1.6 x 1.2 m) ---------------------------------------
    TABLES = [
        (-4.30, 3.80),     # station_a_table
        (-4.10, -2.30),    # tooling_zone_table
        (0.20, 4.20),      # aging_zone_table
        (0.30, -1.70),     # station_b_table
    ]
    for cx, cy in TABLES:
        _fill_rect(grid, cx, cy, 1.6, 1.2)

    # ---- high-voltage zone fence -----------------------------------------
    # Collision fence: front/back walls x=[3.36,5.36], left/right y=[2.06,3.74]
    hv_cx, hv_cy = 4.36, 2.90
    hv_hx, hv_hy = 1.00, 0.84   # half-extents of the fence rectangle
    # The fence is thin (0.02m), but we draw a solid block to block planning
    _fill_rect(grid, hv_cx, hv_cy, 2.0 * hv_hx, 2.0 * hv_hy)

    # ---- write PGM -------------------------------------------------------
    pgm_path = Path(__file__).resolve().parents[1] / "maps" / "map.pgm"
    with open(pgm_path, "wb") as f:
        header = f"P5\n# Synthetic 2x lab ({PX}x{PX})\n{PX} {PX}\n255\n"
        f.write(header.encode("ascii"))
        for row in grid:
            f.write(bytes(row))

    yaml_path = Path(__file__).resolve().parents[1] / "maps" / "map.yaml"
    yaml_content = (
        f"image: map.pgm\n"
        f"mode: trinary\n"
        f"resolution: {MAP_RESOLUTION}\n"
        f"origin:\n"
        f"- {LO:.2f}\n"
        f"- {LO:.2f}\n"
        f"- 0\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.25\n"
    )
    with open(yaml_path, "w", encoding="ascii") as f:
        f.write(yaml_content)

    occ = sum(1 for row in grid for v in row if v == 0)
    free = PX * PX - occ
    print(
        f"Wrote {pgm_path}  ({PX}×{PX}, {free} free px, {occ} occupied px)"
    )
    print(f"Wrote {yaml_path}")
    print(f"Map covers world x/y ∈ [{LO:.1f}, {HI:.1f}], origin=({LO:.1f},{LO:.1f})")


if __name__ == "__main__":
    main()
