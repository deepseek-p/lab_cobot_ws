#!/usr/bin/env python3
"""Generate Manipulation validation figures from experiment CSV files."""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


def read_csv(path: Path) -> list[dict]:
    if not path.exists():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def as_float(value, default=0.0) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return default
    if not math.isfinite(result):
        return default
    return result


def success_rate_by_target(rows: list[dict]) -> dict[str, tuple[int, int]]:
    counts: dict[str, list[int]] = defaultdict(lambda: [0, 0])
    for row in rows:
        target = row.get("target", "unknown")
        counts[target][1] += 1
        if int(as_float(row.get("holding_success"))) and int(as_float(row.get("lift_success"))):
            counts[target][0] += 1
    return {target: (values[0], values[1]) for target, values in counts.items()}


def plot_grasp_success(rows: list[dict], output_dir: Path) -> Path | None:
    rates = success_rate_by_target(rows)
    if not rates:
        return None
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = list(rates)
    values = [100.0 * ok / total if total else 0.0 for ok, total in rates.values()]
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    ax.bar(labels, values, color="#3578b8")
    ax.axhline(90.0, color="#b83b3b", linestyle="--", linewidth=1.2, label="90% target")
    ax.set_ylabel("success rate (%)")
    ax.set_ylim(0, 105)
    ax.tick_params(axis="x", rotation=20)
    ax.legend()
    path = output_dir / "grasp_success_rate.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_stage_times(rows: list[dict], output_dir: Path) -> Path | None:
    stages = [
        "stage_planning_sec",
        "stage_descend_sec",
        "stage_contact_sec",
        "stage_attach_sec",
        "stage_lift_sec",
    ]
    means = []
    labels = []
    for stage in stages:
        values = [as_float(row.get(stage), math.nan) for row in rows]
        values = [value for value in values if math.isfinite(value) and value > 0.0]
        if values:
            labels.append(stage.replace("stage_", "").replace("_sec", ""))
            means.append(statistics.fmean(values))
    if not means:
        return None
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    ax.bar(labels, means, color="#5f8f4f")
    ax.set_ylabel("mean duration (s)")
    path = output_dir / "grasp_stage_time.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_repeatability(rows: list[dict], output_dir: Path) -> list[Path]:
    ok_rows = [row for row in rows if int(as_float(row.get("move_success")))]
    if not ok_rows:
        return []
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    errors = [as_float(row.get("repeatability_error_mm")) for row in ok_rows]
    fig, ax = plt.subplots(figsize=(8, 4.8), constrained_layout=True)
    ax.hist(errors, bins=min(12, max(4, len(errors) // 2)), color="#6c6c9f", edgecolor="white")
    ax.axvline(0.05, color="#b83b3b", linestyle="--", linewidth=1.2, label="0.05 mm")
    ax.set_xlabel("repeatability error (mm)")
    ax.set_ylabel("count")
    ax.legend()
    error_path = output_dir / "repeatability_error.png"
    fig.savefig(error_path, dpi=180)
    plt.close(fig)

    indices = [int(row["trial_id"]) for row in ok_rows]
    dx = [as_float(row.get("dx_mm")) for row in ok_rows]
    dy = [as_float(row.get("dy_mm")) for row in ok_rows]
    dz = [as_float(row.get("dz_mm")) for row in ok_rows]
    fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    ax.plot(indices, dx, marker="o", label="X")
    ax.plot(indices, dy, marker="s", label="Y")
    ax.plot(indices, dz, marker="^", label="Z")
    ax.axhline(0.05, color="#b83b3b", linestyle="--", linewidth=1.0)
    ax.axhline(-0.05, color="#b83b3b", linestyle="--", linewidth=1.0)
    ax.set_xlabel("trial")
    ax.set_ylabel("axis deviation (mm)")
    ax.legend()
    xyz_path = output_dir / "xyz_repeatability.png"
    fig.savefig(xyz_path, dpi=180)
    plt.close(fig)
    return [error_path, xyz_path]


def plot_force(rows: list[dict], output_dir: Path) -> Path | None:
    if not rows:
        return None
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    elapsed = [as_float(row.get("elapsed_sec")) for row in rows]
    left = [as_float(row.get("left_force_n")) for row in rows]
    right = [as_float(row.get("right_force_n")) for row in rows]
    fig, ax = plt.subplots(figsize=(9, 4.8), constrained_layout=True)
    ax.plot(elapsed, left, label="left")
    ax.plot(elapsed, right, label="right")
    ax.set_xlabel("elapsed (s)")
    ax.set_ylabel("force (N)")
    ax.legend()
    path = output_dir / "force_time_series.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_summary(output_dir: Path, grasp_rows: list[dict], repeat_rows: list[dict], figures: list[Path]) -> Path:
    lines = ["# Manipulation Validation Experiment Summary", ""]
    rates = success_rate_by_target(grasp_rows)
    for target, (ok, total) in rates.items():
        lines.append(f"- {target}: {ok}/{total} = {100.0 * ok / total:.2f}%")
    ok_repeat = [row for row in repeat_rows if int(as_float(row.get("move_success")))]
    if ok_repeat:
        errors = [as_float(row.get("repeatability_error_mm")) for row in ok_repeat]
        rms = math.sqrt(statistics.fmean(value * value for value in errors))
        lines.append(f"- repeatability mean error: {statistics.fmean(errors):.6f} mm")
        lines.append(f"- repeatability RMS error: {rms:.6f} mm")
        lines.append(f"- repeatability max error: {max(errors):.6f} mm")
        lines.append("- measurement source: MoveIt FK from current joint_state")
    if figures:
        lines.append("")
        lines.append("## Figures")
        for figure in figures:
            lines.append(f"- {figure}")
    path = output_dir / "manipulation_validation_experiment_summary.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default="results/manipulation_validation")
    args = parser.parse_args()
    output_dir = Path(args.results_dir).expanduser()
    output_dir.mkdir(parents=True, exist_ok=True)
    grasp_rows = read_csv(output_dir / "grasp_trials.csv")
    repeat_rows = read_csv(output_dir / "repeatability.csv")
    force_rows = read_csv(output_dir / "contact_force.csv")
    figures = []
    for figure in (
        plot_grasp_success(grasp_rows, output_dir),
        plot_stage_times(grasp_rows, output_dir),
        *plot_repeatability(repeat_rows, output_dir),
        plot_force(force_rows, output_dir),
    ):
        if figure is not None:
            figures.append(figure)
    summary = write_summary(output_dir, grasp_rows, repeat_rows, figures)
    print(summary)


if __name__ == "__main__":
    main()
