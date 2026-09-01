#!/usr/bin/env python3
"""Analyze Manipulation validation force-control raw force collection."""
from __future__ import annotations

import argparse
import csv
import math
import statistics
from collections import defaultdict
from pathlib import Path


TARGETS = (
    "high_voltage_probe_kit",
    "tooling_fixture_box",
    "material_spare_igbt",
)


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


def moving_average(values: list[float], window: int) -> list[float]:
    if window <= 1:
        return list(values)
    result = []
    for index in range(len(values)):
        start = max(0, index - window + 1)
        result.append(statistics.fmean(values[start : index + 1]))
    return result


def grouped_by_trial(rows: list[dict]) -> dict[str, list[dict]]:
    grouped: dict[str, list[dict]] = defaultdict(list)
    for row in rows:
        grouped[str(row.get("trial", ""))].append(row)
    return dict(grouped)


def first_contact_time(rows: list[dict]) -> float | None:
    for row in rows:
        if row.get("left_contact") == "1" or row.get("right_contact") == "1":
            return as_float(row.get("elapsed_sec"))
    return None


def first_dual_contact_time(rows: list[dict]) -> float | None:
    for row in rows:
        if row.get("left_contact") == "1" and row.get("right_contact") == "1":
            return as_float(row.get("elapsed_sec"))
    return None


def peak(values: list[float]) -> float:
    return max(values) if values else 0.0


def precontact_noise(rows: list[dict], column: str, contact_time: float | None) -> float:
    if contact_time is None:
        values = [as_float(row.get(column)) for row in rows]
    else:
        values = [
            as_float(row.get(column))
            for row in rows
            if as_float(row.get("elapsed_sec")) < contact_time
        ]
    if len(values) < 2:
        return 0.0
    return statistics.pstdev(values)


def filter_summary_for_trial(target: str, trial: str, rows: list[dict]) -> dict:
    left = [as_float(row.get("left_force_raw")) for row in rows]
    right = [as_float(row.get("right_force_raw")) for row in rows]
    contact_time = first_contact_time(rows)
    dual_contact_time = first_dual_contact_time(rows)
    sources = {row.get("force_source", "INVALID") for row in rows}
    raw_peak_left = peak(left)
    raw_peak_right = peak(right)
    left_3 = moving_average(left, 3)
    right_3 = moving_average(right, 3)
    left_5 = moving_average(left, 5)
    right_5 = moving_average(right, 5)
    return {
        "target": target,
        "trial": trial,
        "samples": len(rows),
        "force_source": "|".join(sorted(sources)),
        "first_contact_sec": "" if contact_time is None else f"{contact_time:.6f}",
        "first_dual_contact_sec": "" if dual_contact_time is None else f"{dual_contact_time:.6f}",
        "raw_peak_left": f"{raw_peak_left:.6f}",
        "raw_peak_right": f"{raw_peak_right:.6f}",
        "ma3_peak_left": f"{peak(left_3):.6f}",
        "ma3_peak_right": f"{peak(right_3):.6f}",
        "ma5_peak_left": f"{peak(left_5):.6f}",
        "ma5_peak_right": f"{peak(right_5):.6f}",
        "precontact_noise_left": f"{precontact_noise(rows, 'left_force_raw', contact_time):.6f}",
        "precontact_noise_right": f"{precontact_noise(rows, 'right_force_raw', contact_time):.6f}",
        "raw_balance_peak": f"{max((abs(l - r) for l, r in zip(left, right)), default=0.0):.6f}",
    }


def write_filter_summary(root: Path, rows_by_target: dict[str, list[dict]]) -> list[dict]:
    summaries = []
    for target, rows in rows_by_target.items():
        for trial, trial_rows in sorted(grouped_by_trial(rows).items()):
            summaries.append(filter_summary_for_trial(target, trial, trial_rows))
    path = root / "force_filter_summary.csv"
    with path.open("w", newline="", encoding="utf-8") as handle:
        fieldnames = [
            "target",
            "trial",
            "samples",
            "force_source",
            "first_contact_sec",
            "first_dual_contact_sec",
            "raw_peak_left",
            "raw_peak_right",
            "ma3_peak_left",
            "ma3_peak_right",
            "ma5_peak_left",
            "ma5_peak_right",
            "precontact_noise_left",
            "precontact_noise_right",
            "raw_balance_peak",
        ]
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(summaries)
    return summaries


def plot_target_force(target: str, rows: list[dict], output_dir: Path) -> Path | None:
    if not rows:
        return None
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
    for trial, trial_rows in sorted(grouped_by_trial(rows).items()):
        elapsed = [as_float(row.get("elapsed_sec")) for row in trial_rows]
        left = [as_float(row.get("left_force_raw")) for row in trial_rows]
        right = [as_float(row.get("right_force_raw")) for row in trial_rows]
        axes[0].plot(elapsed, left, alpha=0.35, linewidth=0.8, label=f"L raw T{trial}")
        axes[0].plot(elapsed, right, alpha=0.35, linewidth=0.8, linestyle="--", label=f"R raw T{trial}")
        axes[1].plot(elapsed, moving_average(left, 3), linewidth=1.0, label=f"L ma3 T{trial}")
        axes[1].plot(elapsed, moving_average(right, 3), linewidth=1.0, linestyle="--", label=f"R ma3 T{trial}")
        gap = [as_float(row.get("gripper_gap_mm"), math.nan) for row in trial_rows]
        axes[2].plot(elapsed, gap, linewidth=1.0, label=f"T{trial}")
    axes[0].set_ylabel("raw force (N)")
    axes[1].set_ylabel("3-frame force (N)")
    axes[2].set_ylabel("gripper gap (mm)")
    axes[2].set_xlabel("trial elapsed (s)")
    axes[0].set_title(target)
    for axis in axes:
        axis.grid(True, alpha=0.25)
        axis.legend(fontsize=7, ncol=3)
    path = output_dir / f"{target}_force_time_series.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def plot_filter_comparison(summaries: list[dict], output_dir: Path) -> Path | None:
    if not summaries:
        return None
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    labels = [f"{row['target']}#{row['trial']}" for row in summaries]
    raw = [max(as_float(row["raw_peak_left"]), as_float(row["raw_peak_right"])) for row in summaries]
    ma3 = [max(as_float(row["ma3_peak_left"]), as_float(row["ma3_peak_right"])) for row in summaries]
    ma5 = [max(as_float(row["ma5_peak_left"]), as_float(row["ma5_peak_right"])) for row in summaries]
    x = list(range(len(labels)))
    fig, ax = plt.subplots(figsize=(11, 4.8), constrained_layout=True)
    width = 0.26
    ax.bar([value - width for value in x], raw, width=width, label="raw")
    ax.bar(x, ma3, width=width, label="3-frame")
    ax.bar([value + width for value in x], ma5, width=width, label="5-frame")
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=30, ha="right", fontsize=8)
    ax.set_ylabel("peak force (N)")
    ax.legend()
    ax.grid(True, axis="y", alpha=0.25)
    path = output_dir / "force_filter_comparison.png"
    fig.savefig(path, dpi=180)
    plt.close(fig)
    return path


def write_markdown_summary(root: Path, summaries: list[dict], figures: list[Path]) -> Path:
    lines = ["# Manipulation Validation Force-Control Raw Signal Summary", ""]
    by_target: dict[str, list[dict]] = defaultdict(list)
    for row in summaries:
        by_target[row["target"]].append(row)
    for target in TARGETS:
        rows = by_target.get(target, [])
        if not rows:
            lines.append(f"- {target}: no data")
            continue
        sources = sorted({row["force_source"] for row in rows})
        raw_peaks = [
            max(as_float(row["raw_peak_left"]), as_float(row["raw_peak_right"]))
            for row in rows
        ]
        ma3_peaks = [
            max(as_float(row["ma3_peak_left"]), as_float(row["ma3_peak_right"]))
            for row in rows
        ]
        ma5_peaks = [
            max(as_float(row["ma5_peak_left"]), as_float(row["ma5_peak_right"]))
            for row in rows
        ]
        noise = [
            max(as_float(row["precontact_noise_left"]), as_float(row["precontact_noise_right"]))
            for row in rows
        ]
        lines.append(
            "- %s: trials=%d source=%s raw_peak_mean=%.3fN ma3_peak_mean=%.3fN "
            "ma5_peak_mean=%.3fN precontact_noise_max=%.3fN"
            % (
                target,
                len(rows),
                ",".join(sources),
                statistics.fmean(raw_peaks),
                statistics.fmean(ma3_peaks),
                statistics.fmean(ma5_peaks),
                max(noise) if noise else 0.0,
            )
        )
    if figures:
        lines.extend(["", "## Figures"])
        lines.extend(f"- {figure}" for figure in figures)
    path = root / "force_summary.md"
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return path


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--results-dir", default="results/manipulation_validation/force_control")
    args = parser.parse_args()
    root = Path(args.results_dir).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    rows_by_target = {}
    for target in TARGETS:
        rows_by_target[target] = read_csv(root / target / "raw_n3" / "contact_force_timeseries.csv")
    summaries = write_filter_summary(root, rows_by_target)
    figures = []
    for target, rows in rows_by_target.items():
        figure = plot_target_force(target, rows, root)
        if figure is not None:
            figures.append(figure)
    comparison = plot_filter_comparison(summaries, root)
    if comparison is not None:
        figures.append(comparison)
    summary = write_markdown_summary(root, summaries, figures)
    print(summary)


if __name__ == "__main__":
    main()
