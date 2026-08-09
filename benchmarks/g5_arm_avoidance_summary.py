"""Summarize G5 arm dynamic obstacle avoidance measurement CSV files."""
import argparse
import csv
import math
import statistics
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path


NUMERIC_FIELDS = (
    "apply_scene_latency_ms",
    "apply_scene_service_wait_ms",
    "apply_scene_call_latency_ms",
    "planning_latency_ms",
    "total_latency_ms",
    "control_latency_ms",
)


@dataclass(frozen=True)
class SummaryStats:
    count: int
    ok_count: int
    failed_count: int
    expected_count: int
    expected_met_count: int
    plan_ok_rate: float
    expected_met_rate: float
    apply_scene_mean_ms: float
    apply_scene_std_ms: float
    apply_scene_p95_ms: float
    apply_scene_max_ms: float
    apply_scene_service_wait_mean_ms: float
    apply_scene_call_mean_ms: float
    planning_mean_ms: float
    planning_std_ms: float
    planning_p95_ms: float
    planning_max_ms: float
    total_mean_ms: float
    total_std_ms: float
    total_p95_ms: float
    total_max_ms: float
    total_over_200ms: int
    total_within_200ms: int
    total_within_200ms_rate: float
    control_mean_ms: float
    control_std_ms: float
    control_p95_ms: float
    control_max_ms: float
    control_over_200ms: int
    control_within_200ms: int
    control_within_200ms_rate: float


def _read_measurement_csv(path: Path) -> dict:
    with path.open("r", encoding="utf-8", newline="") as csv_file:
        rows = list(csv.DictReader(csv_file))
    if len(rows) != 1:
        raise ValueError(f"{path} must contain exactly one measurement row")
    row = rows[0]
    for field in NUMERIC_FIELDS:
        if field in row and row[field] not in (None, ""):
            row[field] = float(row[field])
        else:
            row[field] = 0.0
    if row["control_latency_ms"] == 0.0 and row["total_latency_ms"] > 0.0:
        row["control_latency_ms"] = max(
            0.0,
            row["total_latency_ms"] - row["apply_scene_service_wait_ms"],
        )
    row["planning_ok"] = row["planning_ok"] == "True"
    row["apply_scene_ok"] = row["apply_scene_ok"] == "True"
    row["case_label"] = row.get("case_label", "unspecified")
    row["expected_verdict"] = row.get("expected_verdict", "ANY")
    row["expected_met"] = row.get("expected_met", "True") == "True"
    row["trajectory_points"] = int(row["trajectory_points"])
    row["moveit_error_code"] = int(row["moveit_error_code"])
    return row


def read_measurements(paths: list[Path]) -> list[dict]:
    measurements = []
    for path in paths:
        measurements.append(_read_measurement_csv(path))
    return measurements


def _p95(values: list[float]) -> float:
    ordered = sorted(values)
    index = max(0, math.ceil(len(ordered) * 0.95) - 1)
    return ordered[index]


def _std(values: list[float]) -> float:
    return statistics.pstdev(values) if len(values) > 1 else 0.0


def summarize(measurements: list[dict]) -> SummaryStats:
    if not measurements:
        raise ValueError("no measurements")

    apply_values = [row["apply_scene_latency_ms"] for row in measurements]
    service_wait_values = [
        row["apply_scene_service_wait_ms"] for row in measurements
    ]
    apply_call_values = [
        row["apply_scene_call_latency_ms"] for row in measurements
    ]
    planning_values = [row["planning_latency_ms"] for row in measurements]
    total_values = [row["total_latency_ms"] for row in measurements]
    control_values = [row["control_latency_ms"] for row in measurements]
    ok_count = sum(1 for row in measurements if row["planning_ok"])
    failed_count = len(measurements) - ok_count
    expected_rows = [
        row for row in measurements if row.get("expected_verdict", "ANY") != "ANY"
    ]
    expected_met_count = sum(1 for row in expected_rows if row["expected_met"])
    return SummaryStats(
        count=len(measurements),
        ok_count=ok_count,
        failed_count=failed_count,
        expected_count=len(expected_rows),
        expected_met_count=expected_met_count,
        plan_ok_rate=ok_count / len(measurements),
        expected_met_rate=(
            expected_met_count / len(expected_rows)
            if expected_rows
            else 0.0
        ),
        apply_scene_mean_ms=statistics.fmean(apply_values),
        apply_scene_std_ms=_std(apply_values),
        apply_scene_p95_ms=_p95(apply_values),
        apply_scene_max_ms=max(apply_values),
        apply_scene_service_wait_mean_ms=statistics.fmean(service_wait_values),
        apply_scene_call_mean_ms=statistics.fmean(apply_call_values),
        planning_mean_ms=statistics.fmean(planning_values),
        planning_std_ms=_std(planning_values),
        planning_p95_ms=_p95(planning_values),
        planning_max_ms=max(planning_values),
        total_mean_ms=statistics.fmean(total_values),
        total_std_ms=_std(total_values),
        total_p95_ms=_p95(total_values),
        total_max_ms=max(total_values),
        total_over_200ms=sum(1 for value in total_values if value > 200.0),
        total_within_200ms=sum(1 for value in total_values if value <= 200.0),
        total_within_200ms_rate=(
            sum(1 for value in total_values if value <= 200.0) / len(total_values)
        ),
        control_mean_ms=statistics.fmean(control_values),
        control_std_ms=_std(control_values),
        control_p95_ms=_p95(control_values),
        control_max_ms=max(control_values),
        control_over_200ms=sum(1 for value in control_values if value > 200.0),
        control_within_200ms=sum(1 for value in control_values if value <= 200.0),
        control_within_200ms_rate=(
            sum(1 for value in control_values if value <= 200.0)
            / len(control_values)
        ),
    )


def write_summary(
    measurements: list[dict],
    stats: SummaryStats,
    out_dir: Path,
) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    md_path = out_dir / f"g5_arm_avoidance_summary_{stamp}.md"

    lines = [
        f"# G5 机械臂动态避障统计报告 {stamp}",
        f"- 样本数: {stats.count}",
        f"- PLAN_OK: {stats.ok_count}",
        f"- PLAN_FAILED: {stats.failed_count}",
        f"- 规划成功率: {stats.plan_ok_rate * 100.0:.1f}%",
        f"- 有预期场景数: {stats.expected_count}",
        f"- 预期满足数: {stats.expected_met_count}",
        f"- 预期满足率: {stats.expected_met_rate * 100.0:.1f}%",
        f"- PlanningScene 注入延时: mean {stats.apply_scene_mean_ms:.1f} ms | "
        f"std {stats.apply_scene_std_ms:.1f} ms | "
        f"P95 {stats.apply_scene_p95_ms:.1f} ms | "
        f"max {stats.apply_scene_max_ms:.1f} ms",
        f"- PlanningScene 服务发现均值: "
        f"{stats.apply_scene_service_wait_mean_ms:.1f} ms",
        f"- PlanningScene apply 调用均值: "
        f"{stats.apply_scene_call_mean_ms:.1f} ms",
        f"- MoveIt 规划延时: mean {stats.planning_mean_ms:.1f} ms | "
        f"std {stats.planning_std_ms:.1f} ms | "
        f"P95 {stats.planning_p95_ms:.1f} ms | "
        f"max {stats.planning_max_ms:.1f} ms",
        f"- 总响应延时: mean {stats.total_mean_ms:.1f} ms | "
        f"std {stats.total_std_ms:.1f} ms | "
        f"P95 {stats.total_p95_ms:.1f} ms | "
        f"max {stats.total_max_ms:.1f} ms",
        f"- 总响应延时 >200ms 样本数: {stats.total_over_200ms}",
        f"- 总响应延时 <=200ms 样本数: {stats.total_within_200ms}",
        f"- 总响应延时 <=200ms 达标率: {stats.total_within_200ms_rate * 100.0:.1f}%",
        f"- 服务已连接后的控制响应延时: mean {stats.control_mean_ms:.1f} ms | "
        f"std {stats.control_std_ms:.1f} ms | "
        f"P95 {stats.control_p95_ms:.1f} ms | "
        f"max {stats.control_max_ms:.1f} ms",
        f"- 控制响应延时 >200ms 样本数: {stats.control_over_200ms}",
        f"- 控制响应延时 <=200ms 样本数: {stats.control_within_200ms}",
        f"- 控制响应延时 <=200ms 达标率: {stats.control_within_200ms_rate * 100.0:.1f}%",
        "- 口径: 单轮 CSV 中的手动障碍触发 -> PlanningScene 注入 -> "
        "MoveIt 规划结果返回。",
        "",
        "| stamp | case | expected | met | verdict | total ms | control ms | plan ms | scene ms | service wait ms | apply call ms | obstacle center | size |",
        "|---|---|---|---|---|---:|---:|---:|---:|---:|---:|---|---|",
    ]
    for row in measurements:
        lines.append(
            f"| {row['stamp']} | {row['case_label']} | "
            f"{row['expected_verdict']} | {row['expected_met']} | "
            f"{row['verdict']} | "
            f"{row['total_latency_ms']:.1f} | "
            f"{row['control_latency_ms']:.1f} | "
            f"{row['planning_latency_ms']:.1f} | "
            f"{row['apply_scene_latency_ms']:.1f} | "
            f"{row['apply_scene_service_wait_ms']:.1f} | "
            f"{row['apply_scene_call_latency_ms']:.1f} | "
            f"{row['obstacle_center']} | {row['obstacle_size']} |"
        )
    md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return md_path


def _expand_inputs(inputs: list[Path]) -> list[Path]:
    paths: list[Path] = []
    for item in inputs:
        if item.is_dir():
            paths.extend(sorted(item.glob("g5_arm_avoidance_*.csv")))
        else:
            paths.append(item)
    return [
        path for path in paths
        if path.name.startswith("g5_arm_avoidance_")
        and not path.name.startswith("g5_arm_avoidance_summary_")
        and path.suffix == ".csv"
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="汇总 G5 机械臂避障测量 CSV")
    parser.add_argument(
        "inputs",
        nargs="*",
        type=Path,
        default=[Path(__file__).parent / "results"],
        help="CSV files or directories containing g5_arm_avoidance_*.csv",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=Path(__file__).parent / "results",
    )
    return parser


def main() -> int:
    args = build_parser().parse_args()
    paths = _expand_inputs(args.inputs)
    if not paths:
        print("未找到 G5 机械臂避障测量 CSV。", file=sys.stderr)
        return 1
    measurements = read_measurements(paths)
    stats = summarize(measurements)
    report = write_summary(measurements, stats, args.out_dir)
    print(f"G5_ARM_AVOIDANCE_SUMMARY={report}")
    print(
        "G5_ARM_AVOIDANCE_STATS "
        f"n={stats.count} ok={stats.ok_count} "
        f"success={stats.plan_ok_rate * 100.0:.1f}% "
        f"expected={stats.expected_met_rate * 100.0:.1f}% "
        f"within_200ms={stats.total_within_200ms_rate * 100.0:.1f}% "
        f"control_within_200ms={stats.control_within_200ms_rate * 100.0:.1f}% "
        f"control_p95={stats.control_p95_ms:.1f}ms "
        f"p95={stats.total_p95_ms:.1f}ms "
        f"max={stats.total_max_ms:.1f}ms"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
