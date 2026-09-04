"""Pure helpers for checking the V3 six-condition benchmark evidence."""
from __future__ import annotations


DEFAULT_CONDITIONS = (
    "C1_normal_visible",
    "C2_normal_occluded",
    "C3_dark_visible",
    "C4_dark_occluded",
    "C5_reflective_visible",
    "C6_reflective_occluded",
)


def condition_result(summary, minimum_evaluations: int):
    """Return a stable V3 completion record for one modern summary."""
    evaluations = summary.get("evaluations")
    if not isinstance(evaluations, int) or evaluations < 0:
        return None
    required = summary.get("required_labels", [])
    evaluated = set(summary.get("evaluated_labels", []))
    labels_complete = (
        not isinstance(required, list)
        or all(str(label) in evaluated for label in required)
    )
    return {
        "condition": str(summary.get("condition", "unknown")),
        "evaluations": evaluations,
        "recognition_rate": summary.get("recognition_rate"),
        "mean_position_error_m": summary.get("mean_position_error_m"),
        "complete": (
            evaluations >= minimum_evaluations
            and labels_complete
            and bool(summary.get("complete", True))),
    }


def build_aggregate_report(summaries, expected_conditions, minimum_evaluations):
    """Build a Markdown report without treating legacy result files as valid."""
    modern = {}
    ignored = []
    for summary in summaries:
        item = condition_result(summary, minimum_evaluations)
        if item is None:
            ignored.append(str(summary.get("condition", "unknown")))
        else:
            modern[item["condition"]] = item

    rows = [
        "# 六工况三维定位识别率", "",
        "成功定义：检测三维坐标与 Gazebo 建图真值的欧氏误差不超过工况阈值。", "",
        f"完整性要求：六个指定工况均至少有 {minimum_evaluations} 个有效三维定位。", "",
        "| 工况 | 有效定位数 | 识别率 | 平均位置误差 (m) | 数据完整 |",
        "|---|---:|---:|---:|---|",
    ]
    missing = []
    for condition in expected_conditions:
        item = modern.get(condition)
        if item is None:
            rows.append(f"| {condition} | — | — | — | 否（未生成新口径结果） |")
            missing.append(condition)
            continue
        rate = item["recognition_rate"]
        mean = item["mean_position_error_m"]
        rows.append(
            f"| {condition} | {item['evaluations']} | "
            f"{'—' if rate is None else f'{rate:.1%}'} | "
            f"{'—' if mean is None else f'{mean:.3f}'} | "
            f"{'是' if item['complete'] else '否（样本不足）'} |"
        )
        if not item["complete"]:
            missing.append(condition)
    rows.extend(["", "## 结论", ""])
    if missing:
        rows.append(
            "本轮尚不能作为 V3 完成结论；缺少或样本不足的工况："
            + "、".join(missing) + "。"
        )
    else:
        rows.append("六工况样本量完整；请结合各工况识别率和失败图作鲁棒性结论。")
    if ignored:
        rows.extend([
            "", "## 未纳入本报告的旧口径结果", "",
            "以下目录缺少当前三维误差字段，未作为 V3 验收证据："
            + "、".join(ignored) + "。",
        ])
    return "\n".join(rows) + "\n"
