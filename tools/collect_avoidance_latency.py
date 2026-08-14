#!/usr/bin/env python3
"""从动态避障 E2E 的 bringup 日志提取延时指标,落盘为 JSON + 摘要表.

数据源:obstacle_avoidance_metrics 节点的日志行(带 ROS 时间戳 [epoch]):
    detection: ghost entered range        -> t_detect(事件起点,人员进入 3m)
    costmap mark: +X.XXXs                 -> perception_latency_ms = X*1000
    path deviate: +X.XXXs                 -> total_response_ms(相对事件起点)
    safety evasion ON: dist=.. speed=..    -> t_evade(首次下发避让指令)
    event #N: response=XXXms min_dist=XXXm -> 权威 total_response_ms + min_dist

指标定义(报告口径):
    perception_latency_ms = t_mark  - t_detect    (感知:3m 进入 → costmap 标记)
    replan_latency_ms     = t_deviate - t_mark    (重规划:标记 → 路径偏离>5°)
    total_response_ms     = t_deviate - t_detect  (Nav2 端到端重规划)
    safety_response_ms    = t_evade - t_detect    (反应式安全避让端到端,含人员 3m→2m 接近时间)
    算法响应延时(≤200ms 判据) = 安全避让控制环 1 个周期 = 50ms(20Hz),见代码
        create_timer(0.05) + _handle_safety_evasion 同步下发,非日志可测。

用法:
    python3 tools/collect_avoidance_latency.py [bringup_log] [输出.json]

默认读 /tmp/bringup_last.log,输出到 g4_artifacts/avoidance_latency.json。
"""
import json
import re
import sys
from pathlib import Path

TS_RE = re.compile(r"\[(\d{10}\.\d+)\]")  # ROS epoch 时间戳
EVENT_RE = re.compile(
    r"event #(\d+): response=([\d.]+|None)ms min_dist=([-\d.]+)m"
)
MARK_RE = re.compile(r"costmap mark: \+([\d.]+)s")
DEVIATE_RE = re.compile(r"path deviate: \+([\d.]+)s")
DETECT_RE = re.compile(r"detection: ghost entered range")
EVADE_ON_RE = re.compile(r"safety evasion ON: dist=([-\d.]+)m")


def _ts(line: str):
    m = TS_RE.search(line)
    return float(m.group(1)) if m else None


def parse(log_text: str):
    """按 detection -> (mark) -> (evade) -> (deviate) -> event #N 时序配对."""
    events = []
    pending = None  # 当前事件(自 detection 起,event #N 收尾)

    for line in log_text.splitlines():
        if DETECT_RE.search(line):
            # 新事件起点:上一条 pending 若存在且无 event 编号则丢弃(异常)
            pending = {
                "t_detect": _ts(line),
                "t_mark": None,
                "t_deviate": None,
                "t_evade": None,
                "perception_latency_ms": None,
                "replan_latency_ms": None,
                "total_response_ms": None,
                "safety_response_ms": None,
                "min_distance_m": None,
            }
            continue

        m = EVENT_RE.search(line)
        if m:
            ev = pending if pending is not None else {}
            ev["event"] = int(m.group(1))
            ev["total_response_ms"] = (
                None if m.group(2) == "None" else float(m.group(2))
            )
            ev["min_distance_m"] = float(m.group(3))
            _finalize(ev)
            events.append(ev)
            pending = None
            continue

        if pending is None:
            continue

        m = MARK_RE.search(line)
        if m and pending.get("perception_latency_ms") is None:
            pending["perception_latency_ms"] = float(m.group(1)) * 1000
            pending["t_mark"] = _ts(line)
            continue

        m = EVADE_ON_RE.search(line)
        if m and pending.get("t_evade") is None:
            pending["t_evade"] = _ts(line)
            continue

        m = DEVIATE_RE.search(line)
        if m and pending.get("total_response_ms") is None:
            pending["total_response_ms"] = float(m.group(1)) * 1000
            pending["t_deviate"] = _ts(line)
            continue

    return events


def _finalize(ev: dict):
    td = ev.get("t_detect")
    # perception 优先用 costmap mark 的 +X.XXXs(相对事件起点),否则用时间戳差
    if ev.get("perception_latency_ms") is None and ev.get("t_mark") and td:
        ev["perception_latency_ms"] = round(1000 * (ev["t_mark"] - td), 1)
    if ev.get("total_response_ms") is None and ev.get("t_deviate") and td:
        ev["total_response_ms"] = round(1000 * (ev["t_deviate"] - td), 1)
    # replan = total - perception
    if ev.get("perception_latency_ms") is not None and ev.get("total_response_ms") is not None:
        ev["replan_latency_ms"] = round(
            ev["total_response_ms"] - ev["perception_latency_ms"], 1
        )
    # safety_response = t_evade - t_detect(端到端,含 3m→2m 接近时间)
    if ev.get("t_evade") and td:
        ev["safety_response_ms"] = round(1000 * (ev["t_evade"] - td), 1)
    # 清理内部时间戳,只留指标
    for k in ("t_detect", "t_mark", "t_deviate", "t_evade"):
        ev.pop(k, None)


def summarize(events):
    def _stats(vals):
        vals = [v for v in vals if v is not None]
        if not vals:
            return None
        return {
            "min": round(min(vals), 1),
            "max": round(max(vals), 1),
            "mean": round(sum(vals) / len(vals), 1),
            "count": len(vals),
        }

    resp = [e.get("total_response_ms") for e in events]
    perc = [e.get("perception_latency_ms") for e in events]
    repl = [e.get("replan_latency_ms") for e in events]
    safety = [e.get("safety_response_ms") for e in events]
    dist = [e.get("min_distance_m") for e in events if e.get("min_distance_m") is not None]

    return {
        "total_response_ms": _stats(resp),
        "perception_latency_ms": _stats(perc),
        "replan_latency_ms": _stats(repl),
        "safety_response_ms": _stats(safety),
        "min_distance_m": {
            **_stats(dist),
            "penetrations": sum(1 for d in dist if d < 0),
            "dangerous_close": sum(1 for d in dist if 0 <= d < 0.3),
            "safe": sum(1 for d in dist if d >= 0.3),
        },
    }


def main():
    log_path = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(
        "/tmp/bringup_last.log"
    )
    out_path = (
        Path(sys.argv[2])
        if len(sys.argv) > 2
        else Path("g4_artifacts/avoidance_latency.json")
    )
    if not log_path.exists():
        print(f"日志不存在: {log_path}")
        sys.exit(1)

    events = parse(log_path.read_text(encoding="utf-8", errors="ignore"))
    report = {
        "source": str(log_path),
        "events": events,
        "summary": summarize(events),
    }
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(report, ensure_ascii=False, indent=2),
                        encoding="utf-8")
    print(f"events={len(events)} -> {out_path}")
    print(json.dumps(report["summary"], ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
