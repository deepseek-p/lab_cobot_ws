"""Batch benchmark for LLM task-decomposition success rate (manual, needs API key)."""
# 对标赛题指标:任务分解与规划成功率 >= 95%
# 用法(不进 colcon test,需真实 API):
#   export LLM_API_KEY=sk-...
#   source install/setup.bash
#   python3 benchmarks/llm_plan_success_rate.py [--api-base URL] [--model NAME] [--repeat N]
# 输出:逐条明细 + 三层判据成功率 + markdown 报表(benchmarks/results/)
import argparse
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

from lab_cobot_bringup.task_planner import (
    DEFAULT_API_BASE,
    DEFAULT_FULL_PLAN,
    DEFAULT_MODEL,
    PlannerConfig,
    plan_actions,
)
from lab_cobot_bringup.task_state_machine import TaskState

S = TaskState  # 缩写

FULL = [S.NAV_TO_PICK, S.DETECT, S.PICK, S.NAV_TO_PLACE, S.PLACE, S.RETURN_HOME]
INSPECT = [S.NAV_TO_PICK, S.DETECT, S.RETURN_HOME]
HOME = [S.RETURN_HOME]


def has_transport_core(steps):
    """Semantic predicate: plan contains the ordered transport backbone."""
    names = [s.name for s in steps]
    idx = []
    for key in ("NAV_TO_PICK", "DETECT", "PICK", "NAV_TO_PLACE", "PLACE"):
        if key not in names:
            return False
        idx.append(names.index(key))
    return idx == sorted(idx)


# 判据三层:
#   exact    — steps 与期望序列完全一致
#   semantic — 满足语义谓词(允许合理变体,如末尾多一个 RETURN_HOME)
#   defense  — 防线行为正确(降级来源/保底序列)
CASES = [
    # --- 模板内表达(exact) ---
    ("把样件从A送到B", "exact", FULL),
    ("回到起始位置待命", "exact", HOME),
    ("去A工位检查一下样件然后回家", "exact", INSPECT),
    # --- 表达变体(semantic:含完整搬运骨架) ---
    ("帮我把工作台上的零件转运到另一个工位", "semantic", has_transport_core),
    ("请把A工位的样件搬运到B工位去", "semantic", has_transport_core),
    ("将样件转移到B工位", "semantic", has_transport_core),
    ("样件需要送到B工位,麻烦了", "semantic", has_transport_core),
    ("先去A工位看看样件在不在,然后把它搬到B工位", "semantic", has_transport_core),
    ("把台上那个方块运过去", "semantic", has_transport_core),
    ("A工位的东西拿到B工位", "semantic", has_transport_core),
    # --- 巡检/复合变体(semantic:含 DETECT 且不含 PICK,或 exact) ---
    ("去看看A工位的样件状态", "semantic",
     lambda steps: S.DETECT in steps and S.PICK not in steps),
    ("检查一下样件,不用搬", "semantic",
     lambda steps: S.DETECT in steps and S.PICK not in steps),
    ("先回家,然后再去A工位看看,再回家", "semantic",
     lambda steps: steps[0] == S.RETURN_HOME and S.DETECT in steps
     and S.PICK not in steps),
    # --- 单动作变体 ---
    ("机器人归位", "exact", HOME),
    ("回去吧", "exact", HOME),
    # --- 防线:超能力范围 → prompt 规则保底完整序列(或规则回退) ---
    ("给我唱首歌", "defense",
     lambda r: r.steps == DEFAULT_FULL_PLAN),
    ("今天天气怎么样", "defense",
     lambda r: r.steps == DEFAULT_FULL_PLAN),
    # --- 防线:动作集表达不了 → 校验器拦截降级,不得执行非法计划 ---
    ("把B工位的东西搬回A工位", "defense",
     lambda r: r.source in ("llm", "fallback_rule")
     and (r.steps == DEFAULT_FULL_PLAN or has_transport_core(r.steps)
          or r.steps == INSPECT)),
]


def judge(case_kind, expected, result):
    """Return (ok, note) for one benchmark case."""
    if case_kind == "exact":
        return result.steps == expected, "exact"
    if case_kind == "semantic":
        return bool(expected(result.steps)), "semantic"
    return bool(expected(result)), "defense"


def main() -> int:
    parser = argparse.ArgumentParser(description="LLM 任务拆解成功率批测")
    parser.add_argument("--api-base", default=DEFAULT_API_BASE)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--repeat", type=int, default=1,
                        help="每条指令重复次数(评估稳定性)")
    args = parser.parse_args()

    api_key = os.environ.get("LLM_API_KEY")
    if not api_key:
        print("错误: 未设置 LLM_API_KEY", file=sys.stderr)
        return 1
    config = PlannerConfig(
        llm_enabled=True, api_base=args.api_base, model=args.model,
        timeout_sec=args.timeout, api_key=api_key,
    )

    rows = []
    for instruction, kind, expected in CASES:
        for rep in range(args.repeat):
            t0 = time.monotonic()
            result = plan_actions(instruction, config)
            dt_ms = (time.monotonic() - t0) * 1000
            ok, note = judge(kind, expected, result)
            rows.append({
                "instruction": instruction, "kind": kind, "rep": rep,
                "ok": ok, "source": result.source,
                "steps": [s.name for s in result.steps],
                "latency_ms": round(dt_ms, 1), "detail": result.detail,
            })
            mark = "PASS" if ok else "FAIL"
            print(f"[{mark}][{note}][{result.source}] {instruction}")
            print(f"    -> {' > '.join(s.name for s in result.steps)}"
                  f"  ({dt_ms:.0f} ms)")

    total = len(rows)
    passed = sum(1 for r in rows if r["ok"])
    llm_served = sum(1 for r in rows if r["source"] == "llm")
    by_kind = {}
    for r in rows:
        agg = by_kind.setdefault(r["kind"], [0, 0])
        agg[0] += r["ok"]
        agg[1] += 1
    lat = sorted(r["latency_ms"] for r in rows if r["source"] == "llm")
    p95 = lat[int(len(lat) * 0.95) - 1] if lat else 0.0

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = Path(__file__).parent / "results"
    out_dir.mkdir(exist_ok=True)
    report = out_dir / f"llm_plan_{stamp}.md"
    lines = [
        f"# LLM 任务拆解成功率报告 {stamp}",
        f"- 模型: {args.model} @ {args.api_base}",
        f"- 用例: {total} 次(指令 {len(CASES)} 条 × repeat {args.repeat})",
        f"- **总成功率: {passed}/{total} = {passed/total*100:.1f}%**"
        f"(赛题指标 >=95%)",
        f"- LLM 直出占比: {llm_served}/{total}"
        f"(其余为设计内的降级路径,降级且行为正确计为成功)",
        f"- LLM 延迟: 均值 {sum(lat)/len(lat):.0f} ms / P95 {p95:.0f} ms"
        if lat else "- LLM 延迟: 无 llm 直出样本",
        "",
        "| 判据层 | 成功/总数 |",
        "|---|---|",
    ]
    for kind, (okc, cnt) in by_kind.items():
        lines.append(f"| {kind} | {okc}/{cnt} |")
    lines += ["", "## 明细", "```json",
              json.dumps(rows, ensure_ascii=False, indent=1), "```"]
    report.write_text("\n".join(lines), encoding="utf-8")
    print(f"\n总成功率 {passed}/{total} = {passed/total*100:.1f}%  "
          f"(报告: {report})")
    return 0 if passed == total else 2


if __name__ == "__main__":
    sys.exit(main())
