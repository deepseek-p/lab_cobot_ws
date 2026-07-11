"""Manual probe for LLM instruction planning quality (not a colcon test)."""
# 用法(真 API,不进 CI):
#   export LLM_API_KEY=sk-...
#   source install/setup.bash
#   python3 src/lab_cobot_bringup/scripts/manual_llm_plan_check.py [指令...]
#   切换供应商: --api-base <openai兼容端点> --model <模型名>
import argparse
import os
import sys

from lab_cobot_bringup.task_planner import (
    DEFAULT_API_BASE,
    DEFAULT_MODEL,
    PlannerConfig,
    plan_actions,
)

DEFAULT_PROBES = [
    "把样件从A送到B",
    "去A工位检查一下样件然后回家",
    "回到起始位置待命",
    "先去A工位看看样件在不在,然后把它搬到B工位",
    "帮我把工作台上的零件转运到另一个工位",
]


def main() -> int:
    parser = argparse.ArgumentParser(description="LLM 任务拆解质量探针")
    parser.add_argument("instructions", nargs="*", help="待拆解指令(缺省用内置样例)")
    parser.add_argument("--api-base", default=DEFAULT_API_BASE)
    parser.add_argument("--model", default=DEFAULT_MODEL)
    parser.add_argument("--timeout", type=float, default=20.0)
    args = parser.parse_args()

    api_key = os.environ.get("LLM_API_KEY")
    if not api_key:
        print("错误: 未设置 LLM_API_KEY 环境变量", file=sys.stderr)
        return 1

    config = PlannerConfig(
        llm_enabled=True,
        api_base=args.api_base,
        model=args.model,
        timeout_sec=args.timeout,
        api_key=api_key,
    )
    probes = args.instructions or DEFAULT_PROBES
    failures = 0
    for instruction in probes:
        result = plan_actions(instruction, config)
        steps = " -> ".join(s.name for s in result.steps)
        marker = "OK " if result.source == "llm" else "FALLBACK"
        if result.source != "llm":
            failures += 1
        print(f"[{marker}] {instruction}")
        print(f"    source={result.source} detail={result.detail}")
        print(f"    plan: {steps}")
    print(f"\n{len(probes) - failures}/{len(probes)} 条走通 LLM 拆解")
    return 0 if failures == 0 else 2


if __name__ == "__main__":
    sys.exit(main())
