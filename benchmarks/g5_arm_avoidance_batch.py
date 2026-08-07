"""Batch runner for G5 arm dynamic obstacle avoidance measurements."""
import argparse
import subprocess
import sys
from pathlib import Path


THIS_DIR = Path(__file__).resolve().parent
DEMO = THIS_DIR / "g5_arm_avoidance_demo.py"
SUMMARY = THIS_DIR / "g5_arm_avoidance_summary.py"

CASES = {
    "offset": {
        "case_label": "offset_obstacle",
        "expected_verdict": "PLAN_OK",
        "obstacle_center": ("0.35", "0.12", "0.50"),
        "obstacle_size": ("0.12", "0.12", "0.20"),
    },
    "blocked": {
        "case_label": "blocked_obstacle",
        "expected_verdict": "PLAN_FAILED",
        "obstacle_center": ("0.42", "0.0", "0.52"),
        "obstacle_size": ("0.20", "0.28", "0.28"),
    },
    "far": {
        "case_label": "far_obstacle",
        "expected_verdict": "PLAN_OK",
        "obstacle_center": ("2.0", "2.0", "1.0"),
        "obstacle_size": ("0.10", "0.10", "0.10"),
    },
}


def build_demo_command(args, case_name: str) -> list[str]:
    case = CASES[case_name]
    cmd = [
        sys.executable,
        str(DEMO),
        "--case-label",
        case["case_label"],
        "--expected-verdict",
        case["expected_verdict"],
        "--obstacle-center",
        *case["obstacle_center"],
        "--obstacle-size",
        *case["obstacle_size"],
        "--timeout",
        str(args.timeout),
        "--scene-timeout",
        str(args.scene_timeout),
        "--out-dir",
        str(args.out_dir),
    ]
    if args.no_use_sim_time:
        cmd.append("--no-use-sim-time")
    return cmd


def build_summary_command(args) -> list[str]:
    return [
        sys.executable,
        str(SUMMARY),
        str(args.out_dir),
        "--out-dir",
        str(args.out_dir),
    ]


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="批量运行 G5 机械臂避障测量")
    parser.add_argument(
        "--case",
        choices=sorted(CASES),
        action="append",
        dest="cases",
        help="Case to run; may be repeated. Default: offset",
    )
    parser.add_argument("--runs", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--scene-timeout", type=float, default=10.0)
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=THIS_DIR / "results",
    )
    parser.add_argument("--no-summary", action="store_true")
    parser.add_argument("--no-use-sim-time", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if args.runs <= 0:
        print("--runs must be positive", file=sys.stderr)
        return 2

    cases = args.cases if args.cases else ["offset"]
    failures = 0
    for case_name in cases:
        for i in range(args.runs):
            print(f"G5_BATCH case={case_name} run={i + 1}/{args.runs}", flush=True)
            result = subprocess.run(build_demo_command(args, case_name), check=False)
            if result.returncode != 0:
                failures += 1

    if not args.no_summary:
        subprocess.run(build_summary_command(args), check=False)

    if failures:
        print(f"G5_BATCH_UNEXPECTED_FAILURES={failures}", file=sys.stderr)
        return 1
    print("G5_BATCH_OK")
    return 0


if __name__ == "__main__":
    sys.exit(main())
