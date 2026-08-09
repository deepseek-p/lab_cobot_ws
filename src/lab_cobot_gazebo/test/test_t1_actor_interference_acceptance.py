#!/usr/bin/env python3
"""Run the strict T1 actor-interference acceptance test."""
import json
import os
import re
import signal
import subprocess
import time
from pathlib import Path

import pytest

T1_INSTRUCTION = "把样件从A工位搬运到B工位，然后返回原点"
TASK_TIMEOUT_SEC = int(os.environ.get("LAB_COBOT_T1_TIMEOUT_SEC", "1200"))
LAUNCH_STABILIZE_SEC = int(os.environ.get("LAB_COBOT_STABILIZE_SEC", "150"))
COOLDOWN_SEC = int(os.environ.get("LAB_COBOT_COOLDOWN_SEC", "20"))
NUM_RUNS = int(os.environ.get("LAB_COBOT_T1_RUNS", "5"))
MIN_DISTANCE_THRESHOLD_M = 0.3

WORKSPACE = Path(__file__).resolve().parents[3]
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
WORKSPACE_SETUP = WORKSPACE / "install" / "setup.bash"
ARTIFACT_ROOT = WORKSPACE / "artifacts" / "dynamic_actor_t1"
STRICT_ARTIFACT_ROOT = ARTIFACT_ROOT / "pytest_acceptance"


def _source_cmd() -> str:
    return f"source {ROS_SETUP} && source {WORKSPACE_SETUP}"


def _run_with_timeout(cmd: str, timeout: float, env: dict) -> subprocess.CompletedProcess:
    proc = subprocess.Popen(
        ["bash", "-c", cmd],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        env=env,
        start_new_session=True,
    )
    try:
        stdout, stderr = proc.communicate(timeout=timeout)
        return subprocess.CompletedProcess(cmd, proc.returncode, stdout, stderr)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        except OSError:
            pass
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except OSError:
                pass
            proc.wait()
        raise


def _cleanup_sim_processes() -> None:
    subprocess.run(
        [
            "bash",
            "-c",
            "pkill -INT -f 'ros2 launch lab_cobot_bringup' 2>/dev/null || true; "
            "pkill -INT -f gzserver 2>/dev/null || true; "
            "pkill -INT -f gzclient 2>/dev/null || true; "
            "pkill -INT -f rviz2 2>/dev/null || true; "
            "sleep 3; "
            "pkill -9 -f gzserver 2>/dev/null || true; "
            "pkill -9 -f gzclient 2>/dev/null || true; "
            "pkill -9 -f 'ros2 topic echo' 2>/dev/null || true",
        ],
        timeout=20,
    )


def _publish_instruction(env: dict) -> None:
    code = (
        "import time;"
        "import rclpy;"
        "from std_msgs.msg import String;"
        "rclpy.init();"
        "node=rclpy.create_node('t1_actor_instruction_pub');"
        "pub=node.create_publisher(String, '/task/instruction', 10);"
        "time.sleep(1.0);"
        "msg=String();"
        f"msg.data={T1_INSTRUCTION!r};"
        "[pub.publish(msg) or rclpy.spin_once(node, timeout_sec=0.05) for _ in range(60)];"
        "node.destroy_node();"
        "rclpy.shutdown()"
    )
    result = subprocess.run(
        ["bash", "-c", f"{_source_cmd()} && python3 -c {json.dumps(code)}"],
        timeout=20,
        capture_output=True,
        text=True,
        env=env,
    )
    if result.returncode != 0:
        raise RuntimeError(f"instruction publish failed: {result.stderr.strip()}")


def _parse_metrics_from_bringup_log(raw: str) -> list[dict]:
    events = []
    pattern = re.compile(r"event #\d+: response=.*? min_dist=([0-9.]+)m")
    for match in pattern.finditer(raw):
        events.append({"min_distance_m": float(match.group(1))})
    return events


def _parse_metrics_text(raw: str) -> list[dict]:
    events = []
    for line in raw.splitlines():
        payload = line.strip()
        if not payload:
            continue
        if "data:" in payload:
            payload = payload.split("data:", 1)[1].strip().strip("'\"")
        if not payload.startswith("{"):
            match = re.search(r"\{.*\}", payload)
            if not match:
                continue
            payload = match.group(0)
        try:
            events.append(json.loads(payload))
        except json.JSONDecodeError:
            continue
    return events


def _extract_statuses(raw: str) -> list[str]:
    statuses = []
    for line in raw.splitlines():
        if "data:" not in line:
            continue
        statuses.append(line.split("data:", 1)[1].strip().strip("'\""))
    return statuses


@pytest.fixture(scope="session")
def workspace_ready():
    assert WORKSPACE_SETUP.exists(), f"missing {WORKSPACE_SETUP}; run colcon build first"
    STRICT_ARTIFACT_ROOT.mkdir(parents=True, exist_ok=True)


def _run_one(run_idx: int) -> dict:
    env = os.environ.copy()
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    run_dir = STRICT_ARTIFACT_ROOT / f"run_{run_idx:02d}"
    run_dir.mkdir(parents=True, exist_ok=True)
    bringup_log = run_dir / "bringup.log"
    metrics_file = run_dir / "avoidance_metrics.txt"
    summary_file = run_dir / "summary.json"

    _cleanup_sim_processes()
    launch_cmd = (
        f"{_source_cmd()} && "
        "ros2 launch lab_cobot_bringup lab_cobot.launch.py "
        "gui:=false use_rviz:=false enable_actor:=true "
        "use_truth_pose:=true use_refine_detect:=false use_wrist_detect:=false "
        "lighting_profile:=normal skip_visual_dock:=false nav_only:=false "
        "launch_perception:=true"
    )
    with bringup_log.open("w", encoding="utf-8") as log_fp:
        proc = subprocess.Popen(
            ["bash", "-c", launch_cmd],
            stdout=log_fp,
            stderr=subprocess.STDOUT,
            env=env,
            start_new_session=True,
        )
        metrics_proc = None
        try:
            deadline = time.monotonic() + LAUNCH_STABILIZE_SEC
            topic_ready = False
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    raise RuntimeError(f"bringup died during stabilize; see {bringup_log}")
                try:
                    topics = _run_with_timeout(
                        f"{_source_cmd()} && ros2 topic list 2>/dev/null",
                        timeout=10,
                        env=env,
                    )
                    topic_ready = topic_ready or "/task/status" in topics.stdout
                except subprocess.TimeoutExpired:
                    pass
                time.sleep(5.0)
            if not topic_ready:
                raise RuntimeError("/task/status not available before timeout")

            metrics_file.write_text("", encoding="utf-8")
            metrics_code = "\n".join([
                "import rclpy",
                "from std_msgs.msg import String",
                f"path={str(metrics_file)!r}",
                "rclpy.init()",
                "node=rclpy.create_node('t1_metrics_jsonl')",
                "fp=open(path, 'a', encoding='utf-8')",
                "def cb(msg):",
                "    fp.write(msg.data + '\\n')",
                "    fp.flush()",
                "node.create_subscription(String, '/obstacle_avoidance/metrics', cb, 10)",
                "rclpy.spin(node)",
                "fp.close()",
            ])
            metrics_proc = subprocess.Popen(
                ["bash", "-c", f"{_source_cmd()} && python3 -c {json.dumps(metrics_code)}"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=env,
                start_new_session=True,
            )
            status_file = run_dir / "task_status.txt"
            status_file.write_text("", encoding="utf-8")
            status_fp = status_file.open("a", encoding="utf-8")
            status_proc = subprocess.Popen(
                ["bash", "-c", f"{_source_cmd()} && ros2 topic echo /task/status 2>/dev/null"],
                stdout=status_fp,
                stderr=subprocess.DEVNULL,
                env=env,
                start_new_session=True,
            )
            time.sleep(1.5)
            _publish_instruction(env)

            task_status = ""
            task_done = False
            start = time.monotonic()
            try:
                while time.monotonic() - start < TASK_TIMEOUT_SEC:
                    if proc.poll() is not None:
                        raise RuntimeError(f"bringup died during T1; see {bringup_log}")
                    statuses = _extract_statuses(status_file.read_text(encoding="utf-8"))
                    if statuses:
                        task_status = statuses[-1]
                    if any("DONE" in status for status in statuses):
                        task_done = True
                        break
                    if any("FAILED" in status for status in statuses):
                        break
                    time.sleep(1.0)
            finally:
                status_fp.close()
                if status_proc.poll() is None:
                    try:
                        os.killpg(os.getpgid(status_proc.pid), signal.SIGTERM)
                        status_proc.wait(timeout=5)
                    except (OSError, subprocess.TimeoutExpired):
                        try:
                            os.killpg(os.getpgid(status_proc.pid), signal.SIGKILL)
                        except OSError:
                            pass

            if metrics_proc is not None and metrics_proc.poll() is None:
                os.killpg(os.getpgid(metrics_proc.pid), signal.SIGTERM)
                try:
                    metrics_proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(os.getpgid(metrics_proc.pid), signal.SIGKILL)

            raw = metrics_file.read_text(encoding="utf-8") if metrics_file.exists() else ""
            events = _parse_metrics_text(raw)
            if not events and bringup_log.exists():
                events = _parse_metrics_from_bringup_log(
                    bringup_log.read_text(encoding="utf-8", errors="ignore")
                )
            distances = [
                float(e["min_distance_m"])
                for e in events
                if e.get("min_distance_m") is not None
            ]
            min_distance = min(distances) if distances else None
            summary = {
                "run": run_idx,
                "success": bool(task_done),
                "task_status": task_status,
                "avoidance_events": len(events),
                "min_distance_m": min_distance,
                "passed": bool(
                    task_done
                    and len(events) > 0
                    and min_distance is not None
                    and min_distance >= MIN_DISTANCE_THRESHOLD_M
                ),
                "artifacts": {
                    "bringup_log": str(bringup_log),
                    "metrics": str(metrics_file),
                    "status": str(run_dir / "task_status.txt"),
                },
            }
            summary_file.write_text(
                json.dumps(summary, ensure_ascii=False, indent=2),
                encoding="utf-8",
            )
            return summary
        finally:
            if metrics_proc is not None and metrics_proc.poll() is None:
                try:
                    os.killpg(os.getpgid(metrics_proc.pid), signal.SIGKILL)
                except OSError:
                    pass
            if proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=30)
                except (OSError, subprocess.TimeoutExpired):
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except OSError:
                        pass
            _cleanup_sim_processes()
            time.sleep(COOLDOWN_SEC)


@pytest.mark.slow
@pytest.mark.e2e
def test_t1_actor_interference_five_consecutive_runs(workspace_ready):
    results = []
    for i in range(1, NUM_RUNS + 1):
        print(f"\n=== T1 actor run {i}/{NUM_RUNS} ===")
        result = _run_one(i)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        results.append(result)
        assert result["passed"], f"run {i} failed strict gate; see {result['artifacts']}"

    latest_summary = {
        "runs": NUM_RUNS,
        "passed": all(r["passed"] for r in results),
        "min_distance_m": min(r["min_distance_m"] for r in results),
        "results": results,
    }
    (ARTIFACT_ROOT / "latest_summary.json").write_text(
        json.dumps(latest_summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    assert latest_summary["passed"] is True
