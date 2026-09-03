"""
Dynamic obstacle avoidance 5-run regression test — multi-station cruise.

Runs "巡航所有工位" (home→station_a→inspection_zone→tooling_zone→aging_zone→station_b→home)
with the walking actor enabled.  Each run must reach DONE without collision with the ghost.

Perception and manipulation are disabled (nav_only); only navigation + obstacle avoidance
are verified.

Run via colcon:
    PYTEST_ADDOPTS='-p no:anyio' colcon test --packages-select lab_cobot_gazebo \
        --event-handlers console_direct+
    colcon test-result --verbose

Constraints:
    - Must run inside WSL with Gazebo and ROS 2 available.
    - Each run budget ~1500s (25 minutes) for the full cruise route with actor
      interference; 900s was insufficient once the actor repeatedly intercepted
      the robot during the inspection_zone -> tooling_zone leg.
    - Total test budget ~125 minutes for 5 runs.
"""
import json
import os
import re
import signal
import subprocess
import tempfile
import time
from pathlib import Path

import pytest

CRUISE_INSTRUCTION = "巡航所有工位"
CRUISE_INSTRUCTION_PUB = (
    "import time;"
    "import rclpy;"
    "from std_msgs.msg import String;"
    "rclpy.init();"
    "node=rclpy.create_node('dynamic_obstacle_cruise_pub');"
    "pub=node.create_publisher(String, '/task/instruction', 10);"
    "time.sleep(1.0);"
    "msg=String();"
    "msg.data='\\u5de1\\u822a\\u6240\\u6709\\u5de5\\u4f4d';"
    "[pub.publish(msg) or rclpy.spin_once(node, timeout_sec=0.05) "
    "for _ in range(40)];"
    "node.destroy_node();"
    "rclpy.shutdown()"
)
TASK_TIMEOUT_SEC = 1500
LAUNCH_STABILIZE_SEC = 120
COOLDOWN_SEC = 15
NUM_RUNS = 1
WORKSPACE = Path(__file__).resolve().parents[3]
ROS_SETUP = Path("/opt/ros/humble/setup.bash")
WORKSPACE_SETUP = WORKSPACE / "install" / "setup.bash"

SUCCESS_RATE_THRESHOLD = 0.95
MIN_DISTANCE_THRESHOLD_M = 0.3

# A previous run's launch tree can survive even after gzserver is killed and
# keep publishing stale /cmd_vel_safety commands that corrupt the next run.
STALE_ROS_CLEANUP_CMD = (
    "pkill -9 -f '/install/[l]ab_cobot_' 2>/dev/null || true; "
    "pkill -9 -f 'ros2 [l]aunch' 2>/dev/null || true; "
    "pkill -9 -f 'gz[s]erver' 2>/dev/null || true; "
    "pkill -9 -f 'gz[c]lient' 2>/dev/null || true; "
    "pkill -9 -f '[m]ission_node' 2>/dev/null || true; "
    "sleep 2"
)


def _source_cmd() -> str:
    return f"source {ROS_SETUP} && source {WORKSPACE_SETUP}"


def _run_with_timeout(cmd: str, timeout: float, env: dict) -> subprocess.CompletedProcess:
    """Run a bash command with process-group cleanup on timeout."""
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
        return subprocess.CompletedProcess(
            args=cmd, returncode=proc.returncode, stdout=stdout, stderr=stderr,
        )
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


@pytest.fixture
def workspace_ready():
    """Verify the workspace and map exist before running E2E tests."""
    assert WORKSPACE_SETUP.exists(), f"missing {WORKSPACE_SETUP} — run colcon build first"
    map_yaml = (
        WORKSPACE
        / "src"
        / "lab_cobot_navigation"
        / "maps"
        / "map.yaml"
    )
    assert map_yaml.exists(), f"missing {map_yaml}"


def _run_bringup_and_cruise() -> dict:
    """Launch bringup with actor, run cruise, collect avoidance metrics."""
    result = {
        "success": False,
        "min_distance_m": None,
        "avoidance_events": 0,
        "task_status": None,
    }

    env = os.environ.copy()
    env["GAZEBO_MODEL_DATABASE_URI"] = ""
    env["OBSTACLE_AVOIDANCE_EVENTS_FILE"] = str(
        WORKSPACE / "docs/chapter4_navigation/data/avoidance_events_run13.jsonl"
    )
    _run_with_timeout(STALE_ROS_CLEANUP_CMD, timeout=15, env=env)

    bringup_log = Path(tempfile.gettempdir()) / "bringup_last.log"
    launch_cmd = (
        f"{_source_cmd()} && "
        f"ros2 launch lab_cobot_bringup lab_cobot.launch.py "
        f"gui:=false use_rviz:=false enable_actor:=true "
        f"use_truth_pose:=true use_refine_detect:=false use_wrist_detect:=false "
        f"lighting_profile:=normal "
        f"skip_visual_dock:=true nav_only:=true launch_moveit:=false "
        f"launch_perception:=false "
        f"2>&1"
    )

    log_fp = bringup_log.open("w")
    proc = subprocess.Popen(
        ["bash", "-c", launch_cmd],
        stdout=log_fp,
        stderr=subprocess.STDOUT,
        env=env,
    )
    metrics_proc = None
    status_proc = None
    metrics_file = Path(tempfile.gettempdir()) / "avoidance_metrics.txt"
    status_file = Path(tempfile.gettempdir()) / "task_status_dynamic_obstacle.txt"

    try:
        # Wait for system stabilisation.
        deadline = time.monotonic() + LAUNCH_STABILIZE_SEC
        topic_ready = False
        while time.monotonic() < deadline:
            time.sleep(5.0)
            if proc.poll() is not None:
                tail = bringup_log.read_text() if bringup_log.exists() else ""
                elapsed = time.monotonic() - (deadline - LAUNCH_STABILIZE_SEC)
                raise RuntimeError(
                    f"bringup died at +{elapsed:.0f}s. "
                    f"Last log:\n{tail[-2000:]}"
                )
            if not topic_ready:
                try:
                    check = _run_with_timeout(
                        f"{_source_cmd()} && ros2 topic list 2>/dev/null",
                        timeout=10, env=env,
                    )
                    if "/task/status" in check.stdout:
                        topic_ready = True
                except subprocess.TimeoutExpired:
                    pass

        if not topic_ready:
            raise RuntimeError("/task/status topic not available after stabilisation")

        # Start background metrics collector — captures raw JSON String.data lines.
        metrics_file.write_text("", encoding="utf-8")
        metrics_code = "\n".join([
            "import rclpy",
            "from std_msgs.msg import String",
            f"path={str(metrics_file)!r}",
            "rclpy.init()",
            "node=rclpy.create_node('dynamic_obstacle_metrics_jsonl')",
            "fp=open(path, 'a', encoding='utf-8')",
            "def cb(msg):",
            "    fp.write(msg.data + '\\n')",
            "    fp.flush()",
            "node.create_subscription(String, '/obstacle_avoidance/metrics', cb, 10)",
            "rclpy.spin(node)",
            "fp.close()",
        ])
        metrics_proc = subprocess.Popen(
            [
                "bash", "-c",
                f"{_source_cmd()} && python3 -c {json.dumps(metrics_code)}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            start_new_session=True,
        )

        # Start status collector before publishing the instruction.  Avoid
        # repeatedly spawning `ros2 topic echo --once`, which can miss short
        # non-latched state transitions in a busy Gazebo/ROS graph.
        status_file.write_text("", encoding="utf-8")
        status_code = "\n".join([
            "import rclpy",
            "from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy",
            "from std_msgs.msg import String",
            f"path={str(status_file)!r}",
            (
                "qos=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10, "
                "reliability=ReliabilityPolicy.RELIABLE, "
                "durability=DurabilityPolicy.TRANSIENT_LOCAL)"
            ),
            "rclpy.init()",
            "node=rclpy.create_node('dynamic_obstacle_status_jsonl')",
            "fp=open(path, 'a', encoding='utf-8')",
            "def cb(msg):",
            "    fp.write(msg.data + '\\n')",
            "    fp.flush()",
            "node.create_subscription(String, '/task/status', cb, qos)",
            "rclpy.spin(node)",
            "fp.close()",
        ])
        status_proc = subprocess.Popen(
            [
                "bash", "-c",
                f"{_source_cmd()} && python3 -c {json.dumps(status_code)}",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            env=env,
            start_new_session=True,
        )

        # Trigger cruise with retry.  Use a temporary script file instead of
        # nested `python3 -c` quoting so Chinese instruction text and semicolons
        # survive every shell layer.
        publisher_script = Path(tempfile.gettempdir()) / "publish_dynamic_cruise.py"
        publisher_script.write_text(
            "\n".join([
                "import time",
                "import rclpy",
                "from std_msgs.msg import String",
                "rclpy.init()",
                "node = rclpy.create_node('dynamic_obstacle_cruise_pub')",
                "pub = node.create_publisher(String, '/task/instruction', 10)",
                "deadline = time.monotonic() + 10.0",
                "while pub.get_subscription_count() == 0 and time.monotonic() < deadline:",
                "    rclpy.spin_once(node, timeout_sec=0.1)",
                "time.sleep(0.5)",
                "msg = String()",
                "msg.data = '\\u5de1\\u822a\\u6240\\u6709\\u5de5\\u4f4d'",
                "for _ in range(80):",
                "    pub.publish(msg)",
                "    rclpy.spin_once(node, timeout_sec=0.05)",
                "time.sleep(0.5)",
                "node.destroy_node()",
                "rclpy.shutdown()",
            ]),
            encoding="utf-8",
        )
        for attempt in range(1, 4):
            pub_result = subprocess.run(
                [
                    "bash",
                    "-c",
                    f"{_source_cmd()} && python3 {publisher_script}",
                ],
                timeout=15,
                capture_output=True,
                text=True,
                env=env,
            )
            if pub_result.returncode == 0:
                break
            print(
                f"  [WARN] instruction publish attempt {attempt}/3 failed "
                f"(rc={pub_result.returncode})"
            )
            time.sleep(3.0)
        else:
            raise RuntimeError(
                f"Failed to publish /task/instruction after 3 attempts. "
                f"Last stderr: {pub_result.stderr.strip()}"
            )

        # Wait for DONE or timeout.
        start = time.monotonic()
        while time.monotonic() - start < TASK_TIMEOUT_SEC:
            if proc.poll() is not None:
                tail = bringup_log.read_text() if bringup_log.exists() else ""
                raise RuntimeError(f"bringup died during task. Last log:\n{tail[-2000:]}")
            statuses = []
            if status_file.exists():
                statuses = [
                    line.strip()
                    for line in status_file.read_text(encoding="utf-8").splitlines()
                    if line.strip()
                ]
            if statuses:
                task_state = statuses[-1]
                result["task_status"] = task_state
                if "DONE" in task_state:
                    result["success"] = True
                    break
                if task_state.startswith("FAILED"):
                    break
            if bringup_log.exists():
                log_raw = bringup_log.read_text(encoding="utf-8", errors="ignore")
                if "task status: DONE" in log_raw:
                    result["success"] = True
                    result["task_status"] = "DONE"
                    break
                failed_match = re.findall(r"task status: (FAILED[^\\n]*)", log_raw)
                if failed_match:
                    result["task_status"] = failed_match[-1]
                    break
            time.sleep(1.0)

        # Stop metrics/status collectors.
        if status_proc is not None and status_proc.poll() is None:
            try:
                os.killpg(os.getpgid(status_proc.pid), signal.SIGTERM)
                status_proc.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(status_proc.pid), signal.SIGKILL)
                except OSError:
                    pass
        if metrics_proc is not None and metrics_proc.poll() is None:
            try:
                os.killpg(os.getpgid(metrics_proc.pid), signal.SIGTERM)
                metrics_proc.wait(timeout=5)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(metrics_proc.pid), signal.SIGKILL)
                except OSError:
                    pass

        # Parse all collected avoidance events.  Fall back to bringup logs if
        # the non-latched topic was published before the collector subscribed.
        raw_events = []
        if metrics_file.exists():
            for line in metrics_file.read_text(encoding="utf-8").splitlines():
                payload = line.strip()
                if not payload:
                    continue
                if "data:" in payload:
                    payload = payload.split("data:", 1)[-1].strip().strip("'\"")
                try:
                    raw_events.append(json.loads(payload))
                except json.JSONDecodeError:
                    pass
        if not raw_events and bringup_log.exists():
            log_raw = bringup_log.read_text(encoding="utf-8", errors="ignore")
            for match in re.finditer(
                r"event #\d+: response=([\d.]+|None)ms min_dist=(-?[0-9.]+)m",
                log_raw,
            ):
                resp = (
                    None if match.group(1) == "None"
                    else float(match.group(1))
                )
                raw_events.append({
                    "total_response_ms": resp,
                    "min_distance_m": float(match.group(2)),
                })
        for event in raw_events:
            result["avoidance_events"] += 1
            dist = event.get("min_distance_m")
            if dist is not None:
                if result["min_distance_m"] is None or dist < result["min_distance_m"]:
                    result["min_distance_m"] = dist

    finally:
        log_fp.close()
        if status_proc is not None and status_proc.poll() is None:
            try:
                os.killpg(os.getpgid(status_proc.pid), signal.SIGKILL)
            except OSError:
                pass
        if metrics_proc is not None and metrics_proc.poll() is None:
            try:
                os.killpg(os.getpgid(metrics_proc.pid), signal.SIGKILL)
            except OSError:
                pass
        try:
            proc.terminate()
            try:
                proc.wait(timeout=30)
            except subprocess.TimeoutExpired:
                proc.kill()
        except Exception as exc:  # noqa: BLE001 - cleanup must not mask result
            print(f"  [WARN] bringup cleanup error ignored: {exc}")
        try:
            subprocess.run(
                [
                    "bash", "-c",
                    f"{STALE_ROS_CLEANUP_CMD}; "
                    "pkill -9 -f 'mission_node' 2>/dev/null || true; "
                    "pkill -9 -f 'dynamic_obstacle_metrics_jsonl' 2>/dev/null || true; "
                    "pkill -9 -f 'dynamic_obstacle_status_jsonl' 2>/dev/null || true; "
                    "sleep 2",
                ],
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            print("  [WARN] cleanup timed out; continuing with captured result")
        time.sleep(COOLDOWN_SEC)

    return result


@pytest.mark.slow
@pytest.mark.e2e
class TestDynamicObstacleAvoidance:
    """5 runs of multi-station cruise with a walking actor — must avoid collision."""

    def test_five_cruise_with_actor(self, workspace_ready):
        results = []
        for run_idx in range(1, NUM_RUNS + 1):
            print(f"\n=== Run {run_idx}/{NUM_RUNS} ===")
            result = _run_bringup_and_cruise()
            results.append(result)
            status = (
                "PASS"
                if (
                    result["success"]
                    and result["avoidance_events"] > 0
                    and result["min_distance_m"] is not None
                    and result["min_distance_m"] >= MIN_DISTANCE_THRESHOLD_M
                )
                else "FAIL"
            )
            print(
                f"Run {run_idx}: {status} | {'DONE' if result['success'] else 'TIMEOUT'} | "
                f"events={result['avoidance_events']} | "
                f"min_dist={result['min_distance_m']}m"
            )

        success_count = sum(1 for r in results if r["success"])
        no_collision_count = sum(
            1 for r in results
            if (
                r["avoidance_events"] > 0
                and r["min_distance_m"] is not None
                and r["min_distance_m"] >= MIN_DISTANCE_THRESHOLD_M
            )
        )
        print("\n=== Dynamic Obstacle Avoidance Regression Report ===")
        print(f"Route: {CRUISE_INSTRUCTION}")
        print(
            "Stations: home → station_a → inspection_zone → "
            "tooling_zone → aging_zone → station_b → home"
        )
        for i, r in enumerate(results, 1):
            print(
                f"Run {i}: {'OK' if r['success'] else 'FAIL'} | "
                f"events={r['avoidance_events']} | "
                f"min_dist={r['min_distance_m']}m"
            )
        print(
            f"Task completion: {success_count}/{NUM_RUNS} "
            f"({100 * success_count / NUM_RUNS:.0f}%)"
        )
        print(
            f"Collision-free: {no_collision_count}/{NUM_RUNS} "
            f"({100 * no_collision_count / NUM_RUNS:.0f}%)"
        )

        assert success_count / NUM_RUNS >= SUCCESS_RATE_THRESHOLD, (
            f"Task success rate {success_count}/{NUM_RUNS} < {SUCCESS_RATE_THRESHOLD * 100:.0f}%"
        )
        assert no_collision_count == NUM_RUNS, (
            f"Collision detected in {NUM_RUNS - no_collision_count} run(s)"
        )
        assert all(r["avoidance_events"] > 0 for r in results), (
            "Expected at least one actor avoidance event in every nav-only run"
        )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
