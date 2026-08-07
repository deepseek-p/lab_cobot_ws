#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f install/setup.bash ]]; then
  colcon build --symlink-install
fi
set +u
source install/setup.bash
set -u

PYTHONPATH="src/lab_cobot_manipulation:src/pymoveit2:${PYTHONPATH:-}" \
python3 -m pytest -q \
  src/lab_cobot_manipulation/test/test_scene_obstacles.py \
  src/lab_cobot_manipulation/test/test_pick_place_sequence.py \
  src/lab_cobot_manipulation/test/test_dynamic_arm_obstacle_node.py \
  benchmarks/test_g5_arm_dynamic_obstacle_probe.py \
  benchmarks/test_g5_arm_avoidance_demo.py \
  benchmarks/test_g5_arm_avoidance_summary.py \
  benchmarks/test_g5_arm_avoidance_batch.py

ros2 pkg executables lab_cobot_manipulation | grep -q \
  "lab_cobot_manipulation dynamic_arm_obstacle_node"

if [[ -d benchmarks/results/g5_offset_20_split_unique_20260723 ]]; then
  python3 benchmarks/g5_arm_avoidance_summary.py \
    benchmarks/results/g5_offset_20_split_unique_20260723 \
    --out-dir /tmp/g5_verify_offline
else
  echo "G5 historical measurement directory not found; skipping summary replay"
fi

echo "G5_VERIFY_OFFLINE_OK"
