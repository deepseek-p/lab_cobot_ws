#!/bin/bash
# 导航栈救援:lifecycle 编排挂死时手动逐节点拉起(2026-07-10 GUI 演示实测有效)
# 用法: bash tools/nav_stack_rescue.sh
set -u
source /opt/ros/humble/setup.bash
source "$(dirname "$0")/../install/setup.bash"
export ROS_LOCALHOST_ONLY=1
for n in controller_server smoother_server planner_server behavior_server bt_navigator velocity_smoother; do
  st=$(timeout 4 ros2 lifecycle get /$n 2>/dev/null)
  if echo "$st" | grep -q unconfigured; then
    timeout 15 ros2 lifecycle set /$n configure >/dev/null 2>&1
  fi
  if ! echo "$st" | grep -q "active"; then
    timeout 15 ros2 lifecycle set /$n activate >/dev/null 2>&1
  fi
  echo "$n -> $(timeout 4 ros2 lifecycle get /$n 2>/dev/null)"
done
echo "全部 active 即救援成功;仍有 inactive/unconfigured 则重启整个 launch"
