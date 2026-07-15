# mecanum3 导航分支提审计划

日期：2026-07-13
工作树：`/home/lenovo/lab_cobot_ws/.worktrees/mecanum3-chassis-port`
当前基础分支：`feature/mecanum3-chassis-port`
建议提审分支：`feature/navigation-axis-transfer-stability`
远端仓库：`origin -> https://github.com/deepseek-p/lab_cobot_ws.git`

## 1. 文档目的

本文件用于记录本次 `mecanum3` 底盘与跨工位导航修复的提审计划，供以下场景统一使用：

1. 你本人在本地整理本次工作内容。
2. 你新建独立导航分支并上传到 GitHub。
3. 队长在 GitHub 上审核这次导航稳定性补丁。
4. 后续回溯这几天的改动脉络、验证结果和提审依据。

配套记录文档：
- `docs/superpowers/specs/2026-07-11-mecanum3-chassis-port-design.md`
- `docs/superpowers/specs/2026-07-12-original-bringup-mecanum-motion-design.md`
- `docs/superpowers/specs/2026-07-13-chassis-visual-safety-flow-design.md`
- `docs/superpowers/specs/2026-07-13-chassis-path-wheel-alignment-merge-note.md`

## 2. 本次对外只讲的 3 件事

1. 跨工位移动改为显式的 `rotate -> forward -> strafe` 轴向阶段，不再允许斜着切过去。
2. `PICK` 完成后先 `retreat + go_home`，再开始跨工位移动，降低“机械臂一动全场乱闪/崩溃”的触发概率。
3. 麦轮可视化参数与实际运行链路统一，修正轮径/位移观感不一致。

## 3. 本次准备提审的文件范围

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/mecanum_wheel_visualizer.py`
- `src/lab_cobot_navigation/lab_cobot_navigation/waypoints.py`
- `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`
- `src/lab_cobot_bringup/test/test_mission_retreat.py`
- `src/lab_cobot_bringup/test/test_mecanum_wheel_visualizer.py`
- `src/lab_cobot_navigation/test/test_waypoints.py`
- `docs/superpowers/specs/2026-07-13-chassis-path-wheel-alignment-merge-note.md`
- `docs/superpowers/specs/2026-07-13-mecanum3-mainline-merge-plan.md`

## 4. 当前验证结果

已执行回归：

```bash
cd /home/lenovo/lab_cobot_ws/.worktrees/mecanum3-chassis-port
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q   src/lab_cobot_bringup/test/test_mission_retreat.py   src/lab_cobot_bringup/test/test_mission_navigation_handoff.py   src/lab_cobot_navigation/test/test_waypoints.py   src/lab_cobot_bringup/test/test_mecanum_wheel_visualizer.py
```

当前结果：`85 passed`

说明：
- 逻辑层回归是通过的。
- 当前剩余风险主要在 GUI/Gazebo 真实仿真观感与物理稳定性，而不是纯逻辑单元测试。

## 5. 新导航分支的建议流程

### 5.1 建分支

```bash
cd /home/lenovo/lab_cobot_ws/.worktrees/mecanum3-chassis-port
git switch -c feature/navigation-axis-transfer-stability
```

### 5.2 提交本地改动

```bash
git add   src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py   src/lab_cobot_bringup/lab_cobot_bringup/mecanum_wheel_visualizer.py   src/lab_cobot_navigation/lab_cobot_navigation/waypoints.py   src/lab_cobot_bringup/test/test_mission_navigation_handoff.py   src/lab_cobot_bringup/test/test_mission_retreat.py   src/lab_cobot_bringup/test/test_mecanum_wheel_visualizer.py   src/lab_cobot_navigation/test/test_waypoints.py   docs/superpowers/specs/2026-07-13-chassis-path-wheel-alignment-merge-note.md   docs/superpowers/specs/2026-07-13-mecanum3-mainline-merge-plan.md

git commit -m "feat(navigation): stabilize mecanum3 station transfer"
```

### 5.3 上传到 GitHub

```bash
git push -u origin feature/navigation-axis-transfer-stability
```

### 5.4 交给队长审核

建议 PR 标题：
`Stabilize mecanum3 cross-station transfer and post-pick arm state`

建议 PR 摘要只写 4 段：
- Problem
- Change
- Validation
- Risk

## 6. 当前结论

这次改动已经适合整理成独立导航分支上传给队长审核：

- 对外主线清晰，只讲 3 件事。
- 代码、测试、说明文档已经收敛到同一口径。
- 逻辑层回归通过。
- 剩余风险明确，重点是 GUI/Gazebo 烟测，而不是代码契约缺失。
