# 底盘路径与稳定性并入说明

日期：2026-07-13
范围：`/home/lenovo/lab_cobot_ws/.worktrees/mecanum3-chassis-port`

## 一、对外说明只讲 3 件事

1. 跨工位移动改为显式的 `rotate -> forward -> strafe` 轴向阶段，不再允许斜着切过去。
2. `PICK` 完成后先 `retreat + go_home`，再开始跨工位移动，降低“机械臂一动全场乱闪/崩溃”的触发概率。
3. 麦轮可视化参数与实际运行链路统一，修正轮径/位移观感不一致。

## 二、本次修改内容

### 1. 跨工位移动由自由斜切改为轴向阶段运动

修改文件：
- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_navigation/lab_cobot_navigation/waypoints.py`
- `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`
- `src/lab_cobot_navigation/test/test_waypoints.py`

主要处理：
- 将 `station_b` 的 waypoint 从 `y=0.45` 调整到 `y=0.62`，与 `station_a` 共用同一条工位前方通道线。
- 在 `station_a`、`station_b` 之间的跨工位导航中，不再允许底盘自由混合 `x/y` 形成一条斜线。
- 新增显式轴向阶段控制：
  - `rotate`
  - `forward`
  - `strafe`
- 对应逻辑集中在：
  - `axis_aligned_navigation_goals()`
  - `axis_aligned_velocity_for_goal()`
  - `_navigate_axis_aligned()`
  - `_drive_axis_aligned_goal()`

预期效果：
- 跨工位时先朝向目标姿态，再沿通道方向移动，再做横向平移。
- 视觉上不再出现“从一个工位直接斜着飘到另一个工位”的效果。

### 2. 抓取完成后先退让再收臂，降低场景闪烁/崩溃概率

修改文件：
- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_bringup/test/test_mission_retreat.py`

主要处理：
- 将 `PICK` 成功后的收尾流程，从原来的仅 `retreat`，改为：
  - `retreat`
  - `go_home`
- 让底盘跨工位前，机械臂先回到稳定的收拢姿态。

预期效果：
- 降低到达另一个工位后，机械臂一启动就引发整车、机械臂、约束模型乱闪或崩溃的概率。
- 把底盘转运与机械臂大姿态切换分离开，减少仿真不稳定触发点。

### 3. 轮径与可视化几何参数统一

修改文件：
- `src/lab_cobot_bringup/lab_cobot_bringup/mecanum_wheel_visualizer.py`
- `src/lab_cobot_bringup/test/test_mecanum_wheel_visualizer.py`

主要处理：
- 将 `mecanum_wheel_visualizer.py` 中的参数统一到当前运行链路：
  - `wheel_radius = 0.07`
  - `wheel_separation_width = 0.24`
  - `wheel_separation_length = 0.175`
  - `wheelbase_radius = 0.415`
- 保持与以下实现一致：
  - `rover_twist_relay.py`
  - `mecanum_gazebo_kinematic_drive.cpp`
  - `mecanum3` 当前碰撞与运动学配置

预期效果：
- 轮子旋转观感与底盘位移更一致。
- “轮子转了一圈但车身位移看起来不对”的视觉偏差会减小。

## 三、验证结果

已执行回归：

```bash
cd /home/lenovo/lab_cobot_ws/.worktrees/mecanum3-chassis-port
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q   src/lab_cobot_bringup/test/test_mission_retreat.py   src/lab_cobot_bringup/test/test_mission_navigation_handoff.py   src/lab_cobot_navigation/test/test_waypoints.py   src/lab_cobot_bringup/test/test_mecanum_wheel_visualizer.py
```

结果：`85 passed`

说明：
- 逻辑层回归已通过。
- 剩余风险主要在 GUI/Gazebo 真实仿真观感与物理稳定性，尚需人工烟测确认。

## 四、并入骨干时的主要风险

### 风险 1：真实仿真里仍可能存在局部视觉不一致
影响：
- 单元测试已经锁住了轴向阶段逻辑，但 Gazebo 中若还有其他节点对底盘发混合速度，仍可能看到不完全符合预期的运动观感。

建议：
- 并入后优先做一次 A -> B 的人工烟测，确认中段已呈现明显的阶段化移动，而不是继续斜切。

### 风险 2：场景闪烁/崩溃问题目前是通过流程降风险，不是物理引擎根因修复
影响：
- `retreat + go_home` 会降低触发概率，但如果碰撞参数、约束关系或放置姿态本身仍偏激，极端情况下仍可能复现。

建议：
- 队长审核时把这项视为“稳定性缓解补丁”，并要求后续补做一次仿真根因排查。

### 风险 3：waypoint 共线会改变旧演示脚本或截图口径
影响：
- 任何仍基于 `station_b.y = 0.45` 的说明、截图、测试预期，都需要同步。

建议：
- 并入后检索 `station_b` 的相关文档与演示材料，统一口径。

## 五、建议并入范围

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/mecanum_wheel_visualizer.py`
- `src/lab_cobot_navigation/lab_cobot_navigation/waypoints.py`
- `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`
- `src/lab_cobot_bringup/test/test_mission_retreat.py`
- `src/lab_cobot_bringup/test/test_mecanum_wheel_visualizer.py`
- `src/lab_cobot_navigation/test/test_waypoints.py`
- `docs/superpowers/specs/2026-07-13-chassis-path-wheel-alignment-merge-note.md`

## 六、结论

本次修改适合并入“底盘运动与工位转运稳定性”这一大项补骨干，原因是：

- 它解决的是任务链路里的真实演示问题，而不是纯代码整理。
- 改动范围集中，外部说明可以稳定收敛为 3 个点。
- 已有针对性回归通过，具备进入主干审核的基础。
