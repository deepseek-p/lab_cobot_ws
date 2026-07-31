# 工业机械臂抓取与放置轨迹方案

## 目标

本方案用于 ROS 2 Humble、MoveIt 2、Gazebo 机械臂抓取仿真，目标是稳定完成“把样件从 A 送到 B”，并减少无意义的腕部或肩部绕圈。

核心原则是：不要让 RRTConnect 负责所有动作。RRTConnect 只负责远距离、需要全局避障的运动；靠近物体、抓取、抬升、放置下降、放置抬升等局部动作使用 Cartesian Path。

## 本次最终有效结论

这次彻底排查后确认，B 点放置多余动作不是单一原因造成的，而是几个问题叠加：

1. `move_to_observe()` 曾使用 `ur_wrist_2_joint = 4.581185`，这是 `-1.701999 + 2π` 的等价分支，会让后续规划更容易选择绕圈关节解。
2. `pymoveit2.move_to_pose(cartesian=True)` 在 `execute_via_moveit=True` 时原来没有真正走 Cartesian Path，短距离动作仍可能被 MoveGroup/OMPL 规划。
3. B 点放置下降前，OMPL approach 到达的实际 TCP 姿态和下一步强制指定的 `DOWN_QUAT` 不一致，Cartesian 下降被迫一边下降一边修正姿态，导致 IK 连续路径被截断。
4. 触觉夹持时 MoveIt planning scene 里的附着样件盒会在 B 点释放高度附近和台面碰撞盒共同影响 Cartesian Path，导致路径 fraction 不完整。

最终有效修复是：

- 观察位姿改到同一腕部分支，不再使用 `+2π` 腕部角。
- 短距离 Cartesian 动作真正调用 `compute_cartesian_path`。
- B 点放置下降和抬升继承当前 `gripper_tcp` 姿态，只做竖直位移，不再在局部段重新修正朝向。
- 触觉放置下降前，只从 MoveIt planning scene 中移除保守的附着样件盒；真实 Gazebo 样件仍由夹爪插件保持，直到 release。
- B 点下降和抬升禁止退回 OMPL，避免局部放置阶段再次绕圈。

## 2026-07-31 追加排查和修复

今天连续运行三次任务后，现象变成：

- 第一次看起来较正常。
- 第二次、第三次在 A 点夹住样件后又出现多余绕圈。
- 下降夹取物块前能看到明显停顿。

重新对比当天三次 ROS 日志后确认：

1. A 点下降本身不是主要问题，`compute_cartesian_path` 都能完成 `100%`。
2. 多余绕圈主要发生在 A 点抓取成功后的抬升阶段。
3. 第二次和第三次日志里，A 点抬升 Cartesian Path 只完成约 `75% ~ 86%`，随后代码自动退回 MoveGroup/OMPL fallback。
4. 这次 fallback 又调用了 RRTConnect，RRTConnect 会重新采样 IK 分支，因此可能让腕部或肩部多转一圈。
5. 下降夹取前的明显停顿，主要来自 A 点预抓取 approach 的 OMPL 轨迹执行时间变长，不是夹爪下降 Cartesian 卡死。

本次继续修改后，A 点抓取后的局部抬升不再允许 OMPL fallback：

- 抓取下降后，先读取当前 `gripper_tcp` 姿态。
- 抬升时继承当前 TCP 姿态，只做竖直 Cartesian Path。
- 预抓取高度仍保持 9cm，保证夹爪下降前有安全空间。
- 抓住后的第一段抬升单独改为 6.5cm，不再复用 9cm 预抓取高度。
- 如果触觉夹持模式下，MoveIt 中的 `carried_sample` 附着碰撞盒导致 Cartesian 抬升被截断，则只临时从 MoveIt planning scene 移除该保守盒。
- 真实 Gazebo 样件仍由夹爪接触/附着插件保持，不会因为移除 MoveIt 碰撞盒而松开。
- 直线抬升成功后，再把 `carried_sample` 附着盒重新挂回 MoveIt，用于后续长距离搬运避障。

B 点也做了一个稳定性修正：

- B 点下降前读取一次当前 TCP 姿态用于下降。
- 释放并打开夹爪后，再读取一次当前 TCP 姿态用于抬升。
- 这样下降过程中产生的极小姿态误差不会在抬升阶段被强行修正，减少 B 点 Cartesian Path 被截断的概率。

本次修改后的直接效果：

- A 点抓取后抬升不再由 RRTConnect 接管，消除“夹住后又重新规划绕圈”的根因。
- A 点抬升从 9cm 改为 6.5cm，避开当天日志中稳定卡在 81% 的不可达段。
- A 点抬升如果受保守附着碰撞盒影响，会改走“移除规划碰撞盒后继续 Cartesian 直线抬升”的稳定路径。
- B 点释放后的抬升继续保持纯 Cartesian，不再因为姿态继承错误导致偶发失败。
- 远距离运动仍保留 OMPL RRTConnect 和碰撞检测。
- 局部接近、抓取、抬升、放置下降、放置抬升都不让 RRTConnect 兜底。

本次最终实跑验证结果：

- 启动时间：2026-07-31 12:07:24。
- 运行命令：`ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false`。
- 任务命令：`timeout 4 ros2 topic pub -r 2 /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"`。
- A 点预抓取：仍由 RRTConnect 负责远距离 approach。
- A 点下降：Cartesian Path `100%`。
- A 点抓取后抬升：目标高度从 `z=0.741` 改为 `z=0.716`，Cartesian Path `100%`。
- B 点放置下降：Cartesian Path `100%`。
- B 点放置抬升：Cartesian Path `100%`。
- 本次日志中未出现 `retrying with MoveGroup fallback`。
- 样件从 A 成功送到 B，最终状态：`任务结束: DONE`。

仍然需要知道的现象：

- A 点下降前仍能看到短暂停顿，日志显示它主要来自 A 点预抓取 approach 的 RRTConnect 轨迹执行时间，而不是下降 Cartesian 卡顿。
- 这段属于远距离接近段，按当前方案仍保留 OMPL 和碰撞检测；如果后续还想压缩这个等待时间，可以继续把 A 点 approach 前增加一个固定中间关节位，减少 RRTConnect 的随机路径长度。

## 修改文件

### `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`

主要修改：

- `OBSERVE_CONFIG` 中 `ur_wrist_2_joint` 从 `4.581185` 改为 `-1.701999`。
- 新增当前关节读取、腕部路径约束、轨迹角度归一化逻辑，减少等价关节解导致的绕圈。
- `_move()` 改为先 `plan()`，再对轨迹做最近角度分支归一化，最后执行。
- 局部 Cartesian 段降低速度和加速度，减少抓取抖动。
- 抓取前后增加短暂停顿，给夹爪接触和附着状态留出稳定时间。
- B 点放置下降和抬升使用 FK 读取到的当前 `gripper_tcp` 姿态，不再强行切回 `DOWN_QUAT`。
- 触觉放置在下降前移除 MoveIt 中的 `carried_sample` 附着碰撞盒，避免释放附近的保守碰撞截断 Cartesian Path。
- B 点放置下降、放置抬升设置 `fallback_to_ompl=False`，防止局部动作失败后被 RRTConnect 接管并产生多余绕圈。

### `src/pymoveit2/pymoveit2/moveit2.py`

主要修改：

- 修复 `cartesian=True` 在 `execute_via_moveit=True` 下没有真正走 Cartesian Path 的问题。
- Cartesian 规划现在调用 `compute_cartesian_path`。
- Cartesian 轨迹可以通过 MoveIt `ExecuteTrajectory` 执行。
- 对缺少时间戳或时间不递增的 Cartesian 轨迹补安全时间，避免控制器拒绝。

### `src/lab_cobot_moveit/config/joint_limits.yaml`

主要修改：

- 对 `ur_wrist_3_joint` 增加更明确的位置限制，抑制末端法兰自旋。
- 没有强行收窄 `ur_wrist_1_joint`、`ur_wrist_2_joint`，避免破坏已有可达姿态。

### `src/lab_cobot_moveit/config/ompl_planning.yaml`

主要修改：

- 显式指定 `ur_manipulator.default_planner_config: RRTConnectkConfigDefault`。
- RRTConnect 保留给远距离运动，不再隐式承担所有局部动作。

### `src/lab_cobot_manipulation/lab_cobot_manipulation/scene_obstacles.py`

主要修改：

- 保留台面碰撞盒，用于 approach 和局部动作的碰撞检测。
- 保留持物样件附着盒用于长距离搬运避障。
- 放置下降前由 `pick_place_node.py` 临时移除 MoveIt 附着盒，避免释放高度附近出现保守碰撞冲突。

### `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`

主要修改：

- 保留 A 到 B 的任务流程。
- 配合分段抓取、导航、放置动作执行。

### 测试文件

已更新和新增测试：

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
- `src/pymoveit2/test/test_moveit2_cartesian.py`
- `src/lab_cobot_moveit/test/test_moveit_config.py`
- `src/lab_cobot_manipulation/test/test_scene_obstacles.py`
- `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`
- `src/lab_cobot_manipulation/test/test_gripper_driver.py`

测试覆盖内容：

- Cartesian 分段是否真正走 Cartesian Path。
- B 点放置下降和抬升是否禁止 OMPL fallback。
- B 点放置局部段是否继承当前 TCP 姿态。
- 腕部路径约束是否设置和清理。
- 轨迹角度是否归一化到最近等价分支。
- 触觉放置是否在下降前移除 MoveIt 附着样件盒。

## 当前运动流程

```text
Home
  |
  | 关节目标
  v
观察/预备姿态
  |
  | 导航到 A 点
  v
A 点检测样件
  |
  | OMPL RRTConnect
  | 保留碰撞检测
  | 末端目标: gripper_tcp
  v
A 点预抓取位置
  |
  | Cartesian Path
  | 竖直下降
  | gripper_tcp 朝下
  v
抓取
  |
  | 夹爪闭合
  | 等待接触/附着反馈
  v
A 点抬升
  |
  | 优先 Cartesian Path
  | 继承当前 TCP 姿态
  | 触觉模式第一段抬升 6.5cm
  | 禁止 OMPL fallback
  | 触觉模式下若附着碰撞盒截断路径,临时移除 MoveIt 碰撞盒后重试 Cartesian
  v
导航到 B 点
  |
  | OMPL RRTConnect 到 B 点预放置
  | 保留台面碰撞检测
  | 保留腕部路径约束
  v
B 点预放置位置
  |
  | 读取当前 gripper_tcp 姿态
  | 移除 MoveIt 中的 carried_sample 保守附着盒
  v
B 点放置下降
  |
  | Cartesian Path
  | 继承当前 TCP 姿态
  | 只做竖直下降
  | 禁止 OMPL fallback
  v
释放样件
  |
  | 夹爪 release
  | 打开夹爪
  v
B 点放置抬升
  |
  | Cartesian Path
  | 继承同一 TCP 姿态
  | 只做竖直抬升
  | 禁止 OMPL fallback
  v
返回 Home
```

## 最终验证结果

验证命令：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false
ros2 topic pub -r 2 /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
```

本次完整运行结果：

- A 点检测、抓取、夹爪触觉附着成功。
- B 点 approach 第一次可能因为采样失败重试，但第二次成功到达预放置点。
- B 点放置下降：`compute_cartesian_path` 完成 `100%`。
- B 点放置抬升：`compute_cartesian_path` 完成 `100%`。
- B 点下降和抬升没有退回 OMPL。
- 日志出现 `Place complete`。
- 任务最终输出：`任务结束: DONE`。
- 样件从 A 点成功送到 B 点。
- 2026-07-31 最终实跑中，A 点抓取后抬升也达到 Cartesian `100%`，没有再退回 OMPL。

补充说明：

- 关闭仿真时 MoveIt/Gazebo 仍可能打印 `move_group` 退出段错误和 `rcl_shutdown already called`，这是 shutdown 阶段日志，不影响任务已经 `DONE` 的结论。
- 2026-07-31 追加修复后，A 点抬升也不再允许显式退回 OMPL，并且抬升高度改成已实跑验证的 6.5cm；若再看到绕圈，下一步应重点查看 A 点预抓取 approach 的 OMPL 远距离轨迹，而不是抓取下降或抓取抬升局部段。

## 已验证命令

```bash
source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/lab_cobot_manipulation:src/pymoveit2:$PYTHONPATH" /usr/bin/python3 -m pytest -q src/lab_cobot_manipulation/test/test_pick_place_sequence.py src/pymoveit2/test/test_moveit2_cartesian.py
```

结果：

```text
59 passed
```

```bash
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

结果：

```text
8 packages finished
```
