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

## 2026-07-31 再次修复记录

这次问题现象：

- 第一次任务正常，第二次、第三次抓取前后又出现多余绕圈。
- 下降夹取物块前有明显停顿。
- 放置物块后停顿时间偏长。

重新查看运行日志后确认，问题不是 GitHub 上传导致的。当前本地和远端 `feature/grasp` 都在同一个提交上；更关键的是，历史 launch 日志里出现过从旧工作空间加载资源：

- `/home/zww/projects/lab_cobot_ws_G2/install/...`
- `/home/zww/projects/lab_cobot_ws_navigation/install/...`

因此运行前必须确认当前终端只加载 `/home/zww/projects/lab_cobot_ws/install/...`，否则会出现“代码已经改了，但仿真表现还是旧问题”的情况。

本次代码层面的根因：

1. 触觉抓取的 `y` 方向目标被强行限制在 `base_link` 中线附近 `±0.018m`。日志里曾出现物块检测位置 `y=-0.395m`，但抓取目标被改成 `y=-0.018m`，实际偏差接近 `38cm`，夹爪接触插件自然拒绝附着，随后触发多轮侧向重试。
2. 局部 Cartesian 段禁止 OMPL 兜底后，如果执行失败仍会进入 `MOVE_RESULT_GRACE_SEC=5s` 的额外等待。B 点放置下降或释放后抬升失败时，最多会多等两次，表现为放置后长时间不动。
3. 夹爪打开和夹爪闭合共用 `GRIPPER_CLOSE_SETTLE_SEC=1.0s`。闭合需要等待接触稳定，但打开夹爪不需要等这么久。

本次修改内容：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - 删除触觉抓取对绝对 `y` 位置的硬夹紧。
  - 触觉抓取现在保留视觉检测到的物块 `y` 坐标，只叠加毫米级抓取偏置。
  - 侧向重试改为围绕当前检测目标做小范围偏移，不再围绕 `base_link` 的 `y=0` 搜索。
  - 局部 Cartesian 且 `fallback_to_ompl=False` 的动作失败后不再额外等待 5 秒。
  - 新增 `GRIPPER_OPEN_SETTLE_SEC=0.2s`，打开夹爪不再复用闭合等待时间。

- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`
  - 增加 `open_settle_sec` 参数。
  - `open()` 使用打开等待时间。
  - `close()` 和触觉闭合仍使用原来的闭合等待/触觉步进逻辑。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 更新触觉抓取测试，保护“围绕视觉检测点抓取”的新行为。
  - 增加打开等待小于闭合等待的断言。

- `README.md`
  - 将旧工作空间路径 `~/projects/lab_cobot_ws_g4g5` 改成当前路径 `~/projects/lab_cobot_ws`，避免按文档启动时误加载旧 install。

本次形成的效果：

- A 点抓取不会再把物块 `y` 坐标夹到 `base_link` 中线附近，能明显减少“夹爪去错位置、接触拒绝、重复靠近”的多余动作。
- 触觉侧向重试只在物块真实位置附近做毫米级搜索，不会跨大距离乱试。
- B 点释放后的 Cartesian 抬升如果失败，会快速返回并继续已确认释放后的流程，不再因为每次失败额外等待 5 秒而长时间卡住。
- 打开夹爪等待从 1.0 秒降到 0.2 秒，放置后停顿会更短。
- 远距离移动仍使用 OMPL RRTConnect，局部抓取/下降/抬升仍保持 Cartesian Path，整体结构没有大范围重构。

本次验证命令：

```bash
source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/lab_cobot_manipulation:src/pymoveit2:$PYTHONPATH" /usr/bin/python3 -m pytest -q src/lab_cobot_manipulation/test/test_pick_place_sequence.py src/lab_cobot_manipulation/test/test_gripper_driver.py
```

结果：

```text
100 passed
```

运行前环境确认命令：

```bash
cd /home/zww/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg prefix lab_cobot_gazebo
ros2 pkg prefix lab_cobot_description
ros2 pkg prefix lab_cobot_manipulation
ros2 pkg prefix lab_cobot_bringup
ros2 pkg prefix lab_cobot_moveit
```

这几个输出都必须是 `/home/zww/projects/lab_cobot_ws/install/...`。如果出现 `lab_cobot_ws_G2`、`lab_cobot_ws_navigation` 或其他旧目录，就说明当前终端环境仍然混了旧工作空间，需要重新开终端后重新 source。

## 2026-07-31 防掉落和平稳放置修复

这次问题现象：

- 物品在搬运或放置过程中出现掉落。
- 希望 B 点放置更平稳，不要从较高位置自由落下。
- 希望放置时机械臂可以适当再下降一点。

日志确认：

- 最新运行已经正确加载当前工作空间 `/home/zww/projects/lab_cobot_ws/install/...`。
- 抓取阶段日志显示 `夹爪触觉闭合已附着 aruco_sample`，插件确认过附着。
- B 点释放前没有看到 `lost` 或 `持有监控失败`，说明代码没有在中途主动 release。
- B 点第一次下降到 `z=0.725` 时曾出现 `Cartesian path incomplete (fraction: 0.9407)`，随后 mission 又重试了一次 place；这会让放置动作看起来不够连贯。

本次修改内容：

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
  - 将默认 B 点放置 TCP 高度从 `0.725` 降到 `0.705`。
  - 目的：释放前让夹爪和物块再靠近台面约 `2cm`，减少自由落体高度。

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - 触觉模式默认速度从 `0.30/0.30` 降到 `0.24/0.20`。
  - 局部 Cartesian 速度从 `0.18/0.18` 降到 `0.12/0.12`。
  - B 点持物 approach 开启低速执行，减少持物时的甩动。
  - 新增 `TACTILE_PLACE_DESCENT_STEP`,后续已从 `0.02` 收紧到 `0.01`。
  - 触觉放置下降改为分段 Cartesian 小步下探，每段最多约 `2cm`，不再一次性从预放置高度降到底。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 更新触觉放置测试，确认 B 点下降会经过中间点再到释放点。
  - 更新触觉速度测试，保护更低的速度和加速度设置。

- `src/lab_cobot_bringup/test/test_mission_place_pose.py`
  - 更新默认放置高度测试。
  - 放置落差上限从 `8cm` 收紧到 `7cm`，要求更平稳。

本次形成的效果：

- 持物移动更慢，减少由于加速度过大导致的物块晃动或脱落风险。
- B 点放置会先到预放置点，再分段直线下降，不再一次下探到底。
- 默认释放高度降低约 `2cm`，物块更接近台面后再松爪，落下更平稳。
- 局部动作仍然使用 Cartesian Path，远距离移动仍然保留 OMPL RRTConnect 和碰撞检测。

本次验证命令：

```bash
source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/lab_cobot_manipulation:src/pymoveit2:src/lab_cobot_bringup:src/lab_cobot_navigation:$PYTHONPATH" /usr/bin/python3 -m pytest -q src/lab_cobot_manipulation/test/test_pick_place_sequence.py src/lab_cobot_manipulation/test/test_gripper_driver.py src/lab_cobot_bringup/test/test_mission_place_pose.py
```

结果：

```text
102 passed
```

## 2026-07-31 B 点提前松爪和停靠距离修复

这次问题现象：

- 到 B 工位放置时，机械臂还没真正到低放置位就松开了物块。
- B 点放置位置看起来离 B 工位还有一点距离。

日志确认：

- B 点本地停靠完成时，日志显示 `place_map=(-2.193,1.406)`，落点在 B 台面前侧，离台面中心偏前。
- 触觉放置第一段下降到 `z=0.725` 时，Cartesian Path 多次只完成约 `94%`。
- `place()` 因下降失败返回 `False` 后，mission 的失败兜底清理会调用 `release_object()`，这就是“没到位提前松爪”的直接原因。

本次修改内容：

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
  - B 点本地放置停靠目标从 `PLACE_BASE_TARGET_POSE=(-2.0,0.62,90deg)` 改为 `(-2.0,0.72,90deg)`。
  - 这样默认 TCP 放置点投影会从台面前侧约 `map_y=1.40` 推到约 `map_y=1.54`，更接近 B 工位台面中间。
  - mission 记录最后失败状态。
  - 如果失败发生在 `PLACE` 阶段，并且持有监控仍确认物块还在夹爪上，兜底清理不再调用 `release_object()`，避免下降失败后提前松爪。

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - 触觉放置下降步长从 `2cm` 改为 `1cm`。
  - 目的：每段 Cartesian 更短，降低 B 点下降被 IK/碰撞截断的概率，同时放置动作更平稳。

- `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`
  - 更新 B 点本地停靠测试，新的完成位姿为 `y≈0.72`。

- `src/lab_cobot_bringup/test/test_mission_retreat.py`
  - 新增回归测试：PLACE 失败但物块仍被持有时，失败清理不得 release。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 更新触觉放置下降测试，确认释放前会走完多个 `1cm` 小步。

本次形成的效果：

- 放置下降失败时不会再由任务失败兜底提前松爪。
- B 点停靠更靠近工位，默认放置点更接近 B 台面中部。
- 触觉放置下降更细、更慢、更稳，释放动作发生在完成小步下探之后。

本次验证命令：

```bash
source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/lab_cobot_manipulation:src/pymoveit2:src/lab_cobot_bringup:src/lab_cobot_navigation:$PYTHONPATH" /usr/bin/python3 -m pytest -q src/lab_cobot_manipulation/test/test_pick_place_sequence.py src/lab_cobot_manipulation/test/test_gripper_driver.py src/lab_cobot_bringup/test/test_mission_place_pose.py src/lab_cobot_bringup/test/test_mission_navigation_handoff.py src/lab_cobot_bringup/test/test_mission_retreat.py
```

结果：

```text
164 passed
```

## 2026-07-31 B 点下降失败导致整单失败的最终修复

这次问题现象：

- A 地取东西已经正常。
- 到 B 工位后，机械臂到达预放置高度 `z=0.745`。
- 第一段向下 `1cm` 到 `z=0.735` 时，MoveIt 日志显示 Cartesian Path 只完成约 `92%~93%`。
- 因为之前代码要求这一段必须完整成功，所以 `place()` 直接返回失败，任务状态停在 `FAILED`。

根本原因：

- B 工位携物下降处在台面碰撞盒、持物碰撞盒、末端姿态约束和当前 IK 分支共同限制的边界区域。
- 这里继续强制每一厘米都必须笛卡尔完整可解，会把一个局部可恢复问题放大成任务失败。
- 如果退回 OMPL/RRTConnect，又会重新选 IK 分支，容易重新引入腕部或肩部绕圈，所以不能让 RRTConnect 接管贴近台面的下降动作。

本次修改内容：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - `_move_place_descent()` 从只返回成功/失败，改为返回“是否可继续”和“最后一个已确认稳定的释放位置”。
  - 触觉放置下降仍然优先按 `1cm` 小步执行 Cartesian Path。
  - 如果某个下降小步被 MoveIt 判定为不完整，不再触发整单失败，也不退回 OMPL；改为在最后一个已成功到达的位置释放物块。
  - 如果最后稳定位置就是预放置高度，则释放后跳过多余的 Cartesian 抬升，避免原地再规划一次造成停顿。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 将原来的“触觉放置下降失败就不能松爪”测试，改为“触觉放置下降受阻时，在稳定预放置位释放并完成任务”。
  - 保留非触觉模式下降失败仍失败，避免普通夹爪路径误判成功。

本次形成的效果：

- B 点不会因为第一段 `1cm` 下降 Cartesian fraction 不足而整单失败。
- 局部下降失败时不会退回 RRTConnect，因此不会因为重新选 IK 分支产生额外绕圈。
- 放置释放发生在 MoveIt 已确认到达的稳定位姿，不再由 mission 失败清理流程提前松爪。
- 已经在预放置高度释放时，释放后不再做无意义抬升，减少放置后的长时间停顿。

## 2026-07-31 执行等待抢跑根因修复和完整跑通记录

这次继续验证时发现的新问题：

- 第一次修复后任务可以到 `DONE`，但日志显示 B 点放置 approach 的 `execute_trajectory` 还没真正完成，后面的下降和 release 已经开始执行。
- 随后 MoveIt 把旧轨迹报告为 `PREEMPTED`。
- 这会让机械臂动作看起来“不顺”“多余”，也是抓取/放置前后偶发乱动的重要根因。

真正根因：

- 本工程 `PickPlace._plan_execute_pose()` 先调用 `moveit2.plan()`，再调用 `moveit2.execute(trajectory, via_moveit=True)`。
- 这个路径实际走的是 MoveIt 的 `execute_trajectory` action。
- 但是本地 `pymoveit2` 原来没有保存 `execute_trajectory` 的 send/result future。
- `pick_place_node.py` 的等待函数只监控了 `move_action` 和 `follow_joint_trajectory`，因此可能读到上一条动作的完成状态，误以为当前动作已经结束。
- 结果就是下一段 Cartesian 下降或松爪提前启动，造成动作抢跑、旧轨迹被 preempt、任务表现不稳定。

本次修改内容：

- `src/pymoveit2/pymoveit2/moveit2.py`
  - 在 `_send_goal_async_execute_trajectory()` 中保存：
    - `__send_goal_future_execute_trajectory`
    - `__get_result_future_execute_trajectory`
  - 这样上层可以可靠等待当前 `execute_trajectory` 的真实执行结果。

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - `_wait_for_moveit_result()` 增加 `action_future_names` 参数。
  - 等待顺序增加 `execute_trajectory`。
  - `plan()+execute(via_moveit=True)` 分支明确只等待本次 `execute_trajectory`，不再被旧 `move_action` 或旧 controller future 干扰。
  - `execute_trajectory` 的成功判定同时检查 ROS action status 和 MoveIt `error_code`，避免 action 返回但 MoveIt 执行失败时被误判成功。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 新增测试：`execute_trajectory` result future 完成时等待成功。
  - 新增测试：`execute_trajectory` 的 MoveIt `error_code=-7` 时等待失败。

本次形成的效果：

- 每段机械臂动作都会等当前 MoveIt 执行真正结束后，才进入下一阶段。
- B 点不会再出现 approach 未完成就开始下降/松爪的动作抢跑。
- 局部 Cartesian 小步下降可以连续稳定执行到 `z=0.705`，再释放物块。
- 放置后直线上抬也能等到真正完成，再继续返回 home。
- 整体任务从 A 抓取、搬运到 B、放置、返回 home 已完整跑通。

本次验证命令：

```bash
source /opt/ros/humble/setup.bash && PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 PYTHONPATH="src/lab_cobot_manipulation:src/pymoveit2:src/lab_cobot_bringup:src/lab_cobot_navigation:$PYTHONPATH" /usr/bin/python3 -m pytest -q src/lab_cobot_manipulation/test/test_pick_place_sequence.py src/lab_cobot_manipulation/test/test_gripper_driver.py src/lab_cobot_bringup/test/test_mission_place_pose.py src/lab_cobot_bringup/test/test_mission_navigation_handoff.py src/lab_cobot_bringup/test/test_mission_retreat.py
```

结果：

```text
166 passed
```

构建命令：

```bash
source /opt/ros/humble/setup.bash && colcon build --symlink-install
```

结果：

```text
8 packages finished
```

完整仿真验证：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false
ros2 topic pub -r 2 /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
```

关键日志：

```text
Place start target=(0.820, 0.200, 0.705) release=(0.820, 0.200, 0.705) approach=(0.820, 0.200, 0.745)
MoveIt target ... pos=(0.820, 0.200, 0.745)
Goal reached, success!
Execution completed: SUCCEEDED
MoveIt target ... pos=(0.820, 0.200, 0.735)
Computed Cartesian path ... followed 100.000000%
MoveIt target ... pos=(0.820, 0.200, 0.725)
Computed Cartesian path ... followed 100.000000%
MoveIt target ... pos=(0.820, 0.200, 0.715)
Computed Cartesian path ... followed 100.000000%
MoveIt target ... pos=(0.820, 0.200, 0.705)
Computed Cartesian path ... followed 100.000000%
夹爪 contact release accepted
MoveIt target ... pos=(0.820, 0.200, 0.745)
Computed Cartesian path ... followed 100.000000%
Place complete
G4G5_RESULT task_status=DONE g4_touch_ok=True g4_last_contact='released aruco_sample'
任务结束: DONE
```

说明：

- 关闭 launch 时出现的 `KeyboardInterrupt`、`move_group` shutdown 段错误，是任务完成后手动 `Ctrl-C` 结束仿真产生的退出日志，不是任务执行失败。

## 2026-08-02 第二轮 Gazebo A 到 B 实测修正

本轮实测现象：

- 第一次完整 A 到 B 可以完成，但抓取阶段第一次夹取会失败一次。
- 失败日志显示第一次目标点为 `target=(0.829, 0.023, 0.651)`，检测点为 `detected=(0.829, 0.017, 0.633)`，也就是程序主动给 y 方向加了 6mm 偏置。
- 第二次重试使用接近原始检测 y 的目标后，夹爪立即双指接触并完成 attach。
- 因此夹取前的停顿和多余动作，主要来自这一次无意义的初始偏置导致的失败重试。

本次修改内容：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - 将 `TACTILE_PICK_LATERAL_BIAS` 从 `0.006` 改为 `0.0`。
  - 触觉抓取第一夹直接使用视觉/停靠得到的中心 y 坐标。
  - 只有在双指接触失败后，才根据单边触觉结果做 6mm 级别的横向重试。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 同步更新触觉抓取测试期望。
  - 测试现在表达的新策略是：第一次抓取不预偏置，失败后再按触觉方向微调。

本次验证结果：

```text
122 passed
```

构建结果：

```text
3 packages finished
```

第二次完整 Gazebo 验证关键日志：

```text
Pick start detected=(0.820, 0.001, 0.633) target=(0.820, 0.001, 0.651)
Computed Cartesian path ... followed 100.000000%
夹爪触觉闭合已附着 aruco_sample
夹爪 contact attach aruco_sample accepted
Pick complete
Place complete
G4G5_RESULT task_status=DONE g4_touch_ok=True g4_last_contact='released aruco_sample'
任务结束: DONE
```

本次形成的效果：

- A 点第一次夹取直接成功，没有再出现“闭合到上限仍未双指接触”的失败重试。
- 抓取下降、抓取后抬升、B 点下降、释放后抬升都保持 Cartesian 直线路径。
- attach 和 detach 均由 Gazebo 插件确认。
- 最终任务状态为 `DONE`，触觉结果为 `g4_touch_ok=True`。

当前仍观察到的非阻塞问题：

- Nav2/Gazebo 在仿真负载较高时仍会打印 `Control loop missed its desired rate` 和 `Behavior Tree tick rate was exceeded`，但本轮没有导致任务失败。
- 手动 `Ctrl-C` 关闭仿真时，MoveIt/Gazebo 仍可能打印 shutdown 阶段的栈信息；任务已经在关闭前完成。

## 2026-08-02 B 点平稳放置优化

本次目标：

- 不再降低 B 点释放高度，避免重新触发台面碰撞和约束冲突。
- 通过释放后的节奏控制，让物块落稳、夹爪张开稳定后，再让机械臂上抬。

本次修改内容：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - `TACTILE_PLACE_DROP_SETTLE_SEC` 从 `0.3s` 增加到 `0.6s`。
  - 新增 `TACTILE_PLACE_POST_OPEN_SETTLE_SEC = 0.25s`。
  - B 点触觉放置流程变为：
    - Cartesian 下降到释放位。
    - `release_object()` 等 Gazebo 插件确认 detach。
    - 等待 `0.6s`，让物块在台面上稳定。
    - 打开夹爪。
    - 再等待 `0.25s`，避免开爪瞬间就抬升造成接触扰动。
    - Cartesian 直线上抬。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 更新放置等待测试，保护“释放后沉降”和“开爪后沉降”两个等待点。

预期效果：

- B 点放置会比之前慢约 `0.55s`，但物块落下和夹爪打开更稳。
- 不改变 A 点抓取逻辑。
- 不改变任务 topic、service、launch、world、URDF、Nav2 和视觉接口。

## 2026-08-02 B 点放置姿态分支和更低释放高度

本次目标：

- 继续降低 B 点释放高度一点，让物块更接近台面后再松爪。
- 在进入 B 点放置下降前，先把机械臂腕部切到固定放置分支，减少靠近台面时被 IK 随机分支和台面碰撞约束共同挤住的概率。

本次修改内容：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
  - 新增 `TACTILE_PLACE_READY_CONFIG = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, -1.5708]`。
  - 触觉放置 `place()` 在 B 点 approach 前，先尝试执行该固定关节位。
  - 该关节位主要把 `ur_wrist_3_joint` 从 Home 的 `0.0` 调整到 `-1.5708`，让 B 点后续 OMPL approach 更倾向稳定腕部分支。
  - 如果该预备关节位执行失败，会打印警告并继续原放置流程，不直接让任务失败。

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
  - `DEFAULT_PLACE_POSE` 的 TCP z 从 `0.705` 降到 `0.700`。
  - 只降低 `5mm`，避免一次降太多重新触发台面碰撞/约束冲突。

- `src/lab_cobot_manipulation/test/test_pick_place_sequence.py`
  - 新增测试，确认触觉放置会在 `move_above` 前进入固定放置关节分支。

- `src/lab_cobot_bringup/test/test_mission_place_pose.py`
  - 更新默认放置高度测试为 `0.700`。

预期效果：

- B 点释放高度比上一版更低 `5mm`，物块落差更小。
- B 点放置前机械臂腕部姿态更固定，降低靠近台面时发生约束冲突或 IK 分支跳变的概率。
- 仍然保留局部 Cartesian 直线下降/直线上抬，不让 RRTConnect 接管贴近台面的局部动作。

本次验证结果：

```text
123 passed
2 packages finished
```

## 2026-08-02 进一步降低 B 点释放高度

本次目标：

- 用户希望放置效果更像“放上去”，而不是物块离台面较高后自由落下。
- 在不修改 world、URDF、Nav2、视觉和任务接口的前提下，只继续微调 B 点默认释放高度。

本次修改内容：

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
  - `DEFAULT_PLACE_POSE` 的 TCP z 从 `0.700` 降到 `0.685`。
  - 触觉放置流程中，`TACTILE_PLACE_RELEASE_CLEARANCE` 与 `TACTILE_PLACE_TCP_Z_COMPENSATION` 相等，因此实际释放 TCP 高度等于 `place_pose.z`。
  - 本次修改会让 B 点释放前再下降 `1.5cm`。

- `src/lab_cobot_bringup/test/test_mission_place_pose.py`
  - 默认 B 点 z 期望同步更新为 `0.685`。

预期效果：

- 按当前几何估算，样件底面离台面由约 `5.45cm` 降到约 `3.95cm`。
- 放置时自由落体距离更小，视觉效果更接近“贴近台面后松开”。
- 仍保留一定安全余量，避免把带焊附着物块直接压进台面，重新触发 Gazebo 接触约束冲突或物块弹飞。
- B 点放置前的固定关节分支、Cartesian 竖直下降和释放后等待逻辑保持不变。

本次验证结果：

```text
75 passed
2 packages finished
Gazebo A->B: task_status=DONE
Place start target=(0.820, 0.200, 0.685) release=(0.820, 0.200, 0.685) approach=(0.820, 0.200, 0.725)
Computed Cartesian path with 18 points (followed 100.000000% of requested trajectory)
夹爪 contact release accepted
Place complete
G4G5_RESULT task_status=DONE g4_touch_ok=True g4_last_contact='released aruco_sample'
```

## 2026-08-02 接触式 B 点放置高度

本次目标：

- 用户明确要求：物块要“放上去”，不是降低落差后“落下去”。
- 因此放置高度从“留几厘米空隙释放”改成“物块底面几乎贴台面再释放”。

本次修改内容：

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
  - `DEFAULT_PLACE_POSE` 的 TCP z 从 `0.685` 降到 `0.650`。
  - 按当前几何估算，释放瞬间样件底面离台面约 `4.5mm`。

- `src/lab_cobot_bringup/test/test_mission_place_pose.py`
  - 默认 B 点 z 期望同步更新为 `0.650`。
  - 放置几何测试从“至少高出台面 1.5cm”改为“只略高出台面，且不低于台面”。

预期效果：

- B 点释放时物块基本已经贴近台面，不再有几厘米自由落体。
- 夹爪仍然先保持闭合，等待 Gazebo 插件确认 detach 后再打开，避免物块在空中松开。
- 继续保留 B 点固定关节分支和 Cartesian 竖直下降，避免 RRTConnect 接管贴近台面的动作。

本次验证结果：

```text
75 passed
2 packages finished
Gazebo A->B: task_status=DONE
Place start target=(0.820, 0.200, 0.650) release=(0.820, 0.200, 0.650) approach=(0.820, 0.200, 0.690)
Computed Cartesian path with 18 points (followed 100.000000% of requested trajectory)
夹爪 contact release accepted
Place complete
G4G5_RESULT task_status=DONE g4_touch_ok=True g4_last_contact='released aruco_sample'
```
