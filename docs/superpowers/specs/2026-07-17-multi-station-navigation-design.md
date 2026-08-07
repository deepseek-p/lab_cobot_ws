# 多工位导航与固定巡航设计

日期：2026-07-17

## 目标

在 `feature/navigation` 分支现有五功能区环境和真实 Nav2 导航链上增加两类任务：

1. 根据 `/task/instruction` 单独导航到任意已知工位。
2. 按固定顺序完成全工位巡航并返回 `home`。

固定巡航顺序为：

```text
home → station_a → inspection_zone → tooling_zone
→ aging_zone → station_b → home
```

如果巡航指令到达时机器人不在 `home`，先使用 Nav2 导航回 `home`，再开始正式巡航。

## 范围与约束

- 保留 `/task/instruction` 和 `/task/status` 的 `std_msgs/String` 接口。
- 保留现有 A→B 搬运任务的状态机、抓取、放置、退避、持物收臂和返航逻辑。
- 不修改 vendored `src/pymoveit2/`。
- 不使用 teleport、Gazebo 真值位姿控制或 `/set_entity_state` 完成导航。
- 运行时继续使用 AMCL/EKF、真实 `/odom`、`map→base_link` TF 和 Nav2。
- 保持 `use_truth_pose=false`、`use_sim_attach=false`。
- 保持 `use_tactile_grasp=true` 与 `require_finger_contact=true` 成对开启。
- 初始设计不修改 `navigation.launch.py`、Nav2 生命周期编排或 `nav2_params.yaml`；只有仿真证据表明现有 waypoint 不可达时，才对 waypoint 做最小修正。
- 不扩展为任务队列、中断恢复、任意用户自定义路线或 20 条有向路径统计；这些属于后续任务。

## 当前实现

- `waypoints.py` 已定义 `station_a`、`inspection_zone`、`tooling_zone`、
  `aging_zone`、`station_b` 和 `home` 的 map 位姿。
- `MissionNode._navigate(station)` 已能把任意 waypoint 发送给
  `BasicNavigator.goToPose()`。
- `MissionNode._dock_to_station_pose(station)` 已提供地图位姿二次精停。
- 当前任务规划器和 `SequentialTask` 只表达 A→B 搬运动作，无法携带任意工位目标。
- 当前导航 launch 刻意不启动 `waypoint_follower`，以避免已知 GUI 生命周期启动竞态。
- 设计前定向基线测试为 `81 passed`。

## 方案选择

采用“独立工位路线任务 + 保留旧搬运状态机”的方案。

导航指令先由确定性解析器识别。单站和巡航任务进入新增的工位路线执行器；其他指令继续进入现有 `plan_actions → SequentialTask`。这样不需要把旧 `TaskState` 改造成参数化动作，也不需要迁移现有 LLM 动作协议。

不采用以下方案：

- 全量参数化重构：长期扩展性较好，但会同时改变旧状态机、LLM 输出协议、前置条件校验和诚实 E2E 依赖，超出本次低风险范围。
- `waypoint_follower` 或单次 `NavigateThroughPoses`：无法复用现有逐站精停和逐站状态；启用 `waypoint_follower` 还会重新引入已知生命周期竞态。

## 架构

```text
/task/instruction
       |
       +-- 确定性导航指令解析
       |       +-- 单站路线
       |       +-- 固定巡航路线
       |                |
       |          StationRouteTask
       |                |
       |       NavigateToPose + 地图精停
       |
       +-- 未匹配
               |
        现有 plan_actions
               |
        现有 SequentialTask
        A→B 抓取搬运流程
```

## 工位数据与别名

`waypoints.py` 继续作为工位名称和位姿的事实源，并增加固定巡航顺序和工位别名。`get_waypoint()` 返回的字典结构仍保持 `x/y/yaw`，避免破坏现有消费者。

支持的别名至少包括：

| 标准名 | 指令别名 |
|---|---|
| `station_a` | `A工位`、`工位A`、`station_a` |
| `inspection_zone` | `检测区`、`inspection_zone` |
| `tooling_zone` | `工具区`、`工装区`、`tooling_zone` |
| `aging_zone` | `老化区`、`aging_zone` |
| `station_b` | `B工位`、`工位B`、`station_b` |
| `home` | `home`、`起始点` |

巡航路线常量保存完整逻辑顺序，但执行器把起始 `home` 解释为前置归位条件，不重复发送无意义的零距离 goal。

## 指令解析

在现有规划器前增加纯逻辑导航请求解析，返回以下三种结果之一：

- 单站请求：包含一个标准工位名。
- 巡航请求：包含固定巡航标志。
- 未匹配：继续交给现有 `plan_actions()`。

需要支持的单站示例：

```text
去检测区
导航到 tooling_zone
去B工位
```

需要支持的巡航示例：

```text
巡航所有工位
按顺序访问全部工位并回家
```

单站规则采用完整句式匹配，而不是任意子串命中。因此“去A工位检查一下样件然后回家”继续进入旧任务规划器，不会被截断成单站导航。

当指令明确使用“导航到/去/前往”等导航句式但目标无法归一化为已知工位时，返回非法工位错误，不允许回退成默认 A→B 搬运流程。

## 路线状态机

在 `task_state_machine.py` 中新增独立的 `StationRouteTask`，不修改现有 `TaskState`、`SequentialTask` 和 `CrossStationTask` 的语义。

`StationRouteTask` 负责：

- 保存有序工位列表和当前下标。
- 暴露当前目标工位。
- 单站成功后推进到下一站。
- 当前站失败时允许一次重试。
- 重试耗尽后进入失败终态。
- 所有站完成后进入成功终态。

巡航前置归位由 mission 层根据真实 map TF 判定：

- 已满足现有 home 距离和朝向容差时，直接开始 `station_a`。
- 不满足时，先运行一次到 `home` 的路线步骤。

## MissionNode 执行流

`MissionNode._run_mission()` 先解析导航请求：

1. 等待 Nav2 action server 和 `bt_navigator` active。
2. 单站或巡航任务开始前要求机械臂成功回到 home 构型。
3. 对每个目标依次执行现有 `_navigate(station)` 和
   `_dock_to_station_pose(station)`。
4. 到站后发布到达状态并推进路线。
5. 非导航请求继续走现有 A→B 任务分支。

导航到 `station_a` 或 `station_b` 不触发检测、抓取或放置。只有旧 `NAV_TO_PICK` 和 `NAV_TO_PLACE` 分支继续执行视觉停靠与操作动作。

单站任务到达目标后停止并进入 `DONE`，不自动返回 `home`。

## 状态接口与兼容性

现有 A→B 成功状态序列保持不变：

```text
NAV_TO_PICK
DETECT
PICK
NAV_TO_PLACE
PLACE
RETURN_HOME
DONE
```

单站导航示例：

```text
NAV_TO_STATION:inspection_zone
ARRIVED:inspection_zone
DONE
```

从非 home 位置启动巡航时：

```text
RETURN_HOME
ARRIVED:home
NAV_TO_STATION:station_a
ARRIVED:station_a
NAV_TO_STATION:inspection_zone
ARRIVED:inspection_zone
NAV_TO_STATION:tooling_zone
ARRIVED:tooling_zone
NAV_TO_STATION:aging_zone
ARRIVED:aging_zone
NAV_TO_STATION:station_b
ARRIVED:station_b
RETURN_HOME
ARRIVED:home
DONE
```

失败时先发布可诊断原因，再发布旧终态：

```text
FAILED:navigation_failed:tooling_zone
FAILED
```

这样现有只判断精确 `FAILED` 的订阅方和诚实 E2E 仍能工作。旧 A→B 成功状态名不增加前缀或参数。

## 安全与碰撞约束

- 每一段均通过 Nav2 `NavigateToPose` 执行，并继续使用全局/局部代价地图避障。
- 每站继续执行地图精停，保留 waypoint 朝向。
- 单站和巡航移动前必须收臂；收臂失败时不允许底盘继续移动。
- `station_a` 和 `station_b` 的既有工作台前缘安全逻辑保持不变。
- `tooling_zone` 和 `aging_zone` 按当前冻结 world 的工作台前缘加入同一净距约束；`inspection_zone` 没有同类工作台，不套用该约束。
- 不降低 footprint、robot radius、inflation radius、目标容差或碰撞检查要求来迁就新路线。
- 若仿真证明某 waypoint 落在不可达代价区，只允许基于地图、TF 和 Nav2 日志对该 waypoint 做最小位置调整，不修改地图来源链或放宽安全参数。

## 错误处理

错误原因至少区分：

- `unknown_station`
- `nav_not_ready`
- `arm_not_stowed`
- `navigation_failed`
- `docking_failed`
- `unexpected_exception`

失败处理顺序：

1. 取消仍在执行的 Nav2 goal。
2. 发布零速度并停止底盘。
3. 保持或重新请求机械臂 home 构型。
4. 发布带原因和目标工位的 `FAILED:<reason>`。
5. 发布兼容终态 `FAILED`。
6. 清除 busy 标志，允许下一条指令。

Nav2 或停靠已经失败时，不继续访问后续工位，也不盲目强制返航。非法工位在发送任何导航 goal 前失败。

现有任务执行中忽略新指令的 busy 行为不在本次修改范围内。

## 测试设计

### 纯逻辑与配置测试

- waypoint 表包含全部标准工位。
- 固定巡航顺序精确匹配确认顺序。
- 所有巡航工位都能查询到 `x/y/yaw`。
- 中文和标准名别名正确归一化。
- 单站和巡航指令正确解析。
- 非法工位被拒绝，且不会回退为 A→B 搬运。
- 旧复合指令不会被单站规则误匹配。
- `StationRouteTask` 按顺序推进、允许一次重试、重试耗尽后失败。
- 工具区和老化区的最终停靠保持工作台净距和既有朝向。

### Mission 接线测试

- 单站任务只调用目标站导航和地图精停，不调用检测、抓取或放置。
- 巡航从非 home 位置开始时先归位，再按五站顺序执行并最终返航。
- 已在 home 时不发送重复的前置归位 goal。
- 状态序列包含逐站 `NAV_TO_STATION` 和 `ARRIVED`。
- 导航或停靠失败时停止路线、停止底盘并发布原因与兼容 `FAILED`。
- 旧 A→B 执行顺序和成功状态序列保持不变。

### 构建与回归

执行定向 pytest、受影响包 build/test、`git diff --check` 和
`colcon test-result --verbose`。测试期间保持离线、CPU-safe，不下载模型。

## 仿真验收

### Headless

以以下锁定参数启动：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  gui:=false use_rviz:=false \
  use_truth_pose:=false use_sim_attach:=false \
  use_tactile_grasp:=true require_finger_contact:=true
```

验证：

1. 单独导航到一个指定工位并到 `DONE`。
2. 从该非 home 工位发送巡航指令。
3. 确认先回 `home`，再依次访问五个工位并最终返航。
4. 记录每个工位的状态序列、Nav2 结果、最终 `DONE/FAILED` 和已知警告。

### Gazebo GUI

以 `gui:=true use_rviz:=false` 和同样的安全参数重复单站与巡航演示。检查 GUI 负载下生命周期节点 active、启动/停止无编排卡死，并确认机器人朝向、工作台净距和路线顺序。

演示完成后默认保持 Gazebo GUI 打开供用户复核，并在报告中明确进程是否仍在运行。若 GUI 或导航栈异常退出，必须如实报告，不得用 headless 结果替代 GUI 结论。

## 预计修改文件

- `src/lab_cobot_navigation/lab_cobot_navigation/waypoints.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/task_planner.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/task_state_machine.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- 上述模块的现有测试文件，必要时增加独立的多工位任务测试文件

不预计修改：

- `src/pymoveit2/`
- 地图三件套和 `check_map.py`
- `navigation.launch.py`
- `nav2_params.yaml`
- 抓取、放置和触觉安全默认值

## 完成判据

- 单站指令能导航到任意已知工位并明确发布目标及到达状态。
- 巡航从任意当前位置先归位，再按确认顺序访问五站并返回 `home`。
- 非法工位在运动前失败，并给出明确原因。
- 现有 A→B 搬运成功状态和操作链不回退。
- 新旧自动化测试全部通过。
- 至少一次 headless 导航验证通过。
- 一次 `gui:=true use_rviz:=false` 的单站与完整巡航演示完成。
- 报告仅陈述实际验证过的仿真结果，不宣称真实物理性能。
