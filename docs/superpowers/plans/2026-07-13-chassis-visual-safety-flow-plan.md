# Chassis Visual Safety Flow Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在保留原任务、Nav2 和麦轮解算的前提下恢复四轮真实转动，阻止底盘进入工作台安全区，为 MoveIt 注册桌体，并验证完整任务稳定到 `DONE`。

**Architecture:** `rover_twist_relay` 与原麦轮公式不变；Gazebo 平面插件只积分并设置安全位姿，不再覆盖全模型 link 速度。安全停车采用 mission 正常减速和插件禁入区双层保护，MoveIt planning scene 通过独立初始化节点获得两张工作台。

**Tech Stack:** ROS 2 Humble、Gazebo Classic 11、gazebo_ros2_control、C++17、rclpy、MoveIt 2 PlanningScene、pytest、gtest、colcon

---

## 文件结构

- 修改 `src/lab_cobot_gazebo/src/lab_cobot_planar_drive.cpp`：停止覆盖全模型 link 速度并应用安全区。
- 新建 `src/lab_cobot_gazebo/include/lab_cobot_gazebo/planar_safety.hpp`：纯函数实现旋转底盘矩形与工作台安全区判断。
- 新建 `src/lab_cobot_gazebo/test/test_planar_safety.cpp`：安全区 gtest。
- 修改 `src/lab_cobot_gazebo/src/gazebo_odom_bridge.cpp`：按仿真时间对实际位姿差分发布 twist。
- 新建 `src/lab_cobot_gazebo/include/lab_cobot_gazebo/pose_differentiator.hpp` 与对应 gtest：隔离差分数学。
- 修改 `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`：显式传入底盘外形与两张桌体安全配置。
- 修改 `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`：原精停流程中增加安全距离减速/完成条件。
- 新建 `src/lab_cobot_moveit/lab_cobot_moveit/table_scene_initializer.py`：幂等注册两张桌体。
- 修改 `src/lab_cobot_moveit/CMakeLists.txt`、`package.xml`：安装节点并声明依赖。
- 修改 `src/lab_cobot_bringup/launch/lab_cobot.launch.py`：在原 MoveIt 阶段启动桌体初始化节点。
- 更新对应 Python/C++ 契约测试和 `README.md`。

### Task 1: 恢复主轮真实转动并改用差分 odom

**Files:**
- Modify: `src/lab_cobot_gazebo/src/lab_cobot_planar_drive.cpp`
- Modify: `src/lab_cobot_gazebo/src/gazebo_odom_bridge.cpp`
- Create: `src/lab_cobot_gazebo/include/lab_cobot_gazebo/pose_differentiator.hpp`
- Create: `src/lab_cobot_gazebo/test/test_pose_differentiator.cpp`
- Modify: `src/lab_cobot_gazebo/test/test_mecanum_drive_plugin.py`
- Modify: `src/lab_cobot_gazebo/CMakeLists.txt`

- [ ] **Step 1: 写失败契约，禁止模型级速度覆盖**

```python
def test_planar_plugin_does_not_overwrite_all_link_velocities():
    source = (GAZEBO / "src" / "lab_cobot_planar_drive.cpp").read_text()
    assert "model_->SetLinearVel" not in source
    assert "model_->SetAngularVel" not in source
```

- [ ] **Step 2: 写位姿差分 gtest**

```cpp
TEST(PoseDifferentiator, ComputesBodyTwistAcrossYaw)
{
  PoseSample previous{0.0, 0.0, 0.0, 1.0};
  PoseSample current{0.0, 1.0, 0.0, 2.0};
  const auto twist = DifferentiatePose(previous, current);
  EXPECT_NEAR(twist.vx, 1.0, 1e-9);
  EXPECT_NEAR(twist.vy, 0.0, 1e-9);
  EXPECT_NEAR(twist.wz, 0.0, 1e-9);
}

TEST(PoseDifferentiator, RejectsPauseRollbackAndLargeDt)
{
  EXPECT_FALSE(DifferentiatePoseSafe({0, 0, 0, 1}, {1, 0, 0, 1}, 0.2).valid);
  EXPECT_FALSE(DifferentiatePoseSafe({0, 0, 0, 2}, {1, 0, 0, 1}, 0.2).valid);
  EXPECT_FALSE(DifferentiatePoseSafe({0, 0, 0, 1}, {1, 0, 0, 2}, 0.2).valid);
}
```

- [ ] **Step 3: 运行红灯测试**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 -m pytest -q -p no:anyio \
  src/lab_cobot_gazebo/test/test_mecanum_drive_plugin.py
```

Expected: FAIL，源文件仍包含两个模型级速度调用。

- [ ] **Step 4: 实现纯差分 helper**

```cpp
struct PoseSample { double x; double y; double yaw; double time; };
struct DifferentiatedTwist { double vx; double vy; double wz; bool valid; };

inline DifferentiatedTwist DifferentiatePoseSafe(
  const PoseSample & previous, const PoseSample & current, double max_dt)
{
  const double dt = current.time - previous.time;
  if (!(dt > 0.0) || dt > max_dt) return {0.0, 0.0, 0.0, false};
  const double dx = (current.x - previous.x) / dt;
  const double dy = (current.y - previous.y) / dt;
  const double c = std::cos(current.yaw);
  const double s = std::sin(current.yaw);
  const double dyaw = std::atan2(
    std::sin(current.yaw - previous.yaw),
    std::cos(current.yaw - previous.yaw));
  return {c * dx + s * dy, -s * dx + c * dy, dyaw / dt, true};
}
```

- [ ] **Step 5: 移除插件模型级速度设置并让 odom bridge 使用 helper**

`holdPlanarPose()` 只保留 `SetWorldPose`。`gazebo_odom_bridge` 保存上一帧 base pose 和仿真时间；有效差分写入 `twist.twist.linear.x/y` 与 `angular.z`，无效 dt 写零。

- [ ] **Step 6: 注册 gtest、构建和测试**

```bash
colcon build --symlink-install --packages-select lab_cobot_gazebo
source install/setup.bash
colcon test --packages-select lab_cobot_gazebo
colcon test-result --test-result-base build/lab_cobot_gazebo --verbose
```

Expected: gazebo 包 0 failure。

- [ ] **Step 7: 动态验证四轮 position 变化**

启动原 bringup（关闭 mission），20 Hz 发布 `linear.x=0.2` 两秒，要求四个 wheel joint `abs(delta position) > 1 rad`，同时车体 x 正向移动。

- [ ] **Step 8: 提交**

```bash
git add src/lab_cobot_gazebo
git commit -m "fix(gazebo): preserve visual wheel rotation"
```

### Task 2: 添加插件级工作台安全区

**Files:**
- Create: `src/lab_cobot_gazebo/include/lab_cobot_gazebo/planar_safety.hpp`
- Create: `src/lab_cobot_gazebo/test/test_planar_safety.cpp`
- Modify: `src/lab_cobot_gazebo/src/lab_cobot_planar_drive.cpp`
- Modify: `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`
- Modify: `src/lab_cobot_gazebo/CMakeLists.txt`
- Modify: `src/lab_cobot_gazebo/test/test_mecanum_drive_plugin.py`

- [ ] **Step 1: 写旋转底盘盒与禁入区红灯测试**

```cpp
TEST(PlanarSafety, BlocksChassisEnteringExpandedTable)
{
  const OrientedBox chassis{2.0, 0.60, M_PI_2, 0.42, 0.30};
  const AxisAlignedBox table{1.6, 2.4, 1.2, 1.8};
  EXPECT_TRUE(IntersectsExpandedTable(chassis, table, 0.35));
}

TEST(PlanarSafety, AllowsPoseOutsideSafetyLine)
{
  const OrientedBox chassis{2.0, 0.50, M_PI_2, 0.42, 0.30};
  const AxisAlignedBox table{1.6, 2.4, 1.2, 1.8};
  EXPECT_FALSE(IntersectsExpandedTable(chassis, table, 0.35));
}
```

- [ ] **Step 2: 实现 SAT 纯函数**

实现 `OrientedBoxCorners` 和矩形分离轴判断。安全扩张只作用于桌体外缘；底盘尺寸使用 `0.42 x 0.30 m`，不使用 base 点代替车体。

- [ ] **Step 3: 在插件中解析并验证安全配置**

URDF 插件参数：

```xml
<chassis_length>0.42</chassis_length>
<chassis_width>0.30</chassis_width>
<table_safety_margin>0.35</table_safety_margin>
<table_a>2.0 1.5 0.8 0.6</table_a>
<table_b>-2.0 1.5 0.8 0.6</table_b>
```

非正尺寸、负 margin 或格式错误时抛出明确异常。

- [ ] **Step 4: 在设置下一位姿前阻挡进入安全区**

若下一底盘盒进入安全区而当前盒未进入，拒绝平移并把 `vx_`、`vy_` 置零。若当前已在安全区，只允许使碰撞深度减小的远离运动。旋转后进入安全区同样拒绝该 yaw 更新。

- [ ] **Step 5: 构建、测试并做持续前进动态验证**

向 station A 南侧持续发布前进命令 5 秒，要求车体外壳到桌边净距 `>=0.34 m`，随后继续发布也不再接近；反向命令必须能离开。

- [ ] **Step 6: 提交**

```bash
git add src/lab_cobot_gazebo src/lab_cobot_description
git commit -m "fix(gazebo): enforce table safety zones"
```

### Task 3: 保持原精停流程并在安全线完成停车

**Files:**
- Modify: `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- Modify: `src/lab_cobot_bringup/test/test_mission_docking.py`
- Modify: `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`

- [ ] **Step 1: 写安全距离减速测试**

```python
def test_station_dock_stops_at_table_safety_line():
    done, command = station_dock_velocity_for_base(
        (1.94, 0.50, math.radians(90)), "station_a"
    )
    assert done
    assert command == (0.0, 0.0, 0.0)

def test_station_dock_slows_before_safety_line():
    done, command = station_dock_velocity_for_base(
        (1.94, 0.20, math.radians(90)), "station_a"
    )
    assert not done
    assert 0.0 < command[0] <= DOCK_LINEAR_SLOW_SPEED
```

- [ ] **Step 2: 调整完成条件而不改变调用顺序**

保留 `_navigate -> _dock_to_station_pose -> _dock_to_pick_target`。`station_dock_velocity_for_base` 使用车体外壳到桌边净距完成停车；`_dock_to_pick_target` 在达到安全线后只做横向/yaw 微调，不再产生向桌体方向的正速度。

- [ ] **Step 3: 运行 mission 纯函数测试**

```bash
python3 -m pytest -q -p no:anyio \
  src/lab_cobot_bringup/test/test_mission_docking.py \
  src/lab_cobot_bringup/test/test_mission_navigation_handoff.py
```

Expected: 全部 PASS。

- [ ] **Step 4: 提交**

```bash
git add src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py \
  src/lab_cobot_bringup/test/test_mission_docking.py \
  src/lab_cobot_bringup/test/test_mission_navigation_handoff.py
git commit -m "fix(mission): stop chassis before worktables"
```

### Task 4: 向 MoveIt 注册两张工作台

**Files:**
- Create: `src/lab_cobot_moveit/lab_cobot_moveit/table_scene_initializer.py`
- Create: `src/lab_cobot_moveit/test/test_table_scene_initializer.py`
- Modify: `src/lab_cobot_moveit/CMakeLists.txt`
- Modify: `src/lab_cobot_moveit/package.xml`
- Modify: `src/lab_cobot_bringup/launch/lab_cobot.launch.py`
- Modify: `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`

- [ ] **Step 1: 写桌体消息纯函数测试**

```python
def test_table_collision_objects_match_gazebo_world():
    objects = build_table_collision_objects("odom")
    assert [(obj.id, obj.primitives[0].dimensions) for obj in objects] == [
        ("station_a_table", [0.8, 0.6, 0.75]),
        ("station_b_table", [0.8, 0.6, 0.75]),
    ]
    assert objects[0].primitive_poses[0].position.x == 2.0
    assert objects[1].primitive_poses[0].position.x == -2.0
```

- [ ] **Step 2: 实现幂等 planning scene 初始化节点**

使用 `moveit_msgs/srv/ApplyPlanningScene`。节点等待服务，发送 `PlanningScene(is_diff=True)`，两个 `CollisionObject.operation=ADD`；成功后记录日志并退出，失败按固定次数重试并返回非零。

- [ ] **Step 3: 安装节点并在原 stage2 启动**

只把 `table_scene_initializer` 加入现有 `stage2` actions，不改变 move_group、navigation、perception 或 mission 时序。

- [ ] **Step 4: 运行 moveit 与 bringup 测试**

```bash
colcon build --symlink-install --packages-select lab_cobot_moveit lab_cobot_bringup
source install/setup.bash
python3 -m pytest -q -p no:anyio src/lab_cobot_moveit/test src/lab_cobot_bringup/test/test_lab_cobot_launch.py
```

- [ ] **Step 5: 动态查询 planning scene**

启动原 bringup 后调用 `/get_planning_scene`，要求 world collision objects 精确包含 `station_a_table` 和 `station_b_table`。

- [ ] **Step 6: 提交**

```bash
git add src/lab_cobot_moveit src/lab_cobot_bringup
git commit -m "fix(moveit): register worktable collision geometry"
```

### Task 5: 完整回归、抓取坐标验证与文档

**Files:**
- Modify: `README.md`
- Modify tests only if a newly reproduced regression needs a persistent assertion

- [ ] **Step 1: 构建全部受影响包**

```bash
colcon build --symlink-install --packages-select \
  lab_cobot_description lab_cobot_gazebo lab_cobot_moveit lab_cobot_bringup
```

- [ ] **Step 2: 运行相关包测试**

```bash
colcon test --packages-select \
  lab_cobot_description lab_cobot_gazebo lab_cobot_moveit lab_cobot_bringup
colcon test-result --verbose
```

- [ ] **Step 3: 验证视觉与安全性**

原入口关闭 mission 启动，依次验证六向运动、四轮 position、odom twist、watchdog、z/roll/pitch；持续向两张工作台推进，确认安全净距。

- [ ] **Step 4: 验证抓取坐标**

在 station A 停车后记录 base、TCP、双指和样件世界位姿。要求全部有限，抓取插件 offset 每轴在配置封套内，不再出现米级数值。

- [ ] **Step 5: 连续运行两次完整原任务**

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '把样件从A送到B'}"
```

每轮要求状态完整到 `DONE`，station B 样件中心 z 位于桌面以上，机器人回到 home；任何失败必须保留状态、MoveIt 错误、接触状态和模型位姿证据后再修复。

- [ ] **Step 6: 更新 README**

说明轮子由 ros2_control 实际转动、底盘采用运动学安全区而非轮地动力学、0.35 m 安全距离和 MoveIt 工作台碰撞对象。

- [ ] **Step 7: 最终提交**

```bash
git add README.md
git commit -m "docs: document chassis visual and table safety"
git diff --check
git status -sb
```
