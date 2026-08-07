# GitHub main 底盘移植到五功能区环境设计

日期：2026-07-16

## 目标

把 `/home/THW22/projects/lab_cobot_ws` 在 GitHub `origin/main` 提交
`7f6207f72d104f97875c72054544b35e6d40c3ce` 中的完整底盘子系统移植到
`/home/THW22/projects/navigation` 的 `feature/navigation` 分支，同时保留该分支的五功能区
Gazebo 世界、环境模型、光照变体、actor 变体和环境 waypoint。

移植后的默认底盘必须继续采用 `pose_from_wheel_commands` 位姿积分模型。不得使用
`gazebo_ros_planar_move`，也不得把该模型描述成真实麦轮滚子接触动力学或力级物理驱动。

## 已确认基线

- 底盘来源：`lab_cobot_ws` 的 `main`，提交 `7f6207f`。
- 修改目标：`navigation` 的 `feature/navigation`，提交 `ef13df6`。
- 来源底盘：`0.55 × 0.50 m` 盒式麦轮底盘，轮半径 `0.08 m`，轮关节为
  `wheel_fl_joint`、`wheel_fr_joint`、`wheel_rl_joint`、`wheel_rr_joint`。
- 来源驱动：`lab_cobot_mecanum_drive`，默认模式
  `pose_from_wheel_commands`，插件发布 `/odom`。
- 目标环境：五功能区 world，包含 normal/dark/reflective 和可选 actor 变体。
- 目标出生点：map/world 坐标 `(2.25, -2.10)`；换用来源底盘后出生高度改为
  `z=0.0`，使 `base_footprint` 位于地面。

## 方案

采用“完整底盘子系统移植”，而不是只换外观，也不整包覆盖 Gazebo 与 bringup。

### 机器人描述

- 引入来源仓库的 `mecanum_base.xacro`，停止使用 `mecanum3_base.xacro`。
- 顶层 URDF 恢复来源底盘尺寸、轮关节、ros2_control 接口和
  `lab_cobot_mecanum_traction` 插件参数。
- 立柱、传感器安装关系、SRDF 被动关节和 controller joint 列表与来源底盘一致。
- 不复制或修改机械臂、夹爪、腕相机及触觉安全默认值。
- mecanum3 STL 可在确认无引用后删除；删除前必须用生成 URDF 和仓库搜索证明无消费者。

### 驱动与 bringup

- `/cmd_vel` 由来源仓库的 `mecanum_wheel_visualizer` 转为四轮速度命令。
- 四轮速度发送到 `/wheel_velocity_controller/commands`。
- Gazebo 内的 `lab_cobot_mecanum_drive` 按麦轮正解积分底盘位姿并发布 `/odom`。
- 从默认启动链移除目标分支的 `rover_twist_relay`、
  `passive_mecanum_joint_states`、`lab_cobot_planar_drive`、
  `mecanum_gazebo_kinematic_drive` 和 `gazebo_odom_bridge`。
- 无消费者的替代驱动源码、构建目标和测试在确认后删除，避免同一仓库保留相互冲突的
  默认底盘事实源。

### 环境与导航

- 保留所有五功能区 world、模型、光照选择函数、actor 开关和离线 Gazebo 资源配置。
- 保留目标分支的五功能区 waypoint，不用来源仓库旧双工位 waypoint 覆盖。
- 出生位置保留 `x=2.25`、`y=-2.10`，只将 `z` 调整为来源底盘合同要求的 `0.0`。
- Nav2 local footprint 恢复为包含轮外缘的
  `[[0.28, 0.31], [0.28, -0.31], [-0.28, -0.31], [-0.28, 0.31]]`。
- global robot radius 恢复为 `0.42 m`，local/global inflation radius 恢复为 `0.55 m`。
- 任务代码中仅更新底盘尺寸与直接由尺寸推导的安全距离；保留五功能区坐标、轴对齐
  导航逻辑、工作台安全限制和新环境任务语义。
- 若停靠常量无法由底盘尺寸直接推导，先保留目标值，通过 Gazebo 探针验证后再作最小
  调整，不直接复制来源仓库旧环境常量。

## 兼容性与故障处理

- 生成 URDF 必须只有一个默认底盘推进插件和一个 `/odom` 发布源。
- 四个 wheel controller joint、驱动插件 joint 顺序和可视化逆解顺序必须完全一致。
- world launch 必须在控制器启动失败时停止后续控制链，保留目标分支现有 fail-fast 行为。
- Gazebo 找不到模型、mesh 或插件时视为失败，不用静默降级掩盖。
- 底盘出生后若出现持续下沉、倾斜、抖动、异常速度或与环境模型初始重叠，必须定位并
  修复，不通过提高出生高度长期悬空来规避。
- 不修改地图三件套、地图来源链、抓取安全参数和诚实 E2E 断言。

## 测试设计

### 静态与单元合同

- xacro 展开成功，URDF/SDF 可解析。
- 底盘尺寸、质量、轮几何、wheel joint、controller joint、插件 joint 顺序一致。
- 默认插件为 `liblab_cobot_mecanum_drive.so` 且模式为
  `pose_from_wheel_commands`。
- 默认启动链不存在 planar drive、异步 `/set_entity_state` 驱动或第二 odom 源。
- world launch 仍支持 normal/dark/reflective 与 actor 组合。
- Nav2 footprint、robot radius 和 inflation 参数与来源底盘外廓匹配。

### 构建与包测试

构建并测试：

- `lab_cobot_description`
- `lab_cobot_gazebo`
- `lab_cobot_bringup`
- `lab_cobot_navigation`

执行 `git diff --check`、定向 pytest、`colcon build --symlink-install`、
`colcon test` 和 `colcon test-result --verbose`。

### Gazebo 验收

先清理残留 Gazebo 与 ROS 进程，再验证：

1. normal world headless 启动，机器人、全部环境模型和控制器成功加载。
2. `/gazebo/model_states` 中机器人出生位置正确；静止采样期间 z、roll、pitch 无持续漂移，
   无异常线速度或角速度。
3. 发布低速前进、横移和旋转命令，确认底盘方向、轮子转向、`/odom` 和 TF 一致。
4. 检查出生区、主要走廊和工作台前没有初始碰撞或明显穿模。
5. dark、reflective 和至少一个 actor 变体能成功启动且模型资源完整。
6. 使用 `gui:=true` 实际启动和停止一次，确认 WSLg 下无编排卡死和残留进程。

## 完成判据

- 五功能区环境资产与 waypoint 未被来源仓库旧环境覆盖。
- 默认运行只使用来源 `main` 的盒式麦轮底盘与位姿积分驱动链。
- 相关包构建和测试通过，`colcon test-result` 无失败。
- Gazebo 启动无缺失资源、插件错误、底盘下沉/抖动、出生碰撞或明显环境穿模。
- 仿真报告明确列出已验证环境变体、关键 topic/pose 数据和仍存在的真实边界。
