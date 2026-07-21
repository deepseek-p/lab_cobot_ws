# CS-202618 实验室移动协作机器人

面向实验室智能管理的移动协作机器人仿真系统。ROS 2 Humble + Gazebo Classic 11 + Nav2 + MoveIt 2 + ArUco 感知，麦克纳姆底盘 + UR5e + 平行双指夹爪，五功能区实验室场景。

赛题：CS-202618 中车株洲电力机车有限公司「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」。

---

## 当前状态 (2026-07-21)

**开发分支**: `feature/navigation`（领先 origin/feature/navigation 7 commits）

| 检查项 | 状态 |
|--------|------|
| `colcon build`（8 包） | PASS |
| `colcon test`（单元/contract，不含 E2E） | PASS（5/6，E2E 需运行中 ROS 2 系统） |
| 五功能区环境（14×14m） | 已部署 |
| 导航地图（300×300 px, 0.05m/pixel） | 已生成，覆盖 15×15m |
| 20 条有向路径 contract test | PASS |
| AMCL 初始位姿 | 已修正为 2× 坐标 |
| 精停 docking 距离 | ~0.87m（UR5e 可及范围） |
| 视觉精停跳过 | `skip_visual_dock:=true` 可用 |
| 五区 E2E 全闭环 | **待手动验证**（需逐路径启动 bringup） |

---

## 一、五功能区环境总览

### 1.1 实验室布局（14×14m，2× 缩放）

```
        +y (北)
        |
   [station_a]         [aging_zone]        [inspection_zone]
   (-4.30, 3.80)       (0.20, 4.20)        (4.10, 1.10)
   桌面: 1.6×1.2m       桌面: 1.6×1.2m       地面: 高压区围栏
   物品: 3 件           物品: 1 件            物品: 2 件(地面)
        |                    |                    |
   -----+--------------------+--------------------+-----> +x (东)
        |                    |                    |
   [tooling_zone]                            [home]
   (-4.10, -2.30)                            (4.50, -4.20)
   桌面: 1.6×1.2m                            发车/归位区
   物品: 2 件
        |                    |
        |              [station_b]
        |              (0.30, -1.70)
        |              桌面: 1.6×1.2m
        |              物品: 1 件
        |
        -y (南)
```

四张工作台统一尺寸：**1.6m(x) × 1.2m(y) × 0.75m(z)**，桌面高度 z=0.75m。

### 1.2 六工位 Waypoint 表

| 站名 | 别名 | x | y | yaw | 朝向 | 对应桌面中心 |
|------|------|---|---|-----|------|-------------|
| `home` | 起始点 | 4.50 | -4.20 | 0 (东) | +x | — |
| `station_a` | A工位, 工位A | -4.30 | 2.48 | π/2 (北) | +y | (-4.30, 3.80) |
| `inspection_zone` | 检测区 | 4.10 | 1.10 | π/2 (北) | +y | —（地面站位） |
| `tooling_zone` | 工具区, 工装区 | -4.10 | -3.30 | π/2 (北) | +y | (-4.10, -2.30) |
| `aging_zone` | 老化区 | 0.20 | 3.20 | π/2 (北) | +y | (0.20, 4.20) |
| `station_b` | B工位, 工位B | 0.30 | -3.01 | π/2 (北) | +y | (0.30, -1.70) |

**设计说明**：五站 yaw 统一为 π/2（朝北/+y），车头始终向前；仅 home 为 0（朝东/+x）。

### 1.3 巡航路线

```
home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home
```

逆时针周界巡逻：东南发车 → 西北(A) → 东北(检测) → 西南(工具) → 北中(老化) → 南中(B) → 归位。

---

## 二、待抓取物品清单（视觉组协作参考）

### 2.1 各工位物品详情

#### Station A（A工位）— 3 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **ArUco 标记样件** | `aruco_sample` | `aruco_sample` | (-4.16, 3.46, 0.785, 0.10) | **是（默认目标）** | 带 ArUco 标记的方块，默认抓取目标 |
| **IGBT 模块备件** | `material_spare_igbt` | `igbt_module_plain` | (-4.62, 3.92, 0.78, 0.38) | **是** | 深灰色方块 0.09×0.09×0.06m，非静态，有质量/摩擦 |
| **导热硅脂罐** | `material_grease_can` | `thermal_grease_can` | (-3.90, 3.96, 0.75, 0) | 否（道具） | 带罐盖(cap)视觉元素 |

#### Tooling Zone（工具区）— 2 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **工装夹具盒** | `tooling_fixture_box` | `fixture_box_plain` | (-3.88, -2.04, 0.80, -0.28) | **是** | 金棕色方块 0.16×0.12×0.10m，非静态，有质量/摩擦 |
| **手工工具** | `tooling_hand_tools` | `tooling_hand_tools` | (-4.36, -1.96, 0.75, 0.12) | 否（道具） | 红色螺丝刀手柄(screwdriver_handle_red) |

#### Aging Zone（老化区）— 1 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **老化架** | `aging_rack` | `aging_rack` | (0.20, 4.26, 0.80, 0) | 否（道具） | 3 槽位(slot_left/mid/right) + 状态指示灯(绿/黄/红) |

#### Station B（B工位）— 1 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **PCB 测试夹具** | `board_test_fixture` | `pcb_test_fixture` | (0.02, -1.44, 0.75, 0.22) | 否（道具） | 绿色 LED 指示灯(indicator_led_green) |

#### Inspection Zone（检测区/高压区）— 2 件物品（地面，非桌面）

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **高压探头套件** | `high_voltage_probe_kit` | `safety_probe_kit` | (4.04, 2.44, 0.0, -0.18) | 否（地面道具） | 红色探头手柄(probe_red_handle) |
| **高压区围栏** | `high_voltage_zone` | `high_voltage_zone` | (4.36, 2.90, 0.0, 0.12) | 否（围栏） | 4 面墙 + 4 立柱，碰撞盒 2.0×1.68m |

### 2.2 可抓取物品汇总

| 优先级 | 物品 | 所在工位 | 碰撞盒尺寸(m) | 桌面高度 z |
|--------|------|---------|--------------|-----------|
| P0（默认） | `aruco_sample` | station_a | — | 0.785 |
| P1 | `igbt_module_plain` (material_spare_igbt) | station_a | 0.09×0.09×0.06 | 0.78 |
| P2 | `fixture_box_plain` (tooling_fixture_box) | tooling_zone | 0.16×0.12×0.10 | 0.80 |

**视觉组注意**：
- 当前默认抓取目标为 `aruco_sample`（通过 ROS 参数 `target_object` 配置，默认值在 `gripper_driver.py:DEFAULT_TARGET_OBJECT`）
- 可抓取物品的关键属性：`<static>` 未设置（非静态）、有 `<inertial><mass>`、有 `<surface><friction>`
- 桌面物品 z 坐标范围 0.75~0.80m（桌面高 0.75m + 物品半高）
- Station B 为放置目标区，保持空旷用于承接搬运来的物品（drop zone: x∈[-0.435, 1.035], y∈[-2.235, -1.165]）

### 2.3 未启用的备用模型

以下模型已创建但未在 lab.world 中实例化，可按需添加到场景：

| 模型目录 | 用途 |
|---------|------|
| `igbt_module_aruco` | 带 ArUco 标记的 IGBT 模块变体 |
| `reagent_bottle` | 试剂瓶 |
| `toolbox_yellow` | 黄色工具箱 |

---

## 三、导航系统状态

### 3.1 N1 任务：基础导航能力 — **已完成**

| 子项 | 状态 | 说明 |
|------|------|------|
| 底盘运动学链路 | DONE | `cmd_vel → mecanum_wheel_visualizer → mecanum_drive → /odom` |
| SLAM 静态地图 | DONE | `generate_map.py` 合成 300×300 px，origin (-7.5, -7.5)，覆盖 15×15m |
| AMCL 定位 | DONE | initial_pose 修正为 (4.50, -4.20, 0)，匹配 2× 环境 spawn 坐标 |
| Nav2 DWB 局部规划 | DONE | `min_vel_x=0.0`（禁止倒车），`min_speed_xy=0.08`（消除蠕动） |
| 单点导航 | DONE | `ros2 topic pub /task/instruction "{data: '去检测区'}"` 可导航到任意五站 |
| 巡航导航 | DONE | `"巡航所有工位"` 按固定路线遍历 6 站 |
| 机器人到位精度 | VERIFIED | 实际导航偏差 ~7.5cm（E2E 实测 station_a 路径） |

### 3.2 N2 任务：任意点位导航 — **已完成（contract + 参数层面），待实跑验证**

| 子项 | 状态 | 说明 |
|------|------|------|
| 5 站 waypoint 表 | DONE | 6 个 waypoint（5 作业站 + home），坐标互异 |
| 20 条有向路径 contract test | DONE | 5×4=20 路径全验证：距离 1.0~20.0m，不穿高压区围栏 |
| 路径统计表 | DONE | 见 `test_waypoints.py::test_routing_table_20_paths_statistics` |
| Waypoint 合法性 | DONE | 所有 waypoint 不在高压区围栏内，所有站台前留足安全距离 |
| 站名别名系统 | DONE | 支持中英文别名：A工位、检测区、工具区、老化区、B工位、起始点 |
| LLM 任务拆解 | DONE | `llm_enabled:=true` 支持自然语言 → 多站导航序列 |
| 单站导航指令 | DONE | `NAV_TO_STATION:<站>` → `ARRIVED:<站>` → `DONE` |
| 实跑 E2E 验证 | **TODO** | 需手动 `ros2 launch lab_cobot_bringup lab_cobot.launch.py skip_visual_dock:=true` 逐路径验证 |

### 3.3 导航参数配置（nav2_params.yaml）

| 参数 | 值 | 作用 |
|------|-----|------|
| `min_vel_x` | 0.0 | 禁止倒车，车头始终向前 |
| `min_speed_xy` | 0.08 | 最低线速度，消除蠕动感 |
| `trans_stopped_velocity` | 0.05 | 平动停止判定 |
| `xy_goal_tolerance` | 0.15 | 导航目标容差 |
| `yaw_goal_tolerance` | 0.15 | 朝向目标容差 |
| AMCL `initial_pose` | (4.50, -4.20, 0) | 匹配 2× 环境 home spawn |

### 3.4 精停 Docking 参数（mission_node.py）

| 参数 | 值 | 说明 |
|------|-----|------|
| `WORKTABLE_CLEARANCE` | 0.18m | 底盘前沿到桌面前沿的安全距离 |
| `STATION_DOCK_MAX_LINEAR_X/Y` | 0.20 | 精停最大线速度 |
| `STATION_DOCK_GAIN_X/Y` | 1.0 | 精停 P-controller 增益 |
| `STATION_DOCK_TOLERANCE_X/Y` | 0.08 | 精停到位容差 |
| 机械臂到物品距离 | ~0.87m | 在 UR5e 名义可达范围 0.85m 附近 |

### 3.5 跳过视觉精停（导航独立验证用）

```bash
# 启动时跳过 ArUco 视觉精停，导航到站即返回成功
ros2 launch lab_cobot_bringup lab_cobot.launch.py skip_visual_dock:=true
```

此参数用于在不依赖视觉/机械臂就绪的情况下独立验证 20 条导航路径。

---

## 四、系统架构

### 4.1 完整数据流

```text
/task/instruction
  → mission_node (task_planner[LLM拆解] → task_state_machine[状态机])
  → Nav2 AMCL/EKF + DWB
  → /cmd_vel → mecanum_wheel_visualizer(麦轮逆解)
  → /wheel_velocity_controller/commands → lab_cobot_mecanum_drive(正解积分→/odom)
  → aruco_detector(RGB-D solvePnP) → TF/PoseStamped
  → MoveIt 2 + pymoveit2
  → ContactGripperDriver → /gripper_position_controller/commands
  → lab_cobot_grasp_fix(几何封套 → fixed joint attach/detach)
  → /task/status
```

### 4.2 任务状态机

```
双工位搬运: NAV_TO_PICK → DETECT → PICK → NAV_TO_PLACE → PLACE → RETURN_HOME → DONE
单站导航:   NAV_TO_STATION:<站> → ARRIVED:<站> → DONE
巡航:       home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home
```

### 4.3 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 机器人 URDF/SRDF：麦轮底盘(0.55×0.50m)、立柱、UR5e、双指夹爪、激光、IMU、相机 |
| `lab_cobot_gazebo` | 五功能区 world（6 变体）、12 个模型、麦轮驱动/抓取插件、spawn 与控制器 |
| `lab_cobot_navigation` | Nav2 AMCL/EKF/DWB、静态地图、waypoints、`generate_map.py`、导航 launch |
| `lab_cobot_moveit` | UR5e MoveIt 2 配置、controller、move_group launch |
| `lab_cobot_perception` | ArUco 检测(RGB-D solvePnP)、YOLO-World 物体检测、TF/PoseStamped |
| `lab_cobot_manipulation` | pick/place 执行、MoveIt 调用、夹爪驱动、抓取序列 |
| `lab_cobot_bringup` | 一键 launch、任务状态机、LLM 任务拆解、mission 编排 |
| `pymoveit2` | vendored MoveIt 2 Python 接口 |

---

## 五、综合评判

### 5.1 环境模块

| 维度 | 评价 | 说明 |
|------|------|------|
| 场景完整性 | **良** | 5 功能区 + 9 种物品 + 4 张工作台 + 高压区围栏，覆盖实验室典型场景 |
| 物理保真度 | **中** | 底盘位姿积分走 SetWorldPose（非滚子接触动力学）；抓取走 fixed-joint（非摩擦力闭合）。对导航验证无影响，对操作验证有简化 |
| 可扩展性 | **优** | 6 个 world 变体（基础/actor/dark/reflective × 有无 actor），3 个备用模型可按需启用 |
| 地图质量 | **良** | 合成地图覆盖完整 15×15m，origin/provenance 已文档化 |

### 5.2 导航模块

| 维度 | 评价 | 说明 |
|------|------|------|
| N1 基础导航 | **DONE** | 底盘链路、地图、定位、DWB 全部就绪，单点导航 E2E 验证通过 |
| N2 任意点位 | **contract DONE / E2E TODO** | 20 路径 contract test 通过，参数调优完成，待实跑逐路径验证 |
| 到位精度 | **良** | ~7.5cm 导航偏差 + 0.08m 精停容差，满足机械臂操作要求 |
| 运动质量 | **良** | 禁止倒车 + 最低速度 0.08m/s，运动流畅无蠕动 |
| 代码质量 | **良** | 参数声明顺序 bug 已修复，getattr 安全访问模式统一，测试覆盖充分 |

### 5.3 风险与待办

| 优先级 | 事项 | 负责人建议 | 说明 |
|--------|------|-----------|------|
| P0 | 五区 E2E 实跑验证 | 导航 | 用 `skip_visual_dock:=true` 逐路径跑通 20 条 |
| P0 | 多物品抓取验证 | 操作+视觉 | 目前仅默认抓取 `aruco_sample`，需验证 `igbt_module_plain`、`fixture_box_plain` |
| P1 | LLM 模式 E2E | 导航+bringup | `llm_enabled:=true` 需 DeepSeek API key，离线 CI 无法测试 |
| P1 | 世界变体验证 | 环境 | lab_dark/reflective/actor 变体尚未全量回归 |
| P2 | Odin1 导航替换 | 导航 | 见 CLAUDE.md 2026-07-16 补充，保留 Nav2 接口，上游兼容接入 |
| P2 | SLAM 实跑地图 | 导航 | 当前为合成地图，实跑 slam_toolbox 建图可进一步提升定位精度 |

---

## 六、快速操作参考

### 构建与测试

```bash
cd ~/lab_cobot_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 定向回归（快速）
PYTEST_ADDOPTS='-p no:anyio' colcon test --packages-select lab_cobot_description lab_cobot_gazebo lab_cobot_navigation lab_cobot_bringup --event-handlers console_direct+
colcon test-result --verbose

# 20 路径 contract test
PYTEST_ADDOPTS='-p no:anyio' pytest src/lab_cobot_navigation/test/test_waypoints.py -v -k "path" --tb=short
```

### 启动与调试

```bash
source install/setup.bash

# 完整启动（含视觉精停）
ros2 launch lab_cobot_bringup lab_cobot.launch.py

# 导航独立验证（跳过视觉）
ros2 launch lab_cobot_bringup lab_cobot.launch.py skip_visual_dock:=true

# 单站导航指令（需 llm_enabled:=true）
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '去检测区'}"

# 巡航所有工位
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '巡航所有工位'}"

# 查看状态
ros2 topic echo /task/status
```

### 环境运行注意

- WSLg 下 launch 自动设置 D3D12/Qt 环境变量
- headless 结束 launch 时 MoveIt/rclpy 可能输出 SIGINT 噪声，以 `/task/status` 到 `DONE` 为准
- 麦轮底盘不受碰撞阻挡（SetWorldPose 位姿积分），里程计无轮地接触漂移
- 旧底盘链路（rover_twist_relay/planar_drive/odom_bridge，大底盘 0.83×0.75）已删除，备份于 tag `backup/old-chassis-planar-drive`

---

## 文档索引

- `docs/运行与验证.md` — 运行步骤、验证命令、常见问题
- `docs/superpowers/specs/` — 设计文档
- `docs/superpowers/plans/` — 实现计划（含五区导航设计）
- `src/lab_cobot_navigation/maps/map_provenance.yaml` — 地图来源与质量门
- `CLAUDE.md` — 开发工作区配置（不入仓库）

## 许可

Apache-2.0；第三方组件许可见 `THIRD_PARTY_LICENSES.md`。
