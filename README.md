# CS-202618 实验室移动协作机器人

面向实验室智能管理的移动协作机器人仿真系统。ROS 2 Humble + Gazebo Classic 11 + Nav2 + MoveIt 2 + ArUco 感知，麦克纳姆底盘 + UR5e + 平行双指夹爪，五功能区实验室场景。

赛题：CS-202618 中车株洲电力机车有限公司「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」。

---

## 当前状态 (2026-07-26)

**开发分支**: `feature/navigation`（领先 origin/feature/navigation 8 commits）

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
| **N3 动态障碍物感知** | **已部署** — actor 幽灵碰撞 + 局部 costmap 避让 |
| **N3 避让 E2E 测试** | **已添加**（需实跑: 3600s 超时） |
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
| `tooling_zone` | 工具区, 工装区 | **-3.70** | **-5.05** | π/2 (北) | +y | (-4.10, -2.30) |
| `aging_zone` | 老化区 | 0.20 | 3.20 | π/2 (北) | +y | (0.20, 4.20) |
| `station_b` | B工位, 工位B | 0.30 | -3.01 | π/2 (北) | +y | (0.30, -1.70) |

> **注意**：tooling_zone 的导航目标已从 (-4.10, -3.30) 调整为 (-3.70, -5.05)，配合走廊入口 → 工位前停的多段导航（见 N3 章节）。

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
| **IGBT 模块备件** | `material_spare_igbt` | `igbt_module_plain` | (-4.62, 3.92, 0.78, 0.38) | **是** | 导入网格: 游标卡尺 DAE，碰撞: 0.14×0.38×0.12m |
| **导热硅脂罐** | `material_grease_can` | `thermal_grease_can` | (-3.90, 3.96, 0.75, 0) | 否（道具） | 导入网格: 万用表 DAE，碰撞: 0.26×0.06×0.052m |

#### Tooling Zone（工具区）— 2 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **工装夹具盒** | `tooling_fixture_box` | `fixture_box_plain` | (-3.88, -2.04, 0.80, -0.28) | **是** | 导入网格: 活动扳手 DAE，碰撞: 0.12×0.34×0.20m |
| **手工工具** | `tooling_hand_tools` | `tooling_hand_tools` | (-4.36, -1.96, 0.75, 0.12) | 否（道具） | 导入网格: 钳子 DAE，碰撞: 0.12×0.34×0.048m |

#### Aging Zone（老化区）— 1 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **老化架** | `aging_rack` | `aging_rack` | (0.20, 4.26, 0.80, 0) | 否（道具） | 3 槽位(slot_left/mid/right) + 状态指示灯(绿/黄/红) |

#### Station B（B工位）— 1 件物品

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **PCB 测试夹具** | `board_test_fixture` | `pcb_test_fixture` | (0.02, -1.44, 0.75, 0.22) | 否（道具） | 导入网格: 电钻 DAE，碰撞: 0.32×0.22×0.10m |

#### Inspection Zone（检测区/高压区）— 2 件物品（地面，非桌面）

| 物品名 | Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 视觉特征 |
|--------|-------------|---------|---------------------|--------|---------|
| **高压探头套件** | `high_voltage_probe_kit` | `safety_probe_kit` | (4.04, 2.44, 0.0, -0.18) | 否（地面道具） | 导入网格: 螺丝刀 DAE，碰撞: 0.34×0.07×0.08m，**非静态** |
| **高压区围栏** | `high_voltage_zone` | `high_voltage_zone` | (4.36, 2.90, 0.0, 0.12) | 否（围栏） | 4 面墙 + 4 立柱，碰撞盒 2.0×1.68m |

### 2.2 可抓取物品汇总

| 优先级 | 物品 | 所在工位 | 碰撞盒尺寸(m) | 桌面高度 z |
|--------|------|---------|--------------|-----------|
| P0（默认） | `aruco_sample` | station_a | — | 0.785 |
| P1 | `igbt_module_plain` (material_spare_igbt) | station_a | 0.14×0.38×0.12 | 0.78 |
| P2 | `fixture_box_plain` (tooling_fixture_box) | tooling_zone | 0.12×0.34×0.20 | 0.80 |

### 2.3 模型升级：导入网格替代程序化基元

**本版本（2026-07-26）** 将全部 6 个物品模型的视觉表示从程序化几何基元替换为工业工具 DAE 网格，碰撞盒相应放大以匹配网格轮廓：

| 模型 | 导入网格 | 碰撞盒 (原 → 新) | 质量 (原 → 新) |
|------|---------|-----------------|---------------|
| `igbt_module_plain` | digital_caliper.dae | 0.09×0.09×0.06 → 0.14×0.38×0.12 | 0.25 → 0.96 |
| `fixture_box_plain` | adjustable_wrench.dae | 0.16×0.12×0.10 → 0.12×0.34×0.20 | 1.0 → 1.28 |
| `pcb_test_fixture` | drill.dae | 0.16×0.11×0.008 → 0.32×0.22×0.10 | 0.3 → 3.60 |
| `safety_probe_kit` | screwdriver.dae | 0.15×0.09×0.04 → 0.34×0.07×0.08 | *→ 0.64* |
| `thermal_grease_can` | meter_closed.dae | 圆柱 r0.028×h0.09 → 0.26×0.06×0.052 | 0.22 → 1.44 |
| `tooling_hand_tools` | pliers_closed.dae | 0.18×0.10×0.016 → 0.12×0.34×0.048 | 0.5 → 1.12 |

> **设计说明**：碰撞盒采用简化包围盒（AABB）而非精确网格碰撞，在保证物理交互的前提下维持计算效率。`safety_probe_kit` 同时移除了 `<static>true</static>`，使其可被碰撞影响。

### 2.4 未启用的备用模型

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
| Nav2 DWB 局部规划 | DONE | DWB 参数全链路调优（见 3.3） |
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

### 3.3 N3 任务：动态障碍物感知与避让 — **已完成（系统部署 + 测试添加），待实跑验证**

N3 为五区环境的虚拟测试工程师加入了一条沿实验室周界的闭环行走路径，并通过幽灵碰撞体 + 导航避让层实现了 LiDAR 可探测的动态障碍物避让。

| 子项 | 状态 | 说明 |
|------|------|------|
| Actor 幽灵碰撞 | DONE | 可见光透明圆柱(φ0.70×1.70m)与 actor 位姿同步，LiDAR 可扫到 |
| Local costmap 避让层 | DONE | 独立 `actor_obstacle_layer` 订阅 `/actor_ghost/obstacle_cloud` |
| Actor 周界路径 | DONE | 59s 闭环，~0.5m/s 均匀速度，覆盖实验室全部 4 条走廊 |
| cmd_vel 安全多路器 | DONE | `cmd_vel_safety_mux.py` 提供紧急刹车能力 |
| DWA 障碍物评分升级 | DONE | 添加 `BaseObstacle` critic（scale=10.0），`ObstacleFootprint` 提升至 10.0 |
| 避障 E2E 测试 | DONE | `test_dynamic_obstacle_avoidance.py`（3600s 超时，需实跑） |
| Actor 安全 contract 测试 | DONE | `test_actor_safety_contracts.py`（幽灵碰撞与 actor 位姿同步） |
| 任务状态 `/task/status` 增强 | DONE | TRANSIENT_LOCAL QoS，1Hz 周期重发 + 日志记录 |
| `nav_only` 纯导航模式 | DONE | 跳过收臂/回退，支持无机械臂状态下独立验证导航 |
| 多段导航 | DONE | tooling_zone 走走廊入口（x=1.90）→ 工位；aging_zone 走南入口（y=-3.80）→ 东走廊 |
| 导航超时提升 | DONE | 单次导航 60→240s，启动等待 120→240s，适配大范围路径 |
| 精停参数修正 | DONE | 安全切换距离 0.83→0.87m，收臂点 0.90→0.87m |
| WSL FastDDS 配置 | DONE | `fastdds_no_shm.xml` 禁用共享内存传输 |
| 定位 launch 重构 | DONE | 直接启动 map_server/AMCL/cmd_vel_mux，避免 nav2_bringup bond_timeout 竞态 |
| **实跑 N3 E2E 验证** | **TODO** | `ros2 launch lab_cobot_bringup lab_cobot.launch.py nav_only:=true` + actor 环境 |

### 3.4 导航参数配置（nav2_params.yaml）

| 参数 | 旧值 | 新值 | 作用 |
|------|------|------|------|
| `controller_frequency` | 10.0 | 20.0 | 控制频率翻倍，响应更细腻 |
| `max_vel_x` / `max_vel_y` | 0.35 / 0.25 | **0.55 / 0.30** | 最高直行/横移速度，提升巡航效率 |
| `max_speed_xy` | 0.35 | **0.55** | 合速度上限随 max_vel 提升 |
| `min_speed_xy` | 0.05 | **0.02** | 更低的蠕动门限，末端接近时更柔顺 |
| `acc_lim_x` / `acc_lim_y` | 2.5 / 2.5 | **1.5 / 1.5** | 加减速更平缓，减少底盘急动 |
| `sim_time` | 1.7s | **2.5s** | 轨迹仿真窗口延长，更好预见障碍物 |
| `vx_samples` / `vtheta_samples` | 20 / 21 | 17 / **25** | 转向采样更多，直行略减 |
| 局部 costmap 尺寸 | 5×5m | **7×7m** | 更大视野应对 actor 避让 |
| 局部 costmap 频率 | 5/2 Hz | **10/5 Hz** | 更新/发布频率翻倍 |
| 局部 costmap 层 | voxel + inflation | **+ actor_obstacle** | 新增动态障碍物感知 |
| 局部观测范围 | 2.5m | **5.0m** | LiDAR 观测距离翻倍 |
| Critic 权重 | 各种 32/24 | **统一 16.0** | 评分平衡重构，加速更协调 |
| BaseObstacle critic | — | **scale=10.0** | 新增障碍物基底层评分 |
| ObstacleFootprint scale | 0.02 | **10.0** | 障碍物避让权重大幅提升 |
| 全局 inflation_radius | 0.55m | **0.75m** | 膨胀半径扩大，路径离障碍更远 |
| 全局 cost_scaling_factor | 3.0 | **2.5** | 膨胀衰减略缓，路径更平滑 |
| velocity_smoother max | 0.5/0.25 | **0.55/0.30** | 与 DWB 对齐，去除限速瓶颈 |

### 3.5 精停 Docking 参数（mission_node.py）

| 参数 | 值 | 说明 |
|------|-----|------|
| `WORKTABLE_CLEARANCE` | 0.18m | 底盘前沿到桌面前沿的安全距离 |
| `STATION_DOCK_MAX_LINEAR_X/Y` | 0.20 | 精停最大线速度 |
| `STATION_DOCK_GAIN_X/Y` | 1.0 | 精停 P-controller 增益 |
| `STATION_DOCK_TOLERANCE_X/Y` | 0.08 | 精停到位容差 |
| `DOCK_SAFE_HANDOFF_MAX_X` | **0.87m** | 视觉精停→机械臂安全切换距离（原 0.83） |
| `PICK_NAV_HANDOFF_MAX_X` | **0.87m** | 导航→抓取的最大收臂切换距离（原 0.90→对齐 0.87） |
| 机械臂到物品距离 | ~0.87m | 在 UR5e 名义可达范围 0.85m 附近 |

### 3.6 纯导航模式（独立验证用）

```bash
# 启动时跳过视觉精停 + 跳过机械臂操作（nav_only）
ros2 launch lab_cobot_bringup lab_cobot.launch.py nav_only:=true

# 跳过视觉精停但保留机械臂（N2 兼容）
ros2 launch lab_cobot_bringup lab_cobot.launch.py skip_visual_dock:=true
```

`nav_only:=true` 模式下，mission_node 跳过全部机械臂操作（收臂、回退、go_home），仅验证导航路径。适合独立跑通 20 条路径 + N3 动态避让。

---

## 四、Actor 动态障碍物系统

### 4.1 系统架构

```text
lab_actor.world / lab_dark_actor.world / lab_reflective_actor.world
  ├── <actor name="test_engineer_actor">          ← Gazebo 骨骼动画角色
  │     └── trajectory: 59s 闭环周界巡逻 (~0.5m/s)
  └── <model name="actor_ghost_collision">        ← 幽灵碰撞体（静态模型）
        └── cylinder(φ0.70 × h1.70)：透明视觉 + LiDAR 可探测碰撞
              ↑ 位姿同步（actor_collision_shadow.py 订阅 actor 位姿更新幽灵位置）

actor_collision_shadow.py                        ← Python 节点
  ├── 订阅 /actor/test_engineer_actor/pose       ← Gazebo 发布的 actor 位姿
  └── 发布 /actor_ghost/obstacle_cloud           ← PointCloud2 → nav2 costmap 避让层

cmd_vel_safety_mux.py                            ← /cmd_vel 安全多路器
  └── 输入: cmd_vel_nav / cmd_vel_dock
      └── 输出: /cmd_vel (最终送底盘)
```

### 4.2 Actor 周界路径

59 秒闭环，经过实验室全部 4 条走廊和南侧通道：

```
起点(-5.80, 1.20) → 北走廊(-1.50, 2.10) → 西走廊(-1.80, 0.00)
→ 西南角(-1.80, -2.80) → 南走廊(0.50, -3.60) → 东走廊(3.50, -3.50)
→ 东南角(4.00, -3.80) → 南侧通道(2.00, -3.80, -2.00, -3.80)
→ 西南角(-5.00, -3.80) → 西走廊(-5.00, -1.00) → 回到起点
```

> **注意**：actor 路径经过 home 附近 (4.00, -3.80)，机器人导航从 home 出发时可能与 actor 相遇，验证 N3 避让能力。

### 4.3 三个 World 变体

| 世界文件 | 特点 | 适用场景 |
|---------|------|---------|
| `lab_actor.world` | 标准光照 + actor | 默认避障验证 |
| `lab_dark_actor.world` | 暗室 + actor | 低光照避障验证 |
| `lab_reflective_actor.world` | 反光地面 + actor | 反光条件避障验证 |

actor 相关节点（`actor_collision_shadow`、`obstacle_avoidance_metrics`）仅在 `enable_actor:=true`（即 actor world）时启动。

---

## 五、系统架构

### 5.1 完整数据流

```text
/task/instruction
  → mission_node (task_planner[LLM拆解] → task_state_machine[状态机])
  → Nav2 AMCL/EKF + DWB + actor_obstacle_layer
  → /cmd_vel_nav + /cmd_vel_dock
  → cmd_vel_safety_mux → /cmd_vel
  → mecanum_wheel_visualizer(麦轮逆解)
  → /wheel_velocity_controller/commands → lab_cobot_mecanum_drive(正解积分→/odom)
  → aruco_detector(RGB-D solvePnP) → TF/PoseStamped
  → MoveIt 2 + pymoveit2
  → ContactGripperDriver → /gripper_position_controller/commands
  → lab_cobot_grasp_fix(几何封套 → fixed joint attach/detach)
  → /task/status
```

### 5.2 任务状态机

```
双工位搬运:   NAV_TO_PICK → DETECT → PICK → NAV_TO_PLACE → PLACE → RETURN_HOME → DONE
单站导航:     NAV_TO_STATION:<站> → ARRIVED:<站> → DONE
巡航:         home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home
多段导航:     走廊入口/南入口 → 中间走廊 → 目标工位（tooling_zone/aging_zone 专用）
纯导航模式:   NAV_TO_STATION:<站> → ARRIVED:<站> → DONE（跳过收臂/回退）
```

### 5.3 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 机器人 URDF/SRDF：麦轮底盘(0.55×0.50m)、立柱、UR5e、双指夹爪、激光、IMU、相机 |
| `lab_cobot_gazebo` | 五功能区 world（6 变体）、12 个模型（含 DAE 网格）、麦轮驱动/抓取/碰撞阴影插件、导航安全多路器、spawn 与控制器 |
| `lab_cobot_navigation` | Nav2 AMCL/EKF/DWB、actor_obstacle_layer、静态地图、waypoints、`generate_map.py`、导航 launch（直接启动定位） |
| `lab_cobot_moveit` | UR5e MoveIt 2 配置、controller、move_group launch |
| `lab_cobot_perception` | ArUco 检测(RGB-D solvePnP)、YOLO-World 物体检测、TF/PoseStamped |
| `lab_cobot_manipulation` | pick/place 执行、MoveIt 调用、夹爪驱动、抓取序列 |
| `lab_cobot_bringup` | 一键 launch、任务状态机、LLM 任务拆解、mission 编排 |
| `pymoveit2` | vendored MoveIt 2 Python 接口 |

---

## 六、综合评判

### 6.1 环境模块

| 维度 | 评价 | 说明 |
|------|------|------|
| 场景完整性 | **优** | 5 功能区 + 9 种物品（6 个导入网格）+ 4 张工作台 + 高压区围栏 + 动态 actor |
| 物理保真度 | **良** | 底盘位姿积分走 SetWorldPose（非滚子接触动力学）；抓取走 fixed-joint（非摩擦力闭合）。物品碰撞盒放大匹配网格轮廓，`safety_probe_kit` 已改为非静态 |
| 可扩展性 | **优** | 6 个 world 变体 + 3 个备用模型 + 3 个 Python 脚本（碰撞阴影/避障指标/安全多路） |
| 地图质量 | **良** | 合成地图覆盖完整 15×15m，origin/provenance 已文档化 |

### 6.2 导航模块

| 维度 | 评价 | 说明 |
|------|------|------|
| N1 基础导航 | **DONE** | 底盘链路、地图、定位、DWB 全部就绪，单点导航 E2E 验证通过 |
| N2 任意点位 | **contract DONE / E2E TODO** | 20 路径 contract test 通过，参数调优完成，待实跑逐路径验证 |
| **N3 动态避让** | **系统 DONE / E2E TODO** | actor_ghost + costmap 避让层 + 避障指标 + 安全多路器全部就绪 |
| 到位精度 | **良** | ~7.5cm 导航偏差 + 0.08m 精停容差，满足机械臂操作要求 |
| 运动质量 | **优** | 速度提升至 0.55m/s，加减速平滑（acc=1.5），无蠕动（min_speed=0.02） |
| 代码质量 | **优** | 模型六站统一 DAE 网格、actor 周界路径协调设计、定位 launch 重构消除竞态、TRANSIENT_LOCAL QoS |

### 6.3 风险与待办

| 优先级 | 事项 | 负责人建议 | 说明 |
|--------|------|-----------|------|
| P0 | N3 动态避让 E2E 实跑 | 导航 | 用 `nav_only:=true` + actor world 验证避让，超时 3600s |
| P0 | N2 五区 E2E 实跑验证 | 导航 | 用 `skip_visual_dock:=true` 逐路径跑通 20 条 |
| P0 | Actor 与机器人 concurrence 安全 | 导航+环境 | actor 在家门口经过时验证刹停与重新规划 |
| P0 | 多物品抓取验证 | 操作+视觉 | 目前仅默认抓取 `aruco_sample`，需验证 `igbt_module_plain`、`fixture_box_plain` |
| P1 | LLM 模式 E2E | 导航+bringup | `llm_enabled:=true` 需 DeepSeek API key，离线 CI 无法测试 |
| P1 | 世界变体验证 | 环境 | lab_dark/reflective/actor 变体尚未全量回归 |
| P1 | 避障指标量化 | 导航 | `obstacle_avoidance_metrics.py` 已添加，需实跑收集 baseline |
| P2 | Odin1 导航替换 | 导航 | 见 CLAUDE.md 2026-07-16 补充，保留 Nav2 接口，上游兼容接入 |
| P2 | SLAM 实跑地图 | 导航 | 当前为合成地图，实跑 slam_toolbox 建图可进一步提升定位精度 |

---

## 七、测试清单

### 7.1 原有 Contract 测试

```bash
# 20 路径 contract test（离线，无需 ROS 运行）
PYTEST_ADDOPTS='-p no:anyio' pytest src/lab_cobot_navigation/test/test_waypoints.py -v -k "path" --tb=short

# Nav2 参数 consistency
PYTEST_ADDOPTS='-p no:anyio' pytest src/lab_cobot_navigation/test/test_nav2_params.py -v --tb=short

# 精停 docking 计算
PYTEST_ADDOPTS='-p no:anyio' pytest src/lab_cobot_bringup/test/test_mission_docking.py -v --tb=short

# 任务规划逻辑
PYTEST_ADDOPTS='-p no:anyio' pytest src/lab_cobot_bringup/test/test_mission_planning.py -v --tb=short
```

### 7.2 新增 N3 测试

| 测试文件 | 测试项 | 类型 | 说明 |
|---------|--------|------|------|
| `test_actor_safety_contracts.py` | 幽灵碰撞与 actor 位姿同步 | 离线 Contract | 验证 actor 与幽灵模型的 yaw/xy 对齐 |
| `test_dynamic_obstacle_avoidance.py` | 5 次 T1 动态避让 E2E | **需实跑 (3600s)** | actor 周界行走 + 机器人 5 往返完整避让 |
| `test_t1_actor_interference_acceptance.py` | T1 干扰接受度 | **需实跑** | 机器人与 actor 同时沿固定路径运行 |
| `test_mission_planning.py` | 多段导航/Waypoint QoS | 离线 Contract | tooling/aging 多段路径、TRANSIENT_LOCAL QoS |
| `test_mission_docking.py` | 视觉精停安全线 | 离线 Contract | 在 actor 出现时 dock 安全切换 |
| `test_nav2_params.py` | actor_cloud/避让参数 | 离线 Contract | actor_obstacle_layer 配置、BaseObstacle critic |

### 7.3 定向回归（快速）

```bash
cd ~/lab_cobot_ws && source /opt/ros/humble/setup.bash
PYTEST_ADDOPTS='-p no:anyio' colcon test --packages-select lab_cobot_description lab_cobot_gazebo lab_cobot_navigation lab_cobot_bringup --event-handlers console_direct+
colcon test-result --verbose
```

---

## 八、快速操作参考

### 构建与测试

```bash
cd ~/lab_cobot_ws && source /opt/ros/humble/setup.bash
colcon build --symlink-install

# 定向回归（快速）
PYTEST_ADDOPTS='-p no:anyio' colcon test --packages-select lab_cobot_description lab_cobot_gazebo lab_cobot_bringup --event-handlers console_direct+
colcon test-result --verbose

# 20 路径 contract test
PYTEST_ADDOPTS='-p no:anyio' pytest src/lab_cobot_navigation/test/test_waypoints.py -v -k "path" --tb=short
```

### 启动与调试

```bash
source install/setup.bash

# 完整启动（含视觉精停）
ros2 launch lab_cobot_bringup lab_cobot.launch.py

# 导航独立验证（跳过视觉，含机械臂）
ros2 launch lab_cobot_bringup lab_cobot.launch.py skip_visual_dock:=true

# 纯导航模式（跳过全部机械臂操作）
ros2 launch lab_cobot_bringup lab_cobot.launch.py nav_only:=true

# 启用 actor 动态障碍物环境
ros2 launch lab_cobot_bringup lab_cobot.launch.py world:=lab_actor.world

# 暗室 actor 环境
ros2 launch lab_cobot_bringup lab_cobot.launch.py world:=lab_dark_actor.world

# 单站导航指令（需 llm_enabled:=true）
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '去检测区'}"

# 巡航所有工位
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '巡航所有工位'}"

# 查看状态（支持 TRANSIENT_LOCAL，late-join 可收到）
ros2 topic echo /task/status
```

### 参数快速参考

| Launch 参数 | 默认 | 说明 |
|-------------|------|------|
| `skip_visual_dock` | false | 跳过 ArUco 视觉精停 |
| `nav_only` | false | 纯导航模式（跳过所有机械臂操作） |
| `enable_actor` | false | 启用 actor 动态障碍物（由 world 选择决定） |
| `llm_enabled` | false | 启用 LLM 自然语言任务拆解 |

### 环境运行注意

- WSLg 下 launch 自动设置 D3D12/Qt 环境变量
- WSL 下若遇到 DDS 共享内存问题，FastDDS 已配置 `fastdds_no_shm.xml` 禁用 SHM
- headless 结束 launch 时 MoveIt/rclpy 可能输出 SIGINT 噪声，以 `/task/status` 到 `DONE` 为准
- 麦轮底盘不受碰撞阻挡（SetWorldPose 位姿积分），里程计无轮地接触漂移
- Actor 幽灵碰撞体为透明圆柱，costmap 可见但 Gazebo 可视化为不可见
- N3 避让需要 actor world（标准/暗室/反光），普通 `lab.world` 不含 actor
- 旧底盘链路（rover_twist_relay/planar_drive/odom_bridge，大底盘 0.83×0.75）已删除，备份于 tag `backup/old-chassis-planar-drive`

---

## 文档索引

- `docs/运行与验证.md` — 运行步骤、验证命令、常见问题
- `docs/superpowers/specs/` — 设计文档
- `docs/superpowers/plans/` — 实现计划（含五区导航设计、N3 动态障碍物、物品模型升级）
- `src/lab_cobot_navigation/maps/map_provenance.yaml` — 地图来源与质量门
- `CLAUDE.md` — 开发工作区配置（不入仓库）

## 许可

Apache-2.0；第三方组件许可见 `THIRD_PARTY_LICENSES.md`。
