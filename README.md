# CS-202618 实验室移动协作机器人

面向实验室智能管理的移动协作机器人仿真系统。当前版本整合导航、稳定抓取和视觉
双后端三条开发链，三方代码基线为 `b09171e`。

- **ROS 2 Humble + Gazebo Classic 11**，Nav2 导航 + MoveIt 2 操作
- **麦克纳姆底盘**（0.55×0.50m）+ **UR5e** + 平行双指夹爪
- **五功能区、六工位** 14×14m 实验室场景，支持巡航、动态人员避让和 A→B 搬运
- **双深度学习视觉后端**：默认 diagnostic YOLO-World，可切换 eight_class YOLO +
  腕部 RGB-D 点云定位
- 赛题：CS-202618 中车株洲「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」

---

## 当前能力与边界

| 子系统 | 已实现 | 当前边界 |
|---|---|---|
| 导航 | AMCL + EKF + DWB、六工位导航、固定巡航、actor 动态避让 | 底盘施加层是有界 `SetWorldPose` 位姿积分，不是麦轮滚子接触动力学 |
| 操作 | MoveIt 规划、台面/持物碰撞盒、触觉步进闭合、A→B 抓放返航 | 抓取由接触门控后的 fixed joint 保持，不是真实摩擦力或力反馈 |
| 视觉 | bench/腕部 ArUco；diagnostic 与 eight_class 两套 DL 后端启动时互斥切换 | mission 抓取仍使用 ArUco 位姿；eight_class 当前只可靠输出 XYZ |
| 任务 | 搬运、单站导航、全工位巡航、失败清理、可选 LLM/语音入口 | 默认只验证 `aruco_sample` 抓取，其余七类只声明检测和定位 |

## 对外接口

**任务指令（输入）**：
```bash
# 单站导航
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '去检测区'}"

# A→B 搬运
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '把样件从A工位搬运到B工位，然后返回原点'}"

# 全工位巡航
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '巡航所有工位'}"
```

**任务状态（输出）**：`/task/status`（`std_msgs/String`，TRANSIENT_LOCAL QoS，
1Hz），late joiner 可收到最新状态。

**指令别名**：`物料区` `工装工具区` `板卡测试台` `老化实验台` `高压试验区` `A工位` `B工位` `检测区` `工具区` `工装区` `老化区` `起始点` `home` `station_a` `station_b` `inspection_zone` `tooling_zone` `aging_zone`

| 话题 | 类型 | 说明 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | 底盘最终速度指令 |
| `/odom` | `nav_msgs/Odometry` | mecanum drive 插件积分里程计 |
| `/perception/objects` | `vision_msgs/Detection3DArray` | 当前所选 DL 后端的互斥标准输出 |
| `/perception/target_pose` | `geometry_msgs/PoseStamped` | eight_class 后端的单目标输出，尚未接入 mission |

---

## 快速启动

### 环境

- Ubuntu 22.04 / ROS 2 Humble / Gazebo Classic 11
- Nav2、MoveIt 2、robot_localization、slam_toolbox、gazebo_ros2_control
- Python 3.10、OpenCV/cv_bridge、NumPy
- DL/语音运行环境已验证版本：`ultralytics==8.4.90`、`open3d==0.19.0`、
  `torch==2.12.1+cu130`、`faster_whisper==1.2.1`

模型权重不进入 Git，也不会在测试中下载：

| 后端 | 默认外部权重路径 |
|---|---|
| diagnostic | `~/lab_cobot_models/yolo_world_lab_slim.pt` |
| eight_class | `~/lab_cobot_models/lab_cobot_eight_class.pt` |

### 构建

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
export PYTEST_ADDOPTS='-p no:anyio'
colcon build --symlink-install
source install/setup.bash
```

### 全栈启动

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

默认使用 `vision_backend:=diagnostic`，同时启动 Nav2、MoveIt、ArUco、触觉抓取、
mission、G4 接触记录和 G5 动态障碍结果链。

### 视觉后端

```bash
# 默认：bench RGB-D + YOLO-World/点云聚类
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  vision_backend:=diagnostic

# 八类：腕部 RGB-D + 八类 YOLO + 有组织点云逐像素定位
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  vision_backend:=eight_class

# 八类后端固定使用 CPU
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  vision_backend:=eight_class dl_device:=cpu
```

两套 DL 后端都发布 `/perception/objects`，主 launch 保证二者互斥。
`launch_perception:=false` 会关闭全部感知；`use_dl_perception:=false` 只关闭 DL
后端，ArUco 链仍可保留。

### 常用变体

```bash
# 导航独立验证（跳过视觉精停，保留机械臂）
ros2 launch lab_cobot_bringup lab_cobot.launch.py skip_visual_dock:=true

# 纯导航模式（跳过所有机械臂操作 — 推荐导航独立调试用）
ros2 launch lab_cobot_bringup lab_cobot.launch.py nav_only:=true

# 带动态人员避障
ros2 launch lab_cobot_bringup lab_cobot.launch.py enable_actor:=true

# 暗室 / 反光地面
ros2 launch lab_cobot_bringup lab_cobot.launch.py lighting_profile:=dark
ros2 launch lab_cobot_bringup lab_cobot.launch.py lighting_profile:=reflective

# Headless（CI / 批量测试）
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false

# 关闭 G4/G5 结果记录旁路
ros2 launch lab_cobot_bringup lab_cobot.launch.py launch_g4g5_results:=false

```
> **注意**: 不支持单独启动 world.launch.py 或 navigation.launch.py。
> 环境查看和导航调试都通过总 bringup 配合参数实现：
> ```bash
> # 仅 Gazebo 环境（关闭导航/操作/感知）
> ros2 launch lab_cobot_bringup lab_cobot.launch.py \
>   gui:=true launch_navigation:=false launch_moveit:=false \
>   launch_perception:=false launch_mission:=false
> ```

### Launch 参数速查

| 参数 | 默认 | 说明 |
|---|---|---|
| `gui` | true | Gazebo GUI |
| `use_rviz` | false | RViz 导航可视化 |
| `skip_visual_dock` | false | true=用地图坐标停靠，false=ArUco 视觉精停 |
| `nav_only` | false | true=跳过所有机械臂收臂/回退/go_home |
| `enable_actor` | false | true=启动人员幽灵碰撞 + 动态避障 |
| `lighting_profile` | normal | `normal`、`dark` 或 `reflective` |
| `llm_enabled` | false | true=DeepSeek 自然语言→导航序列（需 API key） |
| `launch_navigation` | true | 启动 Nav2 导航栈 |
| `launch_moveit` | true | 启动 MoveIt 2 |
| `launch_perception` | true | 启动 ArUco 与所选 DL 感知 |
| `use_dl_perception` | true | 启动所选 DL 后端 |
| `vision_backend` | diagnostic | `diagnostic` 或 `eight_class` |
| `dl_device` | auto | `auto`、`cpu` 或 CUDA device id |
| `eight_class_model_path` | `~/lab_cobot_models/lab_cobot_eight_class.pt` | 八类权重路径 |
| `target_object` | aruco_sample | 任务与单目标输出的目标标签 |
| `use_wrist_detect` | true | DETECT 阶段腕相机拍照位检测 |
| `use_refine_detect` | true | PICK 悬停后的腕相机精修 |
| `use_tactile_grasp` | true | 使用触觉步进闭合 |
| `require_finger_contact` | true | attach 必须左右指均接触；须与上一参数同值 |
| `use_planning_scene_obstacles` | true | 注入台面和持物碰撞盒 |
| `launch_mission` | true | 启动任务编排 |
| `launch_g4g5_results` | true | 启动 G4 接触记录、G5 障碍桥与终态汇总 |

---

## 五功能区环境

### 布局（14×14m）

```
         +y (北)
         |
    [station_a]         [aging_zone]        [inspection_zone]
    (-4.30, 3.80)       (0.20, 4.20)        (4.36, 1.42)
     物料区              板卡测试台            高压试验区
     桌面: 1.6×1.2m       桌面: 1.6×1.2m       地面高压区 + 围栏
     物品: 5 件           物品: 2 件             物品: 1 件
         |                    |                    |
    -----+--------------------+--------------------+-----> +x (东)
         |                    |                    |
    [tooling_zone]                            [home]
    (-4.10, -2.30)                            (4.50, -4.20)
     工装工具区                                发车/归位区
     桌面: 1.6×1.2m
     物品: 5 件
         |                    |
         |              [station_b]
         |              (0.30, -1.70)
         |              老化实验台
         |              桌面: 1.6×1.2m
         |              物品: 15 件 (2×试管架 + 9×玻璃试管 + 2×烧杯 + 锥形瓶/量筒)
         |
         -y (南)
```

所有工作台：**1.6m(x) × 1.2m(y) × 0.75m(z)**（桌面高 0.75m）。

### 工位命名与导航命令速查（交接用）

> 规范键是代码硬编码的唯一标识，**不可修改**；中文名是用户层别名，可扩展。
> 完整别名见 `waypoints.py` 的 `_STATION_ALIASES`。

| 规范键 | 中文名 | 物理桌位置 (x,y) | 桌面物品（件） | 导航命令示例 |
|---|---|---|---|---|
| `station_a` | 物料区 | (-4.30, 3.80) | ArUco 主样件 + 4 彩色方块（5） | `去物料区` / `去A工位` |
| `tooling_zone` | 工装工具区 | (-4.10, -2.30) | 扳手/钳子/电钻/螺丝刀/卡尺（5） | `去工装工具区` / `去工具区` |
| `aging_zone` | 板卡测试台 | (0.20, 4.20) | 卡槽架 + 板卡（2） | `去板卡测试台` |
| `station_b` | 老化实验台 | (0.30, -1.70) | 2×试管架 + 9×玻璃试管 + 2×烧杯 + 锥形瓶/量筒（15） | `去老化实验台` / `去B工位` |
| `inspection_zone` | 高压试验区 | (4.36, 1.42) | 围栏 + 地面警示（1） | `去高压试验区` / `去检测区` |
| `home` | 起始点 / 归位区 | (4.50, -4.20) | — | `回家` / `去起始点` |

导航命令统一通过 `/task/instruction` 话题（`std_msgs/String`）下发：

```bash
# 单站导航（去任意工位，支持 导航到/前往/移动到/去 前缀）
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '去物料区'}"
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '去板卡测试台'}"

# 全工位巡航
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '巡航所有工位'}"

# 回家
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '回家'}"
```

### 机器人路线表

> **唯一运行时来源**: `src/lab_cobot_navigation/lab_cobot_navigation/waypoints.py` 中的
> `STATION_SPECS`。本表由代码生成，不要手改 README 中的坐标。

| 站 | Nav2 目标 (x, y, yaw) | 停靠位姿 (x, y, yaw) | 接近侧 | 朝向 | 段数 |
|---|---|---|---|---|---|---|
| `home` | (4.50, -4.20, 0) | (4.50, -4.20, 0) | none | 东 | 1 |
| `station_a` | (-4.30, 2.57, π/2) | (-4.30, 2.74, π/2) | south | 北 | 1 |
| `inspection_zone` | (4.36, 1.42, π/2) | (4.36, 1.42, π/2) | none | 北 | 2 |
| `tooling_zone` | (-4.10, -3.23, π/2) | (-4.10, -3.35, π/2) | south | 北 | 2 |
| `aging_zone` | (0.20, 2.97, π/2) | (0.20, 3.15, π/2) | south | 北 | 3 |
| `station_b` | (0.30, -2.63, π/2) | (0.30, -2.75, π/2) | south | 北 | 1 |

> Nav2 目标 = 导航 staging 点；停靠位姿 = 闭环精停终点（含 clearance）。
> inspection/tooling/aging 走多段导航（走廊入口 → 工位前停），
> 坐标见 `STATION_SPECS[站名].nav_legs`。
> 五站 yaw 统一 π/2（朝北），home 为 0（朝东）。

### 巡航路线

```
home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home
```

```

---

## 各工位物品清单（视觉/操作组协作参考）

### 物料区（station_a，5 件）

| Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 说明 |
|---|---|---|---|---|
| `aruco_sample` | `aruco_sample` | (-4.16, 3.46, 0.785, 0.10) | **是（默认目标）** | ArUco 主样件 (id 0/1) |
| `material_cube_red` | `material_cube_red` | (-4.50, 3.30, 0.785, 0) | 是（物理） | 彩色物料方块, ArUco id=2 |
| `material_cube_green` | `material_cube_green` | (-4.30, 3.30, 0.785, 0) | 是（物理） | 彩色物料方块, ArUco id=3 |
| `material_cube_blue` | `material_cube_blue` | (-4.50, 3.52, 0.785, 0) | 是（物理） | 彩色物料方块, ArUco id=4 |
| `material_cube_yellow` | `material_cube_yellow` | (-4.30, 3.52, 0.785, 0) | 是（物理） | 彩色物料方块, ArUco id=5 |

### 工装工具区（tooling_zone，5 件）

| Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 说明 |
|---|---|---|---|---|
| `tooling_fixture_box` | `fixture_box_plain` | (-3.88, -2.04, 0.80, -0.28) | 是（物理） | 活动扳手 |
| `tooling_hand_tools` | `tooling_hand_tools` | (-4.36, -1.96, 0.75, 0.12) | 是（物理） | 手钳 |
| `board_test_fixture` | `pcb_test_fixture` | (-4.70, -2.60, 0.75, 0.05) | 是（物理） | 电钻（自原板卡桌移入） |
| `high_voltage_probe_kit` | `safety_probe_kit` | (-4.30, -2.60, 0.75, -0.15) | 是（物理） | 螺丝刀（自高压区地面移入） |
| `material_spare_igbt` | `igbt_module_plain` | (-3.62, -2.60, 0.81, 0.38) | 是（物理） | 数字卡尺（保留，自物料区移入） |

### 板卡测试台（aging_zone，2 件）

| Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 说明 |
|---|---|---|---|---|
| `aging_rack` | `aging_rack` | (0.20, 4.26, 0.80, 0) | 否（static 道具） | 卡槽架，3 槽位 + 状态灯 |
| `pcb_board` | `pcb_board` | (0.55, 4.30, 0.75, 0) | 是（物理） | 板卡，带金手指，自立可夹 |

### 老化实验台（station_b，15 件）

> 桌面按两排布局：两架并拢于 y=-1.85（间隙 0.05m，便于机械臂单点够取），
> 玻璃器皿于 y=-1.95 左右两簇，架前正中让空；A→B 放件区保持为空。

| Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 说明 |
|---|---|---|---|---|
| `test_tube_rack_1` | `test_tube_rack` | (0.48, -1.85, 0.75, 0) | 否（static 道具） | 木质圆孔试管架，5 孔，插满 5 管 |
| `test_tube_rack_2` | `test_tube_rack` | (0.12, -1.85, 0.75, 0) | 否（static 道具） | 木质圆孔试管架，5 孔，中槽留空（转移任务目标） |
| `test_tube_1`…`test_tube_5` | `test_tube` | (0.36/0.42/0.48/0.54/0.60, -1.85, 0.762, 0) | 是（物理） | 真玻璃试管，架 1 槽内 |
| `test_tube_6`…`test_tube_9` | `test_tube` | (0.00/0.06/(空)/0.18/0.24, -1.85, 0.762, 0) | 是（物理） | 真玻璃试管，架 2 槽内（中槽空） |
| `beaker_1` | `beaker` | (0.72, -1.95, 0.75, 0) | 是（物理） | 玻璃烧杯（架右） |
| `beaker_2` | `beaker` | (-0.15, -1.95, 0.75, 0) | 是（物理） | 玻璃烧杯（架左） |
| `erlenmeyer_flask` | `erlenmeyer_flask` | (0.88, -1.95, 0.75, 0) | 是（物理） | 玻璃锥形瓶 |
| `graduated_cylinder` | `graduated_cylinder` | (-0.30, -1.95, 0.82, 0) | 是（物理） | 玻璃量筒 |

> 已删除：`reagent_rack`、`reagent_bottle_1..4`（旧试剂架 + 蓝色试剂瓶，替换为高保真
> 玻璃试管架 + 试管）。试管外径 0.022m < 夹爪开度 0.16m，物理可抓；支持「从架 1 取
> 管放入架 2 空槽」的试管转移任务（环境侧已就绪，任务接入属操作/感知模块）。

### 高压试验区（inspection_zone，1 件）

| Gazebo 实体名 | 模型目录 | 位姿 (x, y, z, yaw) | 可抓取 | 说明 |
|---|---|---|---|---|
| `high_voltage_zone` | `high_voltage_zone` | (4.36, 2.90, 0.0, 0.12) | 否（static 道具） | 围栏，4 墙 + 4 柱 + 地面警示 |

> 已删除：`material_grease_can`（导热硅脂罐）。

### 抓取范围

当前 URDF 中 `lab_cobot_grasp_fix` 的候选表只包含 `aruco_sample`。环境侧已保证其余
可抓取物（彩色方块、工具、玻璃试管/烧杯/锥形瓶/量筒、板卡）物理可抓（动态 + 碰撞 +
惯量 + 尺寸在夹爪开度内），但把新物体接入任务抓取链（感知标签、attach 桥、grasp_fix
候选）属于感知/操作模块，需另行适配。试管转移任务以 `target_object=test_tube_X`
传参接入即可抓取。

检测后端八类标签（`image_pkg`，硅脂罐已从环境中移除，标签保留但不产生检测）：`material_spare_igbt`、
`aruco_sample`、`material_grease_can`、`aging_rack`、`board_test_fixture`、
`tooling_fixture_box`、`tooling_hand_tools`、`high_voltage_probe_kit`。

### ArUco ID 分配（DICT_4X4_50）

| 对象 | ArUco ID | 用途 |
|---|---|---|
| `aruco_sample` 主样件 | 0（front）/ 1（top） | 主抓取目标；腕相机默认检测 `marker_id=1` |
| `material_cube_red` | 2 | 红色物料方块 |
| `material_cube_green` | 3 | 绿色物料方块 |
| `material_cube_blue` | 4 | 蓝色物料方块 |
| `material_cube_yellow` | 5 | 黄色物料方块 |

> **重要**：彩色方块必须用 ID 2–5，**不得复用 0/1**——主抓取链的腕相机用
> `wrist_marker_id=1` 定位 `aruco_sample`，复用会撞码。

---

## 导航系统

### 数据流

```
/task/instruction → mission_node（LLM拆解 → 状态机）
  → Nav2（AMCL + EKF + DWB）
  → /cmd_vel_nav + /cmd_vel_dock
  → cmd_vel_safety_mux → /cmd_vel
  → mecanum_wheel_visualizer（麦轮逆解）
  → /wheel_velocity_controller/commands
  → lab_cobot_mecanum_drive（正解积分 → /odom）
  → bench/腕部 ArUco（mission 抓取位姿）
  → diagnostic 或 eight_class（互斥发布 /perception/objects）
  → MoveIt 2（台面碰撞盒 + 持物附着盒）
  → ContactGripperDriver（步进闭合）
  → lab_cobot_grasp_fix（几何封套 + 双指接触门控）
  → /task/status
```

### 状态机

```
搬运：NAV_TO_PICK → DETECT → PICK → NAV_TO_PLACE → PLACE → RETURN_HOME → DONE
单站：NAV_TO_STATION:<站> → ARRIVED:<站> → DONE
巡航：home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home
纯导航：NAV_TO_STATION:<站> → ARRIVED:<站> → DONE（跳过收臂/回退）
```

### 视觉双后端

| 项目 | diagnostic（默认） | eight_class |
|---|---|---|
| ROS 包 | `lab_cobot_perception` | `image_pkg` |
| 相机 | bench RGB-D | wrist RGB-D |
| 模型 | YOLO-World slim | 项目八类 YOLO |
| 三维方法 | 点云分割、聚类、2D 分类关联、ArUco 门控 | 2D 框与同时间戳有组织点云逐像素关联 |
| 标准输出 | `/perception/objects` | `/perception/objects` |
| 单目标输出 | — | `/perception/target_pose` |
| 已验证边界 | 聚类尺寸和三维语义 | XYZ；姿态为单位四元数，未估计真实朝向和 bbox 尺寸 |

后端只支持在 launch 启动时选择；运行中切换需要停止当前 launch 后重新启动。

### 导航栈组件

| 组件 | 说明 |
|---|---|
| 底盘链路 | `cmd_vel → mecanum_wheel_visualizer → wheel_velocity_controller → mecanum_drive → /odom` |
| AMCL | 800 粒子，激光匹配，initial_pose=(4.50, -4.20, 0) |
| EKF | robot_localization 融合 odom + IMU（IMU 退化回退 odom-only 未验收） |
| DWB | 20Hz 控制，max_vel_x=0.55m/s，sim_time=2.5s |
| 代价地图 | 全局 15×15m + 局部 7×7m，voxel + inflation 二层；actor ghost 通过 LiDAR scan 进入 voxel 层 |
| Recovery | BT/behavior_server 配置已部署，spin/back_up/wait 行为未经 LiDAR/costmap 阻断实验验证 |
| 地图 | `map.pgm` (300×300px, 0.05m/pixel), origin=(-7.5, -7.5) |

### 动态避障（N3）

人员通过 `actor_ghost_collision`（透明圆柱 φ0.70×1.70m）进入局部代价地图，DWB 实时绕行或等待。`cmd_vel_safety_mux` 提供紧急刹车通道。采用"反应式安全避让 + 规划式重规划"双层：反应式层 20 Hz（50 ms 周期）同步下发避让指令，**算法响应延时 50 ms ≤ 200 ms**（规划效率达标）。

| 参数 | 值 |
|---|---|
| ghost 碰撞云半径 | 0.60m |
| 安全避让启动距离 | 2.00m（0.50 m/s 远离） |
| 强制避让距离 | 1.20m（0.60 m/s） |
| actor 巡逻周期 | 59s，13 路径点，覆盖全部走廊 |

### 导航关键参数

| 参数 | 值 | 说明 |
|---|---|---|
| `max_vel_x` / `max_vel_y` | 0.55 / 0.30 m/s | 最高线速度 / 横移 |
| `acc_lim_x` / `acc_lim_y` | 1.5 / 1.5 m/s² | 加减速 |
| `controller_frequency` | 20.0 Hz | DWB 控制频率 |
| `min_speed_xy` | 0.02 m/s | 蠕动门限 |
| 局部 costmap 尺寸 | 7×7m | 障碍感知窗口 |
| 全局 inflation_radius | 0.55m | 路径安全距离 |
| 局部 inflation_radius | 0.30m | 局部避障距离（≥ 内切半径 0.29m） |
| `STATION_DOCK_TOLERANCE_X/Y` | 0.08m | 精停到位容差 |

---

## 精停 Docking（操作组协作参考）

导航到位后，可选用 ArUco 视觉精停进行厘米级对准。跳过视觉精停时，导航到位偏差约 7.5cm。

| 参数 | 值 | 说明 |
|---|---|---|
| `DOCK_SAFE_HANDOFF_MAX_X` | 0.87m | 视觉精停→机械臂安全切换距离 |
| `PICK_NAV_HANDOFF_MAX_X` | 0.87m | 导航→抓取收臂切换距离 |
| `WORKTABLE_CLEARANCE` | 0.18m | 底盘前沿到桌面前沿安全距离 |

> 机械臂到桌面物品距离约 0.87m，在 UR5e 名义可达范围 0.85m 附近。

---

## 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 一体化 URDF/SRDF、麦克纳姆底盘、UR5e、夹爪和传感器 |
| `lab_cobot_gazebo` | 五区 world（6 变体）、模型、底盘/抓取插件、actor 和安全多路器 |
| `lab_cobot_navigation` | Nav2、AMCL、EKF、地图、waypoints、SLAM 和动态避让配置 |
| `lab_cobot_moveit` | UR5e MoveIt 2、controller、规划场景初始化 |
| `lab_cobot_perception` | bench/腕部 ArUco 与默认 diagnostic YOLO-World 点云后端 |
| `image_pkg` | 八类 YOLO、腕部 RGB-D 有组织点云和三维目标定位 |
| `lab_cobot_manipulation` | pick/place、夹爪驱动、持物监控、G4/G5 记录与机械臂动态障碍 |
| `lab_cobot_bringup` | 分阶段一键 launch、任务状态机、LLM/语音入口和结果汇总 |
| `pymoveit2` | vendored MoveIt 2 Python 接口；不要修改 |

---

## 验证

视觉移植与 bringup 回归：

```bash
source /opt/ros/humble/setup.bash
export PYTEST_ADDOPTS='-p no:anyio'
colcon build --symlink-install --packages-select image_pkg lab_cobot_bringup
source install/setup.bash

colcon test --packages-select image_pkg --event-handlers console_direct+
colcon test --packages-select lab_cobot_bringup \
  --event-handlers console_direct+ --ctest-args -E honest_e2e
ROS_LOCALHOST_ONLY=1 colcon test --packages-select lab_cobot_bringup \
  --event-handlers console_direct+ --ctest-args -R honest_e2e

colcon test-result --test-result-base build/image_pkg --verbose
colcon test-result --test-result-base build/lab_cobot_bringup --verbose
```

地图质量门：

```bash
python3 src/lab_cobot_navigation/maps/check_map.py
```

三方整合提交记录的验证证据：

| 基线 | 结果 |
|---|---|
| 导航 + 抓取合并 `0ee8783` | 7 个第一方包构建成功；常规测试通过；`gui:=true` 完整运行至 `DONE`；honest E2E 250.26s |
| 视觉整合 `b09171e` | `image_pkg` 7 项通过；`lab_cobot_bringup` 281 项、0 错误、0 失败；报告 E2E 199.96s，发布前干净环境复验 222.52s；两后端均完成运行态互斥检查 |

长时动态避障测试和 GUI 仿真应单独运行，执行前必须清理残留 Gazebo/ROS 进程
（`rm -rf /dev/shm/*fastrtps*`，否则残留信号量会让下一次启动 `map_server` 挂起）：

```bash
# 动态避障 E2E（nav_only 巡航 + 行走 actor，单次约 8 分钟）
PYTEST_ADDOPTS='-p no:anyio' colcon test --packages-select lab_cobot_gazebo \
  --event-handlers console_direct+ --ctest-args -R dynamic_obstacle_avoidance
```

## 尚未宣称的能力

- 除 `aruco_sample` 外的七类实体尚未接入抓取插件和任务序列。
- eight_class 已证明模型、话题、点云与 TF 链可运行，但未完成分类精度、XYZ 误差和
  朝向估计的系统标定验收。
- 双视觉后端只支持启动时切换，不支持 mission 运行中的热切换。
- LLM 和真人语音路径依赖外部 API key、音频与人工演示，不属于离线测试结论。
- 报告只证明干净环境中的通过记录，不据此声明无限次连续运行稳定率。

## 运行注意

- **底盘**走 `SetWorldPose` 位姿积分，不受碰撞阻挡；`/odom` 是插件自身积分。
- **抓取**是几何封套和双指接触对门控后的 fixed joint，不是真实摩擦力或触觉力反馈。
- `use_tactile_grasp` 与 `require_finger_contact` 必须保持同值；默认均为 `true`。
- 默认 `use_truth_pose=false`、`use_sim_attach=false`，不读取 Gazebo 目标真值，也不
  启动调试吸附 bridge。
- WSLg 下 launch 自动设置 D3D12/Qt 环境变量；FastDDS 配置禁用共享内存。
- headless 结束时 MoveIt/rclpy 可能输出 SIGINT 噪声，以 `/task/status` 是否到
  `DONE` 为任务判据。
- Actor 幽灵碰撞体透明不可见，但可被 costmap/LiDAR 感知。

## 发布内容索引

| 路径 | 说明 |
|---|---|
| `src/lab_cobot_navigation/maps/map_provenance.yaml` | SLAM 地图来源与质量门 |
| `benchmarks/README.md` | 可重复 benchmark 入口 |
| `THIRD_PARTY_LICENSES.md` | vendored `pymoveit2` 等第三方许可 |

## 许可

本仓库第一方代码采用 Apache-2.0；第三方组件许可见 `THIRD_PARTY_LICENSES.md`。
