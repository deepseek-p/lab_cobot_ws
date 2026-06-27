# 移动协作机器人 · 跨工位识别抓取 仿真系统设计

> 项目:CS-202618 面向实验室智能管理的协作机器人环境感知与动作规划方法研究
> 发榜单位:中车株洲电力机车有限公司 ｜ 提交截止:2026-09-15
> 设计日期:2026-06-28 ｜ 状态:已获批,待生成实现计划
> 工作空间:`~/projects/lab_cobot_ws`(包前缀 `lab_cobot_`)

---

## 1. 目标与范围

### 1.1 本次目标(最小可行闭环)
构建一个 **Gazebo Classic 仿真的一体化移动协作机器人**,打通**单对象跨工位识别抓取闭环**:

```
导航到工位A → 识别样件 → 抓取 → 导航到工位B → 放置 → 返回 home → 状态回传
```

### 1.2 范围边界(YAGNI)
- **做**:麦轮全向底盘 + UR5e + 自主导航(Nav2 + 预建图 AMCL)+ ArUco 感知 + 吸附抓取 + 跨工位编排 + 基本错误处理。
- **暂不做**(后续增量):多对象顺序任务、动态障碍避让、SLAM 实时建图、语义地图/危险区域、LLM 任务拆解、轨道交通主题美术、平行夹爪、真实识别(颜色/点云)。
- 对应赛题:跨工位 + 语音/文本指令属"原型系统(加分项)"与工程价值维度;本闭环是冲奖亮点的地基。

---

## 2. 关键决策记录(用户已拍板)

| 决策项 | 选择 | 理由 |
|---|---|---|
| 仿真栈 | **Gazebo Classic 11**(不沿用本地不稳定项目) | 环境已就位;本地 `robot_lab_demo` 有 WSLg/文档/gz-sim 等隐患 |
| 拼接策略 | **方案A 全新重建**(干净工作空间,以开源项目为蓝本移植) | 规避本地不稳定代码,从验证过的开源架构起步 |
| 移动底盘 | **麦克纳姆全向**(`gazebo_ros_planar_move`) | 全向便于狭窄工位精确对位抓取 |
| 定位/建图 | **预建静态地图 + AMCL** | 实验室布局固定,最稳定可复现 |
| 交付范围 | **单对象跨工位最小闭环** | 先贯通链路,再增量扩展 |
| 工作空间名 | **`lab_cobot_ws`** | 简短 |
| 末端执行器 | **阶段1 真空吸盘 → 阶段2 平行夹爪** | 吸盘先跑通稳定,再换更真实的夹爪 |
| 基座立柱 | **保留(高度可调 ~0.4m)** | 矮底盘上抬高 UR 基座以够到标准实验台,移动机械臂标准设计 |

---

## 3. 移植蓝本:eyrc-24-25-logistic-cobot

### 3.1 为什么选它
- 知识库调研里**唯一被标为 P0「最像完整比赛系统」**的项目。
- **技术栈与本机环境 100% 一致**:ROS 2 **Humble** + Gazebo **11** + Nav2 + robot_localization(EKF)+ MoveIt2 + ArUco/RealSense —— 移植阻力最小。
- **e-Yantra(IIT Bombay)竞赛验证**的端到端系统,非半成品;MIT 许可可自由移植。
- 内部已是**业界标准件集成**(UR 官方、pymoveit2、realsense 插件、Nav2),相当于已完成"选型+集成"脏活。

### 3.2 模块稳定性评级(代码级核查)
| 模块 | 性质 | 代码量 | 稳定性 |
|---|---|---:|:---:|
| `pymoveit2` | 业界标准库(BSD) | 1982 行 | ★★★★★ |
| `ur_description`+`ur_moveit_config` | UR 官方 | 791 行 | ★★★★★ |
| `realsense_gazebo_plugin` | 知名第三方 | 293 行 | ★★★★☆ |
| `ebot_nav2`(Nav2 配置,含预建 `map.pgm`+AMCL+EKF) | 标准 Nav2 + 调参 | 274 行 | ★★★★☆ |
| `ur5_control`(ArUco 4x4_50 + 抓取) | 项目自写核心 | 446 行 | ★★★☆☆ |
| `ebot_description`(差速底盘) | 项目自写 | 266 行 | ★★★☆☆ |

### 3.3 ⚠️ 关键发现:eyrc 是「分离式」,不是一体移动机械臂
- `ebot`(差速小车,车体 0.585×0.30×0.25m)与 UR5 **无 joint 连接**,UR5 单独 `spawn_entity` 固定在世界中。
- eyrc 形态 = **移动小车送货 + 固定臂抓取**(两个独立机器人),这也是其导航/抓取分开启动的原因。
- **含义**:用户要的"UR 装底盘上边移动边抓"的一体移动机械臂,**eyrc 没有现成的,需我们新建**。本地 `lab_ur_mecanum.urdf.xacro` 做过 UR+麦轮组合,可作**纯几何结构参考**(不运行、不依赖)。

---

## 4. 总体架构与数据流

```
文本指令 "把样件从工位A送到工位B"
   │
   ▼
[任务编排状态机]  lab_cobot_bringup
   │  ① 导航到工位A ──────────► [Nav2] ─► /cmd_vel ─► 麦轮底盘(planar_move)
   │                              ▲ AMCL ◄─ /scan(LiDAR) ；EKF ◄─ /odom + /imu
   │  ② 识别样件 ─► [ArUco 感知] ─► TF: base_link → obj_<id>
   │  ③ 抓取 ─────► [MoveIt2 + pymoveit2] ─► UR5e 轨迹 ─► link attach(吸附)
   │  ④ 导航到工位B ─► [Nav2] ...
   │  ⑤ 放置 ──────► [MoveIt2] ─► link detach
   ▼
状态回传 /task/status
```

**移动操作解耦要点**:导航到位后底盘静止,MoveIt 在当前 `base_link`(UR 基座)系规划;抓取目标 TF 也在 `base_link` 系 → 与底盘全局位置无关,移动与抓取干净解耦。

---

## 5. 工作空间包结构(`~/projects/lab_cobot_ws/src/`)

| 包 | 职责 | 主要来源 |
|---|---|---|
| `lab_cobot_description` | 麦轮底盘+基座立柱+UR5e+末端+LiDAR/IMU/相机 URDF | ⭐新建一体化组装;借 eyrc `ebot_description` 车体 + 官方 `ur_description` |
| `lab_cobot_gazebo` | Gazebo Classic 实验室双工位世界 + spawn | 适配 eyrc `eyantra_warehouse` |
| `lab_cobot_navigation` | Nav2 参数 + 预建地图 + AMCL + EKF | 移植 eyrc `ebot_nav2` |
| `lab_cobot_moveit` | UR5e MoveIt2 配置 | 官方 `ur_moveit_config` |
| `lab_cobot_manipulation` | 抓取/放置节点(吸附 attach/detach) | 移植 eyrc `ur5_control`+`pymoveit2`+`linkattacher` |
| `lab_cobot_perception` | ArUco 6D 位姿 + TF 广播 | 移植 eyrc `ur5_control/aruco_detector` + `tf_broadcaster_pkg` |
| `lab_cobot_bringup` | 一键 launch + 跨工位任务编排状态机 | ⭐新建(参考 eyrc launch 组织 + 本地 WSLg 稳定渲染参数) |

### 拼接边界
- **直接移植/复用**:Nav2 配置、ArUco 感知、pymoveit2、UR/MoveIt 配置、link attacher、EKF。
- **⭐我们新建(核心)**:UR+麦轮底盘一体化组装(含基座立柱)、移动基座下 MoveIt、跨工位编排状态机。
- **适配改造**:差速→麦轮、仓库→实验室双工位、吸盘→(后续)夹爪。

---

## 6. 子系统设计

### 6.1 机器人本体(`lab_cobot_description`)
| 部件 | 方案 | 适配 |
|---|---|---|
| 移动底盘 | 4 麦克纳姆轮平台 | 借 eyrc 车体,驱动 `diff_drive`→`gazebo_ros_planar_move`(吃 /cmd_vel 的 x/y/yaw) |
| 基座立柱 | 高度可调立柱(默认 ~0.4m) | 新建;把 UR 基座抬到 ~0.65m 匹配台面 |
| 机械臂 | UR5e(6 DOF) | 官方 `ur_description` |
| 末端 | 真空吸盘+link attacher(阶段1) | 借 eyrc;阶段2 换平行夹爪 |
| 传感器 | LiDAR(`ray_sensor`/scan)+ IMU(`imu_sensor`)+ RGB-D(realsense 插件) | 用于 AMCL/EKF/感知 |

### 6.2 导航(`lab_cobot_navigation` + `lab_cobot_gazebo`)
- **世界**:Gazebo Classic 实验室,工位A+工位B两工作台 + 通道 + 边界墙。
- **建图(一次性)**:`slam_toolbox` 遥控跑一圈,存 `map.pgm`/`map.yaml`。
- **运行时定位**:`map_server` + **AMCL**(eyrc 现成参数,300–5000 粒子)+ `robot_localization` EKF 融合 /odom+/imu。
- **Nav2**:移植 eyrc `nav2_params.yaml`;全局 Dijkstra/NavFn。**麦轮适配**:局部规划**先用差速兼容模式(vx+wz)跑通**,再按需升级全向(vy/MPPI)。
- **工位目标**:预定义 A/B/home 的 map 坐标,`nav2_simple_commander` 发 NavigateToPose。

### 6.3 抓取与感知(`lab_cobot_manipulation` + `lab_cobot_perception`)
- **感知(ArUco 保底)**:移植 `aruco_detector.py`——检测 4x4_50 码 → 深度对齐 → PnP 6D 位姿 → 发布 TF `base_link→obj_<id>`;样件贴码。后续叠加颜色/点云真实识别。
- **抓取**:UR5e + MoveIt2(OMPL+KDL)+ `pymoveit2`;动作 approach→吸附(attach service)→retreat;放置反之(detach)。

### 6.4 任务编排 + 错误处理(`lab_cobot_bringup`)
- **编排状态机**:`导航到A→识别→抓取→导航到B→放置→回home→报告`,`nav2_simple_commander`+pymoveit2 串联,逐步查成败。
- **错误处理(最小)**:

| 失败 | 处理 |
|---|---|
| 导航失败/超时 | 重试 1 次 → 放弃报错,不进入抓取 |
| 识别不到 ArUco | 等待/重试 N 次 → 报错停止 |
| MoveIt 规划失败 | 重试 → 放弃,安全退回 home |
| 吸附失败 | 检测 attach 状态,失败则重试接近 |

- **一键启动**:一个 launch 起全栈,沿用本地踩过坑的 WSLg 稳定渲染参数(`GALLIUM_DRIVER=d3d12` 等)。

---

## 7. 测试与验收

| 层级 | 内容 |
|---|---|
| **第0步 冒烟** | 把 eyrc 原版在本机实际 `colcon build` + 跑通其导航与抓取,确认蓝本可用 |
| 单元 | URDF 能 spawn;Nav2 能到点;MoveIt 能规划;ArUco 能出 pose(各自假数据) |
| 集成 | 完整跑通"A→识别→抓→B→放",连续 5 次成功 |
| 指标(呼应赛题) | 导航到位率、抓取成功率、单次耗时 → CSV |

---

## 8. 已知风险与缓解

| 风险 | 影响 | 缓解 |
|---|---|---|
| eyrc 未实测编译(requirements.sh 未跑) | 移植地基不稳 | **第0步冒烟测试**先验证 |
| 一体化移动机械臂需自建 | 工作量+ | 本地 mecanum URDF 作几何参考;MoveIt 与底盘解耦 |
| 麦轮全向 vs Nav2 差速模型 | 局部规划行为 | 先差速兼容跑通,再升级全向 |
| WSLg 图形不稳定 | 演示卡顿 | 沿用本地验证的 D3D12 渲染参数;headless Gazebo+RViz |
| UR 装矮底盘工作空间受限 | 抓取够不到 | 基座立柱抬升 + 工作台高度匹配,冒烟后微调 |

---

## 9. 环境确认(已就位,无需联网)

- ROS 2 **Humble** ｜ Gazebo **Classic 11** ｜ MoveIt2 2.5.9
- Nav2 全栈 1.1.20(amcl/bringup/map-server/simple-commander)｜ slam_toolbox ｜ robot_localization
- Gazebo 插件:`gazebo_ros_planar_move`、`diff_drive`、`ray_sensor`、`imu_sensor`、`camera`
- UR 包:`ur_description`/`ur_moveit_config`/`ur_controllers`
- 蓝本已克隆:`/tmp/eyrc_ref`(286MB,Gazebo Classic 主体)
- ⚠️ `ros_gz_sim` 未装(故本地麦轮 gz-sim 版无法运行,本方案不依赖它)

---

## 10. 实现里程碑概览(详细见后续实现计划)

0. **冒烟测试**:验证 eyrc 原版可编译运行
1. 工作空间骨架 + 7 包脚手架
2. 一体化移动机械臂 URDF(麦轮+立柱+UR5e+传感器)能在 Gazebo spawn
3. 导航子系统:建图 → AMCL 定位 → Nav2 到点
4. 抓取子系统:MoveIt 规划 + 吸附抓放(固定位)
5. 感知子系统:ArUco 出 6D 位姿 TF
6. 集成:跨工位编排状态机打通最小闭环
7. 错误处理 + 指标 CSV + 一键 launch + 文档

> 下一步:用 `writing-plans` skill 把里程碑展开为可执行的分步实现计划。
