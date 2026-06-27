# 移动协作机器人跨工位识别抓取 — 实现计划

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** 在 Gazebo Classic 11 + ROS2 Humble 下,构建一台麦轮全向底盘 + UR5e 的一体化移动协作机器人,打通"导航到工位A→ArUco 识别样件→吸附抓取→导航到工位B→放置→返回 home"的单对象跨工位最小闭环。

**Architecture:** 以 e-Yantra `eyrc-24-25-logistic-cobot`(MIT,Humble+Gazebo Classic,已克隆于 `/tmp/eyrc_ref`)为移植蓝本。蓝本是"移动小车+固定臂"分离式,故**一体化移动机械臂需自建**;导航(Nav2+AMCL+EKF)、ArUco 感知、pymoveit2、UR/MoveIt 配置、link attacher 直接移植/复用。全新工作空间 `~/projects/lab_cobot_ws`,7 个 `lab_cobot_` 包。

**Tech Stack:** ROS2 Humble · Gazebo Classic 11 · `gazebo_ros_planar_move`(麦轮) · Nav2(AMCL+DWB+EKF/robot_localization) · MoveIt2(OMPL+KDL)+ pymoveit2 · OpenCV ArUco 4x4_50 · Gazebo link attacher(吸附)

---

## 阅读须知:本计划的两个特殊约定

### A. 详尽度分层(因为这是"移植"项目)
- **Phase 0–2 写到命令/代码级**:可直接执行。
- **Phase 3–7 写到任务级**(文件清单 + eyrc 来源映射 + 验证标准 + 提交点):**确切代码在执行到该 Phase 时,依据 Phase 0 冒烟测试产出的 `docs/notes/eyrc-smoke-test.md`(记录 eyrc 真实文件结构与可用性)再写死**。在未验证蓝本前就把后续代码写死,会建立在错误假设上。每个 Phase 0 任务都会沉淀这些笔记。

### B. 测试策略(ROS/仿真版 TDD)
ROS 仿真很多行为依赖运行时,不能全用纯单元测试。本计划分两类"测试门":
- **纯逻辑 → 真单元测试(pytest,严格 TDD)**:ArUco 像素+深度→3D 位姿换算、编排状态机转移、参数解析。先写失败测试。
- **仿真集成 → 可验证集成检查(作为该任务的"测试门")**:用 launch 起子系统 + 断言话题/TF/action 状态/日志关键行(尽量用 `launch_testing` 或脚本 `ros2 topic echo --once`/`ros2 node list` 断言)。每个集成任务都给出**明确的通过判据**。

> 工作空间 `lab_cobot_ws` 本身是独立 git 仓库,已隔离,无需额外 worktree。每个 Task 末尾 commit。

---

## Phase 0:冒烟测试 — 先证明蓝本能跑(最高优先级)

> 目的:在写一行新代码前,确认 eyrc 在本机可编译、导航可跑、抓取可跑;并把 eyrc 真实结构记成笔记,作为后续移植依据。**若此 Phase 暴露蓝本根本跑不通,立即停下来与用户复盘方案,不要硬往下做。**

### Task 0.1:搭建 eyrc 冒烟工作空间并安装依赖

**Files:**
- Create: `~/projects/eyrc_smoke_ws/`(独立于 lab_cobot_ws 的临时验证空间)
- Note: `~/projects/lab_cobot_ws/docs/notes/eyrc-smoke-test.md`

**Step 1:** 建工作空间并软链蓝本源码
```bash
mkdir -p ~/projects/eyrc_smoke_ws/src
ln -s /tmp/eyrc_ref ~/projects/eyrc_smoke_ws/src/eyrc_ref
ls -l ~/projects/eyrc_smoke_ws/src/
```
Expected: 看到 `eyrc_ref -> /tmp/eyrc_ref`

**Step 2:** 安装依赖(注意 sudo,逐条观察成败)
```bash
cd /tmp/eyrc_ref && bash requirements.sh
```
Expected: gazebo-ros/plugins/xacro/tf-transformations 报 installed 或 already newest;transforms3d 安装。记录任何 failed 行。

**Step 3:** 用 rosdep 探测缺失依赖(预期会报 `ebot_docking` 找不到)
```bash
cd ~/projects/eyrc_smoke_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y 2>&1 | tail -20
```
Expected: 大部分已满足;**预警**:`ebot_docking` 无法解析(仓库缺该包)。记到笔记。

**Step 4:** 首次构建,跳过我们不需要、且依赖缺失的包
```bash
cd ~/projects/eyrc_smoke_ws
colcon build --symlink-install --packages-skip my_auv_sim ebot_docking 2>&1 | tail -30
```
Expected: 记录哪些包成功、哪些失败。**核心目标包必须成功**:`pymoveit2` `ur_description` `ur_moveit_config` `ur_simulation_gazebo` `ebot_nav2` `ur5_control` `tf_broadcaster_pkg` `linkattacher_msgs` `realsense_gazebo_plugin`。若 `ebot_description` 因 `ebot_docking` 失败,记下并继续(我们最终自建底盘)。

**Step 5:** 把结果写进笔记并提交
```bash
# 在 docs/notes/eyrc-smoke-test.md 记录:依赖安装结果、rosdep 缺失项、各包 build 成功/失败、报错摘要
cd ~/projects/lab_cobot_ws
git add docs/notes/eyrc-smoke-test.md
git commit -m "docs: 记录 eyrc 冒烟测试 - 依赖与构建结果"
```

### Task 0.2:冒烟验证导航(AMCL 定位 + Nav2 到点)

**Step 1:** 起仿真世界 + 机器人(终端1)
```bash
cd ~/projects/eyrc_smoke_ws && source install/setup.bash
ros2 launch ebot_description ebot_gazebo_launch.py
```
**Step 2:** 起 Nav2 bringup(终端2)
```bash
cd ~/projects/eyrc_smoke_ws && source install/setup.bash
ros2 launch ebot_nav2 ebot_bringup_launch.py
```
**Step 3:** 验证关键话题/TF(终端3)
```bash
ros2 topic echo /scan --once        # 有激光数据
ros2 topic echo /odom --once        # 有里程计
ros2 topic echo /amcl_pose --once   # AMCL 输出定位
ros2 topic list | grep -E "map|scan|cmd_vel|amcl"
```
**Step 4:** 发一个导航目标(用 RViz 2D Goal Pose,或 simple_commander 脚本),观察机器人是否规划+移动到点。

**通过判据:** AMCL 有定位输出 + 机器人能朝目标移动并到达。结果(成功/卡点/报错)记入笔记。**截图存 `docs/notes/img/`。**

**Step 5:** Commit 笔记更新。

### Task 0.3:冒烟验证感知 + 机械臂抓取

**Step 1:** 起抓取场景(终端1,注意 README 命名差异,实际 launch 见 Task 0.1 笔记)
```bash
ros2 launch eyantra_warehouse task1c.launch.py
```
**Step 2:** 起 UR MoveIt(终端2)
```bash
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py
```
**Step 3:** 起 ArUco 检测(终端3)
```bash
ros2 run ur5_control aruco_detector
```
**Step 4:** 验证
```bash
ros2 topic list | grep -iE "aruco|image|points"
ros2 run tf2_tools view_frames   # 看是否有 obj_<id> 帧
```
**通过判据:** ArUco 出 `obj_<id>` TF + UR 能在 MoveIt 下规划运动 + link attacher 能吸起物体。记入笔记 + 截图。

### Task 0.4:沉淀冒烟结论(后续移植的"事实地图")

在 `docs/notes/eyrc-smoke-test.md` 补全这些**后续 Phase 直接引用**的事实:
- 各核心包能否 build、运行;遇到的坑与解法。
- eyrc 关键文件实际路径:`ebot_description.xacro`(底盘几何/插件)、`nav2_params.yaml`、`ekf.yaml`、`map.pgm/yaml`、`aruco_detector.py`、link attacher 用法、`ur_sim_moveit.launch.py` 结构。
- UR5 在 eyrc 里的实际安装高度(判断我们基座立柱目标高度)。
- link attacher `.so` 是否可用(ABI)。

**Commit:** `docs: 完成 eyrc 冒烟测试结论笔记`

---

## Phase 1:工作空间骨架 + 7 包脚手架

### Task 1.1:创建 7 个空包

**Files:** `~/projects/lab_cobot_ws/src/` 下 7 个包

**Step 1:** 创建包(混合 ament_cmake / ament_python)
```bash
cd ~/projects/lab_cobot_ws/src
source /opt/ros/humble/setup.bash
# 描述/世界/导航/moveit 用 ament_cmake(装 launch/config/urdf)
ros2 pkg create lab_cobot_description --build-type ament_cmake
ros2 pkg create lab_cobot_gazebo --build-type ament_cmake
ros2 pkg create lab_cobot_navigation --build-type ament_cmake
ros2 pkg create lab_cobot_moveit --build-type ament_cmake
ros2 pkg create lab_cobot_bringup --build-type ament_cmake
# 感知/抓取/编排逻辑用 ament_python
ros2 pkg create lab_cobot_perception --build-type ament_python
ros2 pkg create lab_cobot_manipulation --build-type ament_python
```
**Step 2:** 构建空包验证脚手架
```bash
cd ~/projects/lab_cobot_ws && colcon build --symlink-install
```
Expected: 7 packages finished
**Step 3:** Commit `chore: 初始化 7 个 lab_cobot_ 包脚手架`

### Task 1.2:引入可复用的第三方资产(license 合规)

> 决策:`pymoveit2` 与 link attacher 插件作为 vendored 依赖纳入,保留出处与 LICENSE。

**Files:**
- Create: `src/third_party/pymoveit2/`(从 `/tmp/eyrc_ref/pymoveit2` 复制)
- Create: `src/lab_cobot_manipulation/plugins/`(放 `libgazebo_link_attacher.so` 等)
- Create: `THIRD_PARTY_LICENSES.md`(记录 pymoveit2/UR/realsense 插件出处与许可)

**Step 1:** 复制 pymoveit2 与 link attacher、记录许可。
**Step 2:** colcon build 通过。
**Step 3:** Commit `chore: 引入 pymoveit2 与 link attacher(含许可声明)`

---

## Phase 2:一体化移动机械臂 URDF(`lab_cobot_description`)

> 这是 eyrc 没有、我们的核心新建工作。逐件搭建,每件都 spawn 验证。参考 `/tmp/eyrc_ref/ebot_description/models/ebot/ebot_description.xacro` 的几何与本地 `lab_ur_mecanum.urdf.xacro` 的组合思路(仅参考,不复制运行)。

### Task 2.1:麦轮底盘 + planar_move 插件

**Files:**
- Create: `src/lab_cobot_description/urdf/inc/mecanum_base.xacro`
- Create: `src/lab_cobot_description/urdf/lab_cobot_base.urdf.xacro`(临时顶层,仅底盘)
- Create: `src/lab_cobot_description/launch/spawn_base_test.launch.py`

**Step 1:** 写底盘 xacro:base_link(box,参考 eyrc 0.585×0.30×0.25)+ base_footprint + 4 麦轮 visual/collision + `gazebo_ros_planar_move` 插件(`<commandTopic>cmd_vel</commandTopic>`、`<odometryTopic>odom</odometryTopic>`、`<odometryFrame>odom</odometryFrame>`、`<robotBaseFrame>base_footprint</robotBaseFrame>`)。
**Step 2:** 写 spawn 测试 launch(gzserver + spawn_entity + robot_state_publisher)。
**Step 3:** 验证(集成测试门):
```bash
ros2 launch lab_cobot_description spawn_base_test.launch.py
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "{linear: {x: 0.2, y: 0.1}}"
ros2 topic echo /odom --once   # 通过判据:odom 随运动变化,x 与 y 都能动(全向)
```
**Step 4:** Commit `feat(description): 麦轮全向底盘 + planar_move`

### Task 2.2:基座立柱(可调高度)

**Files:** Create `src/lab_cobot_description/urdf/inc/base_column.xacro`
**Step 1:** 写立柱 xacro:`<xacro:property name="column_height" default="0.4"/>`,box 立柱 link,joint 固定到 base_link 顶面,顶部输出 `arm_mount_link`。
**Step 2:** 验证:spawn 后 `ros2 run tf2_ros tf2_echo base_footprint arm_mount_link`,z ≈ 底盘高+0.4。
**Step 3:** Commit `feat(description): 可调高度基座立柱`

### Task 2.3:挂载 UR5e + ros2_control

**Files:** Modify `lab_cobot_base.urdf.xacro`→ 重命名为 `lab_cobot.urdf.xacro`;引入官方 `ur_macro.xacro`
**Step 1:** include UR5e xacro,parent=`arm_mount_link`,配置 `ur_type:=ur5e`、`simulation_controllers`、`gazebo_ros2_control`。
**Step 2:** 写/复用控制器 yaml(`joint_state_broadcaster` + `scaled_joint_trajectory_controller`)。
**Step 3:** 验证:spawn 后 `ros2 control list_controllers` 两个控制器 active;`ros2 topic echo /joint_states` 含 6 个 UR 关节。
**Step 4:** Commit `feat(description): 在立柱上挂载 UR5e + 控制器`

### Task 2.4:真空吸盘末端 + link attacher 挂载点

**Files:** Create `src/lab_cobot_description/urdf/inc/vacuum_gripper.xacro`
**Step 1:** 在 UR `tool0` 加吸盘几何 + `ee_link`/`suction_link`;加载 link attacher Gazebo 插件(参考 Phase 0 笔记的用法)。
**Step 2:** 验证:TF 有 `suction_link`;Gazebo 加载插件无报错。
**Step 3:** Commit `feat(description): 真空吸盘末端 + link attacher`

### Task 2.5:传感器(LiDAR + IMU + RGB-D)

**Files:** Create `src/lab_cobot_description/urdf/inc/sensors.xacro`
**Step 1:** 加 2D LiDAR(`gazebo_ros_ray_sensor`→`/scan`,装底盘前方)、IMU(`gazebo_ros_imu_sensor`→`/imu`)、RGB-D(realsense 插件或 `gazebo_ros_camera`→`/camera/image`+`/camera/points`,装立柱/腕部俯视工作台)。
**Step 2:** 验证:`/scan`、`/imu`、`/camera/points` 三个话题都有数据。
**Step 3:** Commit `feat(description): LiDAR + IMU + RGB-D 传感器`

### Task 2.6:整机整合 + SRDF

**Files:** Finalize `lab_cobot.urdf.xacro`;Create `src/lab_cobot_description/srdf/lab_cobot.srdf.xacro`;Create `launch/view_robot.launch.py`(RViz)
**Step 1:** 顶层组合全部 inc;SRDF 定义 planning group(UR5e)+ 末端。
**Step 2:** 验证(集成门):
```bash
ros2 launch lab_cobot_description view_robot.launch.py
ros2 run tf2_tools view_frames   # 通过判据:完整 TF 树 base_footprint→...→suction_link 无断裂
xacro lab_cobot.urdf.xacro > /tmp/check.urdf && check_urdf /tmp/check.urdf  # Successfully parsed
```
**Step 3:** Commit `feat(description): 一体化移动机械臂整机 URDF + SRDF`

---

## Phase 3–7:任务级计划(执行时依据 Phase 0 笔记细化代码)

> 以下每个 Task 给出:**文件 / eyrc 来源映射 / 做什么 / 通过判据 / 提交**。代码骨架在执行该 Task 时,对照 `docs/notes/eyrc-smoke-test.md` 里 eyrc 的真实实现写死。

### Phase 3:导航子系统(`lab_cobot_gazebo` + `lab_cobot_navigation`)

| Task | 文件 | eyrc 来源 | 做什么 | 通过判据 |
|---|---|---|---|---|
| 3.1 | `lab_cobot_gazebo/worlds/lab.world` | 适配 `eyantra_warehouse` world | Gazebo Classic 实验室:工位A/B 两工作台+通道+边界墙 | Gazebo 能加载,无缺模型报错 |
| 3.2 | `lab_cobot_gazebo/launch/world.launch.py` | eyrc `start_world_*` | 起世界 + spawn 整机 | 机器人出现在世界、传感器话题有数据 |
| 3.3 | `lab_cobot_navigation/maps/lab_map.{pgm,yaml}` | 用 slam_toolbox(eyrc `mapper_params_online_async.yaml`) | 遥控跑一圈建图存盘 | 地图 pgm 清晰、可被 map_server 加载 |
| 3.4 | `lab_cobot_navigation/config/{nav2_params,ekf}.yaml` | 移植 eyrc `nav2_params.yaml`+`ekf.yaml` | AMCL+DWB(先差速兼容 vx/wz)+ EKF 融合 odom/imu;改 frame/footprint/速度 | 参数加载无报错 |
| 3.5 | `lab_cobot_navigation/launch/navigation.launch.py` | eyrc `ebot_bringup_launch.py` | map_server+AMCL+Nav2+EKF 一起起 | RViz 看到定位;simple_commander 发 A/B/home 能到达 |
| 3.6 | `lab_cobot_navigation/lab_cobot_navigation/waypoints.py` | 新建 | 定义 A/B/home map 坐标 + 封装 go_to(name) | 单测:坐标表解析;集成:连续到 3 点 |

**阶段提交:** 每 Task 后 commit;Phase 末 `feat(nav): 跨工位自主导航打通`

### Phase 4:抓取子系统(`lab_cobot_moveit` + `lab_cobot_manipulation`)

| Task | 文件 | eyrc 来源 | 做什么 | 通过判据 |
|---|---|---|---|---|
| 4.1 | `lab_cobot_moveit/config/*`(kinematics/ompl/controllers/srdf) | 官方 `ur_moveit_config` + 我们的 SRDF | UR5e MoveIt 配置适配本机器人 | move_group 启动无报错 |
| 4.2 | `lab_cobot_moveit/launch/move_group.launch.py` | eyrc `ur_sim_moveit.launch.py` | 起 move_group + RViz MotionPlanning | RViz 能交互规划并执行 |
| 4.3 | `lab_cobot_manipulation/.../pick_place.py` | eyrc `ur5_control` + `pymoveit2` | approach→吸附(attach srv)→retreat;place 反之 | **固定坐标**抓起样件并放到目标点 |
| 4.4 | `lab_cobot_manipulation/.../attach_client.py` | eyrc link attacher 用法 | 封装 attach/detach service 调用 | 单测:srv 请求构造;集成:物体随末端移动 |

**阶段提交:** Phase 末 `feat(manip): 固定位吸附抓放打通`

### Phase 5:感知子系统(`lab_cobot_perception`)

| Task | 文件 | eyrc 来源 | 做什么 | 通过判据 |
|---|---|---|---|---|
| 5.1 | `lab_cobot_gazebo/models/sample_*/` | 新建(带 ArUco 4x4_50 贴图) | 样件模型 + 码,放入 world | RViz/相机能看到码 |
| 5.2 | `lab_cobot_perception/.../pose_math.py` | 新建(纯逻辑) | 像素+深度+内参→相机系 3D 点(针孔模型) | **TDD 单元测试**:已知输入→已知 3D 输出 |
| 5.3 | `lab_cobot_perception/.../aruco_detector.py` | 移植 eyrc `aruco_detector.py` | 检测码→调 pose_math→发布 TF `base_link→obj_<id>` | 集成:RViz 中 obj TF 跟随样件;误差 < 阈值 |

**阶段提交:** Phase 末 `feat(perception): ArUco 6D 位姿 + TF`

### Phase 6:跨工位编排(`lab_cobot_bringup`)

| Task | 文件 | 做什么 | 通过判据 |
|---|---|---|---|
| 6.1 | `lab_cobot_bringup/.../task_state_machine.py` | 状态机:导航A→识别→抓→导航B→放→home;每步查成败 | **TDD 单元测试**:状态转移表(含失败分支) |
| 6.2 | `lab_cobot_bringup/.../mission_node.py` | 订阅 `/task/instruction`,驱动状态机,发 `/task/status` | 集成:发指令触发流程 |
| 6.3 | `lab_cobot_bringup/launch/bringup.launch.py` | 一键起全栈(world+nav+moveit+perception+mission),含 WSLg D3D12 渲染参数 | 一条命令起全系统无致命报错 |
| 6.4 | (集成) | 端到端跑最小闭环 | **连续 5 次** A→识别→抓→B→放→home 成功 |

**阶段提交:** Phase 末 `feat(bringup): 跨工位最小闭环打通`

### Phase 7:错误处理 + 指标 + 文档

| Task | 文件 | 做什么 | 通过判据 |
|---|---|---|---|
| 7.1 | 各执行节点 | 兜底:导航失败重试1次→放弃;识别重试N次;规划失败退home;吸附失败重试 | 注入失败仿真,行为符合预期 |
| 7.2 | `lab_cobot_bringup/.../metrics.py` + `results/*.csv` | 记录到位率/抓取成功率/单次耗时 | 跑一轮生成 CSV |
| 7.3 | `README.md` + `docs/DEPLOY.md` | 构建/启动/指令/评测全流程命令 | 他人按文档可复现 |
| 7.4 | (验收) | 最终集成 + 录屏 | 闭环稳定演示 + 指标 CSV 齐全 |

**阶段提交:** `feat: 错误处理、指标记录与部署文档`

---

## 完成定义(Definition of Done)
- [ ] Phase 0 冒烟测试通过(eyrc 导航+抓取在本机可跑)且笔记沉淀
- [ ] 一体化移动机械臂能在 Gazebo spawn,TF/控制器/传感器齐全
- [ ] AMCL 定位 + Nav2 跨工位到点稳定
- [ ] MoveIt 吸附抓放成功
- [ ] ArUco 6D 位姿 TF 正确
- [ ] 端到端跨工位闭环连续 5 次成功
- [ ] 错误处理 + 指标 CSV + 一键 launch + 部署文档齐全
- [ ] 全程频繁 commit,每个 Task 一次

## 风险登记(同设计文档 §8)
eyrc 未实测编译 → Phase 0 先验证;一体化需自建 → Phase 2 逐件 spawn 验证;麦轮 vs Nav2 差速模型 → 先差速兼容;WSLg 图形 → D3D12 参数 + headless;预编译 `.so` ABI → Phase 0 验证。
