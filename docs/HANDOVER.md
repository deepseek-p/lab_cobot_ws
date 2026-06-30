# CS-202618 移动协作机器人 — 开发交接文档(HANDOVER)

> **给接手 AI**:本文档让你直接接手「实验室移动协作机器人跨工位识别抓取仿真」项目。
> 先读完本文档,再看 `git log` 和 `docs/plans/`。**特别注意第 4、5 节的踩坑与环境配置**——那是几小时血泪换来的,不读必重蹈覆辙。
> 更新日期:2026-07(对话交接时)

---

## 1. 项目概述

- **赛题**:CS-202618 中车株洲「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」
- **目标**:一台**麦克纳姆全向底盘 + 立柱 + UR5e + 真空吸盘**的一体化移动机械臂,在实验室双工位场景完成:`文本指令→导航到工位A→ArUco识别样件→抓取→导航到工位B→放置→返回home`
- **工作空间**:`~/projects/lab_cobot_ws`(git 仓库,~30 commits)
- **技术栈**:ROS 2 Humble · Gazebo Classic 11 · Nav2(AMCL+EKF+DWB) · MoveIt 2 · slam_toolbox · ArUco · 真空吸盘插件
- **移植蓝本**:eyrc-24-25-logistic-cobot(MIT,分离式小车+固定臂)→ 我们自建一体化麦轮移动机械臂

## 2. 当前状态(完成度)

### ✅ 已完成并验证(headless)
- **8 个包全部 colcon build 通过**,25 单元测试全绿,5 launch 可解析
- **本体**:一体化麦轮 URDF(底盘+立柱+UR5e+吸盘+激光/IMU/RGBD),RViz/Gazebo 显示正常、装配正确(check_urdf 27 link)
- **场景**:双工位 world + ArUco 样件(贴桌上)
- **建图**:slam_toolbox 栈可用,建出干净图(5 簇)
- **导航**:**单次导航实测打通**(发 1 个 goal,机器人从 0 走到 1.22m,planner 出 29 点路径、controller 发 0.26m/s)
- **关键运行时 bug 全部修复**(见第 4 节):轮子陷地、激光扫立柱、地图假障碍、机器人翻倒、nav2 DDS 卡死、AMCL 简陋

### ⚠️ 未完成 / 待验证(接手重点)
- **连续多次导航的稳定性**:用户反馈"导航 2 次后图乱/停住"。已诊断为 **AMCL 配置简陋导致定位漂移**,已补全参数(commit 964ffa4)但**未最终验证**(被 WSL nav2 启动偶发卡顿挡住)。**接手第一优先级:GUI 连续导航 3-4 次确认 AMCL 加固是否解决**。
- **阶段 5 未做**:MoveIt 抓取实跑、ArUco 感知实跑、端到端 mission(`/task/instruction`→状态机)——代码骨架都在(`lab_cobot_manipulation`/`lab_cobot_perception`/`lab_cobot_bringup`),但没在 GUI 跑通过。
- **标定**:吸盘抓取姿态 DOWN_QUAT、放置点坐标、相机内参——预设值,需运行时标定。

## 3. 包结构(8 包)

| 包 | 职责 | 关键文件 |
|---|---|---|
| `lab_cobot_description` | 一体化 URDF + SRDF | `urdf/lab_cobot.urdf.xacro` + `urdf/inc/{mecanum_base,pillar,sensors,vacuum_gripper}.xacro` + `config/initial_positions.yaml` |
| `lab_cobot_gazebo` | 双工位 world + 样件 + spawn | `worlds/lab.world` + `models/aruco_sample/` + `launch/world.launch.py` |
| `lab_cobot_navigation` | Nav2 + EKF + 建图 + 地图 | `config/{nav2_params,ekf,mapping}.yaml` + `launch/{navigation,mapping}.launch.py` + `maps/map.{pgm,yaml}` + `maps/{check_map,denoise_map}.py` |
| `lab_cobot_moveit` | UR5e MoveIt 配置 | `config/*` + `launch/move_group.launch.py` |
| `lab_cobot_perception` | ArUco 检测 + 针孔反投影 | `pose_math.py` + `aruco_detector.py` |
| `lab_cobot_manipulation` | pick/place(pymoveit2+吸盘) | `pick_place_node.py` |
| `lab_cobot_bringup` | 跨工位状态机 + 一键 launch | `task_state_machine.py` + `mission_node.py` + `launch/lab_cobot.launch.py` |
| `pymoveit2` | vendored MoveIt2 Python 接口 | (第三方 BSD-3) |

## 4. 关键修复与踩坑(★必读★,按主题)

每一条都是真 bug + 已修复,改坏会复发:

1. **激光扫到自身立柱 → 建图全是噪点**(commit 304dd69)
   - 激光高 0.27m,立柱高 0.23~0.53m,**激光扫描平面穿过立柱**,360° 扫到自己 → 建图时立柱随机器人移动留下 594 个假障碍簇。
   - 修复:`sensors.xacro` 激光 `<min>0.32</min>`(立柱在 0.165~0.291m 处,提高最小量程使其入盲区,保留 360°)。
   - **教训:改激光位置/立柱尺寸时,必须保证激光扫描平面不穿过任何自身结构。**

2. **地图起点是假障碍 → planner 无路径 → 机器人只原地转**(commit 8309931/f2e78c2)
   - 现象"只转圈不走"的真因:planner 规划失败(起点被占)→ 触发 recovery spin。
   - 工具:`maps/check_map.py`(检查起点占用+噪点数)、`maps/denoise_map.py`(去噪)。
   - **好图标准**:尺寸≈140×140(不是 368 那种拉长=漂移变形)、连通域<15、起点(0,0)=254 free。

3. **机器人容易翻倒**(commit f3cf625)
   - UR5e 零位竖直向上 1.7m,重心过高 → 移动即翻。
   - 修复:底盘加大到 `0.55×0.50`、加重到 `180kg`、立柱降到 `0.30`、臂 spawn 即收拢成 home 姿态(`config/initial_positions.yaml`)。
   - **教训:Gazebo Classic 的 `spawn_entity.py` 不支持 `-J` 设初始关节!** 只能靠 URDF ros2_control 的 `initial_value`。

4. **★nav2 启动卡死/controller 不激活 → WSL2 DDS 问题★**(最关键,固化在 `~/.bashrc`)
   - 现象:`controller_server.rclcpp: failed to send response to change_state (timeout)`,lifecycle 卡死。
   - 根因:**WSL2 + 默认 FastRTPS DDS 跨网络发现,服务握手超时**。
   - 修复:**`export ROS_LOCALHOST_ONLY=1`**(已写入 `~/.bashrc`)。这是 WSL2 跑 nav2 的**必备**设置。
   - 验证:加上后 controller/planner/bt 全部 active,机器人实测走通。

5. **AMCL 配置极简陋 → 多次导航后定位漂移"图乱停住"**(commit 964ffa4,**待验证**)
   - eyrc 的 AMCL 只有 alpha+坐标系,缺 `max_beams/particles/update_min/recovery_alpha/laser_model` 等。
   - 已补全(`nav2_params.yaml` amcl 段)。**接手要 GUI 连续导航验证此修复。**

6. **诊断方法论教训(避免我踩的坑)**:
   - **`ps aux | grep 'xxx'` 会假阳性**(匹配到你自己命令行里的字面字符串)!**一律用 `pgrep -x 进程名`** 精确判断。我曾被"编译反复反弹"的假象骗了好几轮,真相是 `pgrep -x cc1plus=0`。
   - **headless 启动 gazebo:用 `run_in_background:true` 比 `nohup ... &` 可靠**(后者常启动即崩、空日志)。
   - **孤儿 `gzclient`(无 gzserver)会占资源**,新 gazebo 起不来 → `pkill -9 gzclient`。
   - **nav2 在 WSL 下 lifecycle 启动偶发卡**(map_server/controller configure 卡):`Ctrl+C` 重启一次基本就好。

## 5. 环境要求与配置(★关键★)

- **WSL2 内存**:`C:\Users\<用户>\.wslconfig` 设 `[wsl2] memory=10GB swap=4GB`(物理 16G),否则 Gazebo+nav2 易 OOM。已配。
- **`~/.bashrc` 已固化**(新终端自动生效):
  ```bash
  export ROS_LOCALHOST_ONLY=1                          # ★WSL2 nav2 DDS 必备
  source /opt/ros/humble/setup.bash
  source ~/projects/lab_cobot_ws/install/setup.bash
  ```
- **一次只跑一套重负载**:别同时跑大型 C++ 编译 + gazebo+nav2(16G 扛不住)。

## 6. 关键命令

```bash
# 构建(若 pymoveit2 symlink 报错: rm -rf build install 后重来)
cd ~/projects/lab_cobot_ws && colcon build --symlink-install && source install/setup.bash

# 看模型
ros2 launch lab_cobot_description view_robot.launch.py

# Gazebo + 机器人(新终端,bashrc 已 source)
ros2 launch lab_cobot_gazebo world.launch.py

# 建图(3 终端:world + mapping + teleop),要领:慢速(z降到0.15)、待在房间中部别冲墙、回起点闭环
ros2 launch lab_cobot_navigation mapping.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run nav2_map_server map_saver_cli -f ~/projects/lab_cobot_ws/src/lab_cobot_navigation/maps/map
python3 ~/projects/lab_cobot_ws/src/lab_cobot_navigation/maps/check_map.py   # 检查图质量

# 导航(2 终端:world + navigation),RViz 点「Nav2 Goal」发目标
ros2 launch lab_cobot_gazebo world.launch.py
ros2 launch lab_cobot_navigation navigation.launch.py
#   若 nav2 没全激活(卡 map_server) → 终端2 Ctrl+C 重启一次

# MoveIt / 一键全栈 / 发任务(阶段5,未跑通)
ros2 launch lab_cobot_moveit move_group.launch.py
ros2 launch lab_cobot_bringup lab_cobot.launch.py
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"

# 单元测试
cd src && python3 -m pytest lab_cobot_perception/test lab_cobot_bringup/test lab_cobot_navigation/test -v
```

## 7. 下一步(接手优先级)

1. **【最高】GUI 连续导航 3-4 次,验证 AMCL 加固(commit 964ffa4)是否解决"多次导航后乱"**。若仍乱:看 RViz 激光红点和墙是否重合(=定位漂移,继续调 AMCL)还是机器人被一圈 costmap 障碍围住(=改 costmap clearing/inflation)。
2. 阶段 5:GUI 跑通 MoveIt 抓取 + ArUco 感知 + 端到端 mission。
3. 标定:吸盘 DOWN_QUAT、放置点、相机内参。
4. (可选增强)平行夹爪替换吸盘、真实点云识别、轨道交通主题场景。

## 8. 关键参数速查

- 底盘:`0.55×0.50×0.15`m,`base_mass=180kg`,麦轮 radius 0.08
- 立柱:`column_height=0.30`,UR 安装点 footprint z≈0.53
- 激光:`min_range=0.32`,max 12m,360°,高 0.27m
- footprint(含麦轮):`[[0.28,0.31],...]`,robot_radius 0.42
- 地图:`map.pgm` 138×138 @0.05,origin[-3.46,-3.46]
- 工位:A(取)world 桌 (2.0,1.5),B(放)(-2.0,1.5);机器人停 y=0.65 朝 +y

## 9. 给接手 AI 的核心建议

- **环境优先**:任何 ROS 命令前确认 `echo $ROS_LOCALHOST_ONLY` = 1。
- **诊断用 `pgrep -x`,不用 `ps|grep`**(假阳性教训)。
- **gazebo 起不来先 `pkill -9 gzclient` 清孤儿**;启动用 `run_in_background`。
- **nav2 偶发卡就重启 navigation**,别去猎杀进程(危险且徒劳)。
- 用户偏好:中文交流;不要擅自 kill claude 进程(曾误伤);commit 用 conventional 格式、无 attribution。
- 设计/计划详见 `docs/plans/2026-06-28-*`,进度见 `docs/notes/overnight-progress.md`。
