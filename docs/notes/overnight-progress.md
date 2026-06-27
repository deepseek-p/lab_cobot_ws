# 通宵自主推进进度

> 用户睡觉期间自主推进(2026-06-28 夜)。醒来看这份笔记 + `git log` + `README.md` 即可掌握全貌。
> 原则:所有不需要 GUI 的环节都做掉并自动验证(colcon build / xacro / check_urdf / pytest);需 GUI 实跑的标记在"待收尾"。

## ✅ 已完成(全部 headless 自动验证通过)
- **Phase 0 冒烟**:eyrc 10 核心包编译、world 加载、组件齐全(`eyrc-smoke-test.md`)。
- **Phase 1**:7 包骨架 + colcon build。
- **Phase 2 本体**:一体化麦轮移动机械臂 URDF(26 links)+ SRDF。check_urdf ✓、xacro ✓。
- **Phase 3 场景**:实验室双工位 world(headless gzserver 加载无错)+ 带 ArUco 纹理样件(gz sdf 校验)+ spawn launch。
- **Phase 4 MoveIt**:UR5e 配置(kinematics/ompl/joint_limits/controllers)+ move_group launch。**MoveItConfigsBuilder 实际加载 URDF+SRDF+config 验证通过**(强验证)。
- **Phase 5 感知**:pose_math 针孔反投影(**8 测试**)+ aruco_detector 节点(import ✓,内参动态、复用 pose_math)。
- **Phase 6 编排**:跨工位状态机(**10 测试**)+ mission_node(import 链 nav2_simple_commander+manipulation+navigation+bringup 全联通)。
- **抓取**:pick_place 节点(pymoveit2 + 真空吸盘 vacuum 插件)+ vendored pymoveit2。
- **导航**:Nav2 配置移植(AMCL+EKF+costmap,base_footprint/scan/odom 适配)+ 工位 waypoints(**7 测试**)+ 导航 launch。
- **集成**:一键全栈 launch(8 actions)+ WSLg 渲染参数。
- **全量构建**:`colcon build` **8 包全部 finished**;**25 单元测试全通过**。
- **文档**:README(部署说明,赛题必交项)+ THIRD_PARTY_LICENSES。

## ⚠️ 踩坑记录
1. eyrc 公开仓库精简了 PyArmor 加密 spawner_box(ur_description scripts)+ realsense ROS 封装源 → 已绕过(改用标准 gazebo_ros_camera/自建场景)。
2. UR macro base_link 与底盘 base_link 冲突 → UR 加 `tf_prefix=ur_`。
3. **pymoveit2 symlink-install 缓存污染**(`failed to create symbolic link ... Is a directory`)→ `rm -rf build install && colcon build --symlink-install` 解决(已写入 README)。

## 🖥️ 待用户收尾(需 GUI / WSLg 运行时验证)
1. `ros2 launch lab_cobot_description view_robot.launch.py` — RViz 看模型(加 RobotModel,Topic=/robot_description)。
2. `ros2 launch lab_cobot_gazebo world.launch.py` — Gazebo 看机器人落地、控制器、传感器话题(/scan /imu/data /bench_camera/*)。
3. 用 slam_toolbox 在 lab.world 重建地图,替换占位 map。
4. `ros2 launch lab_cobot_bringup lab_cobot.launch.py` + 发 /task/instruction — 端到端闭环。
5. 标定:吸盘抓取姿态 DOWN_QUAT、放置点坐标、ArUco 相机内参。

## 🔬 后台任务结局
- nav-port subagent:疑似卡 429,已发 shutdown;navigation 配置由主线自行完成。
- 替代方案调研 workflow:429 限流失败;待重试(见下)。

## git 提交链(本次新增)
7包骨架 → 一体化URDF+SRDF → pose_math+状态机 → world → 样件 → aruco_detector → MoveIt配置 → manipulation(pick_place+pymoveit2+吸盘) → navigation(Nav2+waypoints) → bringup(mission+一键launch) → README

## 进度日志
- Phase 0-6 + 抓取/导航/集成全部完成,8 包全量 build 通过,25 测试全绿。
- 项目代码骨架与集成层完整;剩余为运行时(GUI)验证与标定,留用户收尾。
- 下一步(夜间继续):对抗审查代码 + 重试替代方案调研。
