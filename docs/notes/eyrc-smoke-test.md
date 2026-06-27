# eyrc 冒烟测试笔记

> 日期:2026-06-28 ｜ 工作空间:`~/projects/eyrc_smoke_ws` ｜ 蓝本:eyrc-24-25-logistic-cobot(已持久化,219MB,已删 .git)

## 环境与依赖
- ROS2 Humble + Gazebo Classic 11
- `requirements.sh` 依赖**全部已满足,无需 sudo**:gazebo-ros/plugins/xacro/tf-transformations/transforms3d/tf_transformations/warehouse_ros_sqlite 均 ✓
- 未做:SkyX_Moon 天空盒贴图复制(需 `sudo cp` 到 `/usr/share/gazebo-11/media/skyx/`,仅影响天空材质,功能无关,暂跳过)

## 构建结果
命令:`colcon build --symlink-install --packages-skip my_auv_sim realsense_gazebo_plugin`
- ✅ **10 个核心包成功**:ebot_description, ebot_nav2, eyantra_warehouse, linkattacher_msgs, pymoveit2, tf_broadcaster_pkg, ur5_control, ur_description, ur_moveit_config, ur_simulation_gazebo
- ⚠️ realsense_gazebo_plugin 失败(源文件不全)
- ⏭️ my_auv_sim 跳过(AUV 仿真,与本项目无关)

## 踩坑与处理(eyrc 公开仓库不完整)
1. **ur_description 缺 `scripts/`**:CMakeLists 安装 PyArmor 加密的 `scripts/armed_1/spawner_box_*.py`,但该目录被作者从公开仓库移除。
   - 处理:改写 `ur_description/CMakeLists.txt`,移除对缺失加密脚本的安装项(见该文件注释)。
   - 影响:eyrc 的 task1b/task1c 抓取 demo **无法开箱即用复现**(缺 spawn 箱子的加密脚本)。我们自建场景/样件,不受影响。
2. **realsense_gazebo_plugin 缺 `gazebo_ros_realsense.cpp`**:只剩 `RealSensePlugin.cpp`,缺 ROS 封装层。
   - 处理:skip 该插件;RGB-D 相机改用标准 `gazebo_ros_camera`(已装)。

## 关键结论
- **eyrc 核心组件齐全、可编译、可 source**(导航/抓取/感知逻辑/MoveIt 全在)。
- **eyrc 不是"拿来即跑的完整 demo"**(spawner_box 加密缺失),但作为**组件移植蓝本**完全胜任——这正是方案A的定位。
- 导航栈(ebot_description + ebot_nav2 + 预建 map.pgm)完整,是冒烟验证导航的基础。

## 导航冒烟入口
- 世界+机器人:`ros2 launch ebot_description ebot_gazebo_launch.py`
- 导航栈:`ros2 launch ebot_nav2 ebot_bringup_launch.py`
- 预建地图:`install/ebot_nav2/share/ebot_nav2/maps/map.pgm` ✓

## 待补(后续 Task 填充)
- [ ] 导航冒烟结果(AMCL 定位 + Nav2 到点)
- [ ] 抓取组件验证(MoveIt 规划 / aruco_detector 启动 / linkattacher)
- [ ] eyrc 关键文件实际路径与参数(nav2_params / ekf / aruco_detector)
- [ ] UR5 在 eyrc 中的安装高度(供基座立柱定高)
