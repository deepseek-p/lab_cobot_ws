# CS-202618 实验室移动协作机器人(lab_cobot)

> 面向实验室智能管理的协作机器人 —— **一体化麦轮移动机械臂跨工位识别抓取仿真系统**
> ROS 2 Humble · Gazebo Classic 11 · Nav2 · MoveIt 2 · ArUco 感知 · 真空吸盘

赛题:CS-202618 中车株洲电力机车有限公司「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」

## 1. 系统概述

一台**麦克纳姆全向底盘 + 基座立柱 + UR5e + 真空吸盘**的一体化移动机械臂,在实验室双工位场景中完成:

```
文本指令 → 导航到工位A → ArUco 识别样件 → 抓取 → 导航到工位B → 放置 → 返回 home
```

端到端数据流:
```
/task/instruction → mission_node(状态机)
   → Nav2(AMCL+EKF) → /cmd_vel → 麦轮底盘(planar_move)
   → aruco_detector → TF: base_link→obj_<id>
   → MoveIt2 + pymoveit2 → UR5e 轨迹 → 真空吸盘 attach
   → /task/status
```

## 2. 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 一体化机器人 URDF(麦轮底盘+立柱+UR5e+吸盘+激光/IMU/RGBD)+ SRDF |
| `lab_cobot_gazebo` | 实验室双工位 world + 带 ArUco 纹理样件 + spawn launch |
| `lab_cobot_navigation` | Nav2 配置(AMCL+EKF+costmap)+ 工位 waypoints + 导航 launch |
| `lab_cobot_moveit` | UR5e MoveIt2 配置(kinematics/ompl/controllers)+ move_group launch |
| `lab_cobot_perception` | ArUco 检测 + 针孔反投影 6D 位姿(pose_math)+ TF/PoseStamped |
| `lab_cobot_manipulation` | pick/place 执行(pymoveit2 + 真空吸盘) |
| `lab_cobot_bringup` | 跨工位任务状态机 + mission_node 编排 + 一键全栈 launch |
| `pymoveit2`(vendored) | MoveIt2 Python 接口(第三方,BSD-3,见 THIRD_PARTY_LICENSES.md) |

## 3. 环境要求

- Ubuntu 22.04 + ROS 2 Humble
- Gazebo Classic 11、MoveIt 2、Nav2、robot_localization、slam_toolbox(均 apt 安装)
- Python: opencv(cv2)、cv_bridge、numpy(本机已具备)

## 4. 构建

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> ⚠️ **已知坑**:若 pymoveit2 报 `failed to create symbolic link ... Is a directory`,
> 是 symlink-install 缓存污染。执行 `rm -rf build install && colcon build --symlink-install` 即可。

## 5. 运行

### 一键全栈(需 GUI / WSLg)
```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```
启动顺序:Gazebo+机器人+控制器 →(10s 后)move_group+Nav2+感知 →(15s 后)mission 编排。

### 发任务指令(另开终端)
```bash
source ~/projects/lab_cobot_ws/install/setup.bash
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
ros2 topic echo /task/status     # 观察状态机进展
```

### 分模块调试
```bash
# 仅看模型(RViz)
ros2 launch lab_cobot_description view_robot.launch.py
# 仅 Gazebo + 机器人
ros2 launch lab_cobot_gazebo world.launch.py
# 仅 MoveIt
ros2 launch lab_cobot_moveit move_group.launch.py
# 仅导航
ros2 launch lab_cobot_navigation navigation.launch.py
```

## 6. 测试

纯逻辑单元测试(headless,共 25 项):
```bash
cd ~/projects/lab_cobot_ws/src
python3 -m pytest lab_cobot_perception/test/test_pose_math.py \
                  lab_cobot_bringup/test/test_task_state_machine.py \
                  lab_cobot_navigation/test/test_waypoints.py -v
```

## 7. 已验证 / 待运行时验证

**已 headless 自动验证**:
- URDF 结构(check_urdf,26 links)、world 加载、样件 SDF
- pose_math(8)、状态机(10)、waypoints(7)单元测试
- MoveIt 配置加载(MoveItConfigsBuilder 处理 URDF+SRDF+config)
- 各节点 import、launch 可构建、全量 colcon build

**待运行时(GUI)验证**(用户在 WSLg 进行):
- Gazebo 中机器人正确落地、控制器激活、传感器出数据
- AMCL 定位 + Nav2 跨工位到点(需先建图替换占位 map)
- MoveIt 抓取轨迹 + 真空吸附效果(吸盘姿态四元数 DOWN_QUAT 可能需标定)
- ArUco 检测(样件贴码,相机内参标定)
- 端到端闭环连续成功率

## 8. 已知限制 / 后续

- 地图为占位(eyrc warehouse),需用 slam_toolbox 在 lab.world 重建实验室地图。
- 局部规划先用 DWB 差速兼容模式;麦轮全向潜力(vy)待升级。
- 抓取吸盘姿态、放置点坐标为预设值,需运行时标定。
- 增量方向:平行夹爪、颜色/点云真实识别、动态障碍、多对象、LLM 任务解析、轨道交通主题场景。

## 9. 设计与计划文档

- `docs/plans/2026-06-28-mobile-manipulator-cross-station-design.md` — 设计
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-plan.md` — 实现计划
- `docs/notes/eyrc-smoke-test.md` — 蓝本冒烟测试结论
- `docs/notes/overnight-progress.md` — 自主推进进度

许可:本仓库自有代码 Apache-2.0;第三方组件见 `THIRD_PARTY_LICENSES.md`。
