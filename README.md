# CS-202618 实验室移动协作机器人

面向实验室智能管理的移动协作机器人仿真系统。项目基于 ROS 2 Humble、Gazebo Classic、Nav2、MoveIt 2 和 ArUco 感知，完成一体化麦克纳姆底盘 + UR5e + 平行双指夹爪在双工位场景中的识别、抓取、搬运、放置和返航。

赛题：CS-202618 中车株洲电力机车有限公司「面向实验室智能管理的协作机器人环境感知与动作规划方法研究」。

## 当前状态

当前实现已完成导航地图、任务编排、抓取失败恢复、contact grasp 抓取、ArUco 相机位姿、MoveIt 等运行链路的修复和加固，默认路径不读 Gazebo 真值、不使用瞬移吸附。

- `colcon build`：8 个包构建通过
- `colcon test`：562 个测试（含 1 个禁真值 Gazebo 端到端与 36 个插件 gtest 行为用例），15 跳过（其中 13 个为 cppcheck 环境性跳过，对应源码均有 gtest 覆盖）
- 静态地图为 `slam_toolbox` 实跑产物（来源与质量门见 `maps/map_provenance.yaml`）
- 默认 `use_truth_pose=false`、`use_sim_attach=false`：视觉走相机检测，抓取走 contact grasp 插件
- 默认 `use_tactile_grasp=true`、`require_finger_contact=true`：抓取 attach 需几何封套命中且左右指触觉探针均上报真实接触对（两开关必须同值）
- 默认 `use_wrist_detect=true`、`use_refine_detect=true`：腕相机 eye-in-hand 链为主路径——DETECT 移动到固定拍照位用腕相机定位（手眼变换经 Tsai 标定管线仿真验证），PICK 悬停后二次精修；任一环节失败自动降级 bench 全景相机/粗位姿
- 默认 `use_planning_scene_obstacles=true`：机械臂规划场景注入工位台面碰撞盒与持物样件附着盒，PICK 完成后先收臂再导航（消除持物横扫工位的穿模）
- headless 端到端（禁真值/禁吸附桥）已多次验证到 `DONE`
- `mission_node` 是正式任务入口；旧的 `pick_place` console entry point 已移除
- 末端执行器是平行双指夹爪；样件固定由 contact grasp 插件的 fixed-joint 实现

### 仿真保真度边界（诚实声明）

- **底盘**：默认由自定义插件按"cmd_vel → 麦轮逆解 → 轮速命令 → 正解"驱动，运动学链真实，但施加层是**有界位姿积分（`SetWorldPose`）**——底盘不受碰撞阻挡、`/odom` 为插件自身积分（零漂移）。不是麦轮滚子接触动力学。
- **抓取**：默认路径为触觉门控——attach 需要**几何封套命中且左右指触觉探针均上报真实几何接触对**（步进闭合，非一步到位）；接触判定是传感器上报的接触对，**不是力反馈**。物块全程保留质量/重力；fixed-joint 持有期样件碰撞响应被挂起、release 时原子恢复（消除固定关节与台面接触约束冲突）。放置采用悬空释放（名义落差约 5cm）避免约束冲突。不是真实摩擦力闭合。回退路径 `use_tactile_grasp:=false require_finger_contact:=false` 仍为"靠近即焊"，引用其行为时须注明。
- **机械臂避障**：规划场景中的工位台面碰撞盒为**名义几何**（检测/放置点定中心 + 已知台面尺寸放大裕度），不是感知重建；只约束 MoveIt 规划的臂轨迹，底盘仍不受碰撞阻挡（见底盘边界）。
- **视觉**：默认走 RGB-D + solvePnP 真实检测管线；`/gazebo/model_states` 仅保留为 `use_truth_pose:=true` 显式调试路径。腕相机拍照位检测为"拍一帧→手眼变换→开环执行"，不是接近过程闭环视觉伺服。

## 系统流程

```text
/task/instruction
  -> mission_node
  -> Nav2 AMCL/EKF + DWB
  -> /cmd_vel -> mecanum_wheel_visualizer(麦轮逆解)
  -> /wheel_velocity_controller/commands -> lab_cobot_mecanum_drive(正解+位姿积分)
  -> aruco_detector(bench RGB-D solvePnP) + wrist_aruco_detector(腕相机拍照位/精修) -> TF/PoseStamped
  -> MoveIt 2 + pymoveit2(规划场景含台面碰撞盒+持物样件附着盒)
  -> ContactGripperDriver
  -> /gripper_position_controller/commands
  -> lab_cobot_grasp_fix(几何封套+双指接触门控 -> fixed joint attach/detach)
  -> /task/status
```

任务状态机：

```text
NAV_TO_PICK -> DETECT -> PICK -> NAV_TO_PLACE -> PLACE -> RETURN_HOME -> DONE
```

失败时会执行退让、停止底盘、释放夹爪持有对象等清理动作，避免任务失败后留下不一致的仿真状态。

## 包结构

| 包 | 职责 |
|---|---|
| `lab_cobot_description` | 一体化机器人 URDF/SRDF：麦克纳姆底盘、立柱、UR5e、平行双指夹爪、激光、IMU、相机 |
| `lab_cobot_gazebo` | 双工位实验室 world、样件模型、自定义麦轮驱动/抓取插件、Gazebo spawn 和控制器启动 |
| `lab_cobot_navigation` | Nav2、AMCL、EKF、地图、工位 waypoints、导航 launch |
| `lab_cobot_moveit` | UR5e MoveIt 2 配置、controller 配置、`move_group` launch |
| `lab_cobot_perception` | ArUco 检测、相机反投影、Gazebo model pose fallback、TF/PoseStamped 输出 |
| `lab_cobot_manipulation` | pick/place 执行逻辑、MoveIt 调用、平行夹爪驱动边界和抓取序列 |
| `lab_cobot_bringup` | 一键全栈 launch、跨工位任务状态机、mission 编排、调试用软附着 bridge（默认关闭） |
| `pymoveit2` | vendored MoveIt 2 Python 接口，第三方许可见 `THIRD_PARTY_LICENSES.md` |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Nav2、MoveIt 2、robot_localization、slam_toolbox
- Python 3.10，OpenCV/cv_bridge/numpy
- 深度学习感知手动验证环境：ultralytics==8.4.90、open3d==0.19.0、
  torch==2.12.1+cu130、faster_whisper==1.2.1
- YOLO-World 离线权重路径：`~/lab_cobot_models/yolo_world_lab_slim.pt`
  （用 `tools/prepare_yolo_world_model.py` 制备，权重不进 git）

示例依赖安装：

```bash
sudo apt update
sudo apt install -y \
  ros-humble-navigation2 \
  ros-humble-nav2-bringup \
  ros-humble-moveit \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-cv-bridge \
  python3-opencv
```

## 构建

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

如果 `pymoveit2` 的 symlink-install 缓存污染导致符号链接创建失败，清理后重建：

```bash
rm -rf build install log
colcon build --symlink-install
```

## 运行

启动完整仿真：

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

另开终端发送任务：

```bash
source /opt/ros/humble/setup.bash
source ~/projects/lab_cobot_ws/install/setup.bash
ros2 topic pub --once /task/instruction std_msgs/msg/String "{data: '把样件从A送到B'}"
```

查看状态：

```bash
ros2 topic echo /task/status
```

headless 运行：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false
```

关键可选参数：

| 参数 | 默认值 | 作用 |
|---|---|---|
| `use_wrist_detect` | `true` | DETECT 阶段先移动到固定拍照位，使用腕相机顶面 ID=1 marker 定位；移动或检测失败时自动降级到 bench 相机。置 `false` 回退纯 bench 检测。 |
| `use_refine_detect` | `true` | 启用腕部精修相机、`/perception/wrist` ArUco 检测实例和 PICK 悬停后的位姿精修；失败时自动沿用粗位姿。 |
| `use_planning_scene_obstacles` | `true` | 向 MoveIt 规划场景注入工位台面碰撞盒与持物样件附着盒；置 `false` 回退规划对环境盲的旧行为。 |

只启动全栈但不启动任务节点：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false launch_mission:=false
```

分模块调试：

```bash
ros2 launch lab_cobot_description view_robot.launch.py
ros2 launch lab_cobot_gazebo world.launch.py
ros2 launch lab_cobot_moveit move_group.launch.py
ros2 launch lab_cobot_navigation navigation.launch.py
ros2 launch lab_cobot_navigation mapping.launch.py
```

## 验证

本机如果存在 user-level `anyio` pytest 插件问题，给 pytest/colcon 测试命令加上 `PYTEST_ADDOPTS='-p no:anyio'`。

完整构建和测试：

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
PYTEST_ADDOPTS='-p no:anyio' colcon build --cmake-force-configure
PYTEST_ADDOPTS='-p no:anyio' colcon test --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

地图检查：

```bash
python3 src/lab_cobot_navigation/maps/check_map.py
```

期望汇总：

```text
Summary: 562 tests, 0 errors, 0 failures, 15 skipped
PASS: map covers four walls, has low obstacle noise, and key points are free
```

## 运行注意

- `lab_cobot.launch.py` 默认延迟启动 MoveIt/Nav2/感知/mission，以等待 Gazebo、spawn 和控制器就绪。
- 向刚启动的栈发布任务建议用连发模式（`timeout 4 ros2 topic pub -r 2 /task/instruction ...`）；`--once` 单发可能因 DDS discovery 未完成而丢失。
- 平行夹爪通过 `gripper_position_controller` 驱动手指开合；样件固定由 `lab_cobot_grasp_fix` 插件在几何封套命中且双指触觉探针均上报接触对时创建 fixed joint 实现（接触判定为几何接触对，非力反馈，也非 SetEntityState 瞬移）；旧的 `gripper_attach_bridge` 仅在 `use_sim_attach:=true` 时作为调试后端启动。
- WSLg 下 launch 会设置 D3D12 和 Qt 相关环境变量以提高 Gazebo/RViz 稳定性。
- headless 结束 launch 时，MoveIt/rclpy 可能输出 SIGINT/shutdown 噪声；判断任务结果以 `/task/status` 是否到 `DONE` 为准。
- Gazebo GUI、物理步进和渲染性能会影响端到端任务耗时。

## 文档

以下为内部工作区文档（不随发布快照分发，发布仓库中不存在 `docs/` 目录）：

- `docs/HANDOVER.md`：项目交接和运行记录
- `docs/STATUS_HONEST.md`：阶段性状态说明
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-design.md`：系统设计
- `docs/plans/2026-06-28-mobile-manipulator-cross-station-plan.md`：实现计划
- `docs/notes/eyrc-smoke-test.md`：参考工程冒烟测试
- `docs/notes/overnight-progress.md`：历史推进记录

## 许可

本仓库自有代码采用 Apache-2.0；第三方组件许可见 `THIRD_PARTY_LICENSES.md`。
