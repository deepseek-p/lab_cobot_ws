# MERGE_PLAN

目标: 合并 `origin/feature/navigation` 与 `origin/feature/grasp` 到本地集成分支时，保留两边有效改动，最小化回退和重复工作。

## 1. 分支统计

- `origin/main..origin/feature/navigation`: 116 files, `+371077/-719`
- `origin/main..origin/feature/grasp`: 58 files, `+9144/-307`

## 2. feature/navigation 修改范围

### robot description
- `src/lab_cobot_description/config/lab_cobot_controllers.yaml`
- `src/lab_cobot_description/meshes/mecanum3/*`
- `src/lab_cobot_description/urdf/inc/mecanum_base.xacro`
- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`
- `src/lab_cobot_description/test/test_mecanum_chassis_contracts.py`
- `src/lab_cobot_description/test/test_stage5_description_contracts.py`

### gazebo/world
- `src/lab_cobot_gazebo/CMakeLists.txt`
- `src/lab_cobot_gazebo/launch/world.launch.py`
- `src/lab_cobot_gazebo/package.xml`
- `src/lab_cobot_gazebo/scripts/*`
- `src/lab_cobot_gazebo/src/lab_cobot_mecanum_drive.cpp`
- `src/lab_cobot_gazebo/test/*`
- `src/lab_cobot_gazebo/worlds/lab*.world`
- `src/lab_cobot_gazebo/models/*`

### navigation
- `src/lab_cobot_navigation/config/*`
- `src/lab_cobot_navigation/launch/navigation.launch.py`
- `src/lab_cobot_navigation/maps/*`
- `src/lab_cobot_navigation/scripts/generate_map.py`
- `src/lab_cobot_navigation/test/*`

### bringup
- `src/lab_cobot_bringup/config/fastdds_no_shm.xml`
- `src/lab_cobot_bringup/lab_cobot_bringup/gripper_attach_bridge.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/task_planner.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/task_state_machine.py`
- `src/lab_cobot_bringup/launch/lab_cobot.launch.py`
- `src/lab_cobot_bringup/test/*`

### 其他
- `.gitignore`
- `README.md`
- `docs/superpowers/specs/*`
- `src/lab_cobot_moveit/*`
- `src/lab_cobot_perception/*`
- `tools/grasp_gap_probe.py`

## 3. feature/grasp 修改范围

### manipulation
- `src/lab_cobot_manipulation/lab_cobot_manipulation/contact_force_recorder.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/dynamic_arm_obstacle_node.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/joint_state_qos_relay.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/scene_obstacles.py`
- `src/lab_cobot_manipulation/package.xml`
- `src/lab_cobot_manipulation/setup.py`
- `src/lab_cobot_manipulation/test/*`

### MoveIt
- `src/lab_cobot_moveit/config/joint_limits.yaml`
- `src/lab_cobot_moveit/config/moveit_controllers.yaml`
- `src/lab_cobot_moveit/config/ompl_planning.yaml`
- `src/lab_cobot_moveit/launch/move_group.launch.py`
- `src/lab_cobot_moveit/test/test_moveit_config.py`
- `src/pymoveit2/pymoveit2/moveit2.py`
- `src/pymoveit2/test/test_moveit2_cartesian.py`

### gripper
- `src/lab_cobot_gazebo/finger_collision_config.hpp`
- `src/lab_cobot_gazebo/src/lab_cobot_grasp_fix.cpp`
- `src/lab_cobot_gazebo/test/test_finger_contact_gate.cpp`
- `src/lab_cobot_gazebo/test/test_grasp_envelope.cpp`
- `src/lab_cobot_gazebo/test/test_grasp_fix_plugin.py`

### mission
- `src/lab_cobot_bringup/lab_cobot_bringup/g4g5_result_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_bringup/launch/lab_cobot.launch.py`
- `src/lab_cobot_bringup/test/test_g4g5_result_node.py`
- `src/lab_cobot_bringup/test/*`

### 其他
- `README.md`
- `benchmarks/*`
- `docs/arm_pick_place_trajectory_plan.md`
- `src/lab_cobot_gazebo/CMakeLists.txt`
- `src/lab_cobot_gazebo/launch/world.launch.py`
- `src/lab_cobot_gazebo/package.xml`
- `src/lab_cobot_gazebo/worlds/grasp_place.world`
- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`
- `src/lab_cobot_description/test/test_stage5_description_contracts.py`
- `tools/g5_verify_offline.sh`

## 4. 共同修改文件

- `.gitignore`
- `README.md`
- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_bringup/launch/lab_cobot.launch.py`
- `src/lab_cobot_bringup/test/test_honest_e2e_launch.py`
- `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`
- `src/lab_cobot_bringup/test/test_mission_navigation_handoff.py`
- `src/lab_cobot_bringup/test/test_mission_place_pose.py`
- `src/lab_cobot_bringup/test/test_mission_retreat.py`
- `src/lab_cobot_description/test/test_stage5_description_contracts.py`
- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`
- `src/lab_cobot_gazebo/CMakeLists.txt`
- `src/lab_cobot_gazebo/launch/world.launch.py`
- `src/lab_cobot_gazebo/package.xml`
- `src/lab_cobot_gazebo/test/test_grasp_fix_plugin.py`
- `src/lab_cobot_gazebo/test/test_world_launch.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
- `src/lab_cobot_manipulation/test/test_gripper_driver.py`

## 5. 合并策略

### 5.1 共同文件的保留原则

#### `mission_node.py`
- 保留 navigation 的多工位导航框架、轴对齐转移、导航超时与 readiness 探测。
- 保留 grasp 的持物监控、检测缓存、失败兜底、`g4g5` 相关状态联动。
- 人工融合点:
  - `RETURN_HOME` / `NAV_TO_PLACE` / `NAV_TO_PICK` 的状态串接
  - `skip_visual_dock`、`nav_only`
  - `_failsafe_cleanup` 中的 release/detach 顺序
  - `TaskState` / `StationRouteTask` / `RouteState` 的枚举与状态机接口

#### `lab_cobot.launch.py`
- 保留 navigation 的分阶段启动、`skip_visual_dock`、`nav_only`、感知/导航分离启动逻辑。
- 保留 grasp 的 `g4g5_result_node`、`contact_force_recorder`、`dynamic_arm_obstacle_node`、`joint_state_qos_relay`。
- 人工融合点:
  - 启动顺序
  - 各节点 `condition`
  - `mission` 传参集合
  - `launch_perception` 与 `launch_moveit` 的交叉门控

#### `world.launch.py`
- 保留 navigation 的 `lighting_profile`、`enable_actor`、离线 Gazebo 资源配置、actor world 选择。
- 保留 grasp 的 `world`、`robot_x`、`robot_y`、`robot_yaw`、`controller_bootstrap`、`enable_contact_force`、`enable_lab_sensors`。
- 人工融合点:
  - world 文件选择逻辑
  - controller 启动链
  - `gzclient` 环境变量
  - `spawn_entity` 初始位姿

#### `gripper_driver.py`
- 保留 grasp 的 `hold_status` 语义、实时持物判定、release 语义。
- 保留 navigation 的 attach fallback 路径。
- 人工融合点:
  - `ContactGripperDriver.acquire_object()`
  - `release_object()`
  - `wait_until_holding()` / `is_holding_object()`
  - fallback 触发条件与状态恢复

#### `pick_place_node.py`
- 以 grasp 版本为主。
- 保留 navigation 补丁：`enable_attach_fallback` 透传。
- 人工融合点:
  - 持物监控与 scene attach/detach 时机
  - tactile / non-tactile 路径
  - `_move_*` 的 timeout 与 fallback 策略

#### `lab_cobot.urdf.xacro`
- 保留 grasp 的抓取 envelope、virtual force sensor、增强后的 grasp fix 目标集。
- 保留 navigation 的 `publish_odom_tf`、底盘/传感器相关契约。
- 人工融合点:
  - `lab_cobot_grasp_fix` 参数阈值
  - `enable_lab_sensors`
  - `object_model` 列表

#### `src/lab_cobot_gazebo/CMakeLists.txt` / `package.xml`
- 两边依赖与脚本安装取并集。
- 人工融合点:
  - `ament_cmake_python`
  - `rclpy` / `tf2_ros` / `gazebo_msgs`
  - 新脚本 install 条目
  - 测试注册与 timeout

### 5.2 纯测试文件
- 统一改成对最终合并后的 launch / URDF / gripper 行为断言。
- 删除互斥旧假设:
  - 旧的单 world、单 controller、单 grasp envelope、单 waypoints 断言

## 6. 建议合并顺序

1. 先合并非重叠文件。
2. 先处理 `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro` 与 `src/lab_cobot_gazebo/launch/world.launch.py`。
3. 再处理 `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py` 与 `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`。
4. 然后处理 `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py` 与 `src/lab_cobot_bringup/launch/lab_cobot.launch.py`。
5. 最后同步测试与 package/CMake 依赖。

## 7. 风险点

- `mission_node.py` 的状态机接口已经在两边都发生结构性变化，不能做简单文本级三方合并。
- `gripper_driver.py` 同时引入 hold heartbeat 和 attach fallback，状态语义需要统一，否则会出现“已抓取但系统认为未持物”的假失败。
- `world.launch.py` 同时承担 world 选择、controller bootstrap、spawn 位置与 Gazebo 环境变量，容易发生启动链顺序回退。
- `lab_cobot.urdf.xacro` 的抓取阈值和传感器开关会直接影响 grasp plugin 和测试契约，需要联动更新。

