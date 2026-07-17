---
name: ros2-gazebo-sim-runbook
description: Use when ROS 2 Humble Gazebo Classic simulations need stable multi-process orchestration, gzserver residue checks, world/move_group/visualizer/mission startup order, topic probes, /gazebo/model_states monitoring, GAZEBO_MASTER_URI isolation, interruption recovery, or launch_testing E2E hardening against DDS races and false positives.
---

# ROS 2 Gazebo sim runbook

来源：2026-07 lab_cobot_ws Goal-P/S/T/V 实测。未在本轮复核的项标注“未验证”。

## 1. 仿真启动前预检

- 症状：E2E 卡在 `NAV_TO_PICK` 直到 420s 超时，或第二个 Gazebo 起不来。
- 根因：上一次中断后残留 `gzserver` / `gzclient`，新的 launch 连接到旧 world 或端口冲突。
- 修法：先查再清。
  ```bash
  pgrep -af 'gzserver|gzclient|world[.]launch[.]py|move_group|mecanum_wheel_visualizer|lab_cobot[.]launch[.]py' || true
  pkill -9 -x gzserver || true; pkill -9 -x gzclient || true
  ```
- 陷阱：**pkill 一律 `-x` 精确匹配进程名，禁用 `-f 'gzserver|gzclient'`**——`-f` 匹配完整命令行，会命中调用者自己的 wrapper shell（命令行里含该字符串）导致自杀，exit 137（双方实测）。`pgrep -af` 只查看不杀，可以用 `-f`。
- E2E 内也要 fail-fast：`test_honest_e2e_launch.py` 在 `generate_test_description()` 里 `pgrep -x gzserver`，发现残留就抛错并提示清理命令。

## 2. 手动多进程启动顺序

- 症状：MoveIt、控制器、topic probe 偶发找不到服务或 topic。
- 根因：ROS overlay、DDS discovery、Gazebo spawn、controller spawner 需要按依赖顺序稳定启动。
- 修法：按这个顺序起，另开终端或独立 exec session：
  ```bash
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  export ROS_LOCALHOST_ONLY=1
  ros2 daemon stop || true
  ros2 daemon start
  ```
  ```bash
  ros2 launch lab_cobot_gazebo world.launch.py gui:=false require_finger_contact:=true
  ```
  ```bash
  ros2 launch lab_cobot_moveit move_group.launch.py use_sim_time:=true
  ```
  ```bash
  ros2 run lab_cobot_bringup mecanum_wheel_visualizer --ros-args -p use_sim_time:=true -p publish_odom:=false
  ```
  ```bash
  ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false require_finger_contact:=true use_tactile_grasp:=true
  ```
- 验证点：world 日志应出现 `/gazebo/model_states`，控制器 active，grasp_fix 加载，左右 bumper topic 可 echo。

## 3. 中断恢复清单

- 症状：用户中断、tool abort、长命令卡住后，需要确认后台还剩什么。
- 根因：launch 进程、Gazebo server、MoveGroup 可能不随 shell 中断干净退出。
- 修法：先做状态自检，不要直接叠加新 launch。
  ```bash
  pgrep -af 'gzserver|gzclient|world[.]launch[.]py|move_group|mecanum_wheel_visualizer|lab_cobot[.]launch[.]py' || true
  ros2 node list || true
  ros2 topic list || true
  ```
- 若是旧 world 仍在：停止对应 session，必要时只清 Gazebo：`pkill -9 -x gzserver || true; pkill -9 -x gzclient || true`。
- 若是 colcon build/test 被中断：先接管原 session 或等它结束；不要并发跑第二个 colcon 写同一 build/install。

## 4. topic probe 与 model_states 监控

- 症状：任务失败只看到 `FAILED`，不知道是导航、感知、抓取还是物理爆炸。
- 根因：缺少同步采样 `/task/status`、`/gripper/contact/status`、`/gazebo/model_states`、`/rosout`。
- 修法：probe 至少记录：
  - `/task/status`：状态序列，期望最后 `DONE`。
  - `/gripper/contact/status`：`attached/refused/released`，多物体时断言只 attach `aruco_sample`。
  - `/gazebo/model_states`：`aruco_sample` pose/twist；T-4 目标是 twist `<1 m/s`。
  - `/rosout`：只保留 `mission_node` / `pick_place_node` 中 Pick、Place、MoveIt、夹爪、视觉、地图、任务相关日志。
- E2E 失败消息要带最近 pose/twist/contact/runtime logs；这比只报 `FAILED` 更可定位。

## 5. GAZEBO_MASTER_URI 隔离

- 症状：需要并行跑多个 Gazebo 或怀疑连接到旧 master。
- 根因：Gazebo Classic 默认 master URI 相同，进程会互相污染。
- 修法：给并行 world 设置不同 `GAZEBO_MASTER_URI`。
- 未验证：本轮没有完成并行多 Gazebo A/B；该规则来自项目既有实测约束，使用前仍要查当前 launch 是否透传环境。

## 6. launch_testing E2E 加固

- 症状：启动期服务读取单次 20s 假超时。
- 根因：DDS discovery 刚启动时不稳定。
- 修法：服务 `wait_for_service` 后，关键 `call_async` 用 3 次重试；`_assert_truth_pose_disabled()` 对 `/aruco_detector/get_parameters` 已这样做。

- 症状：常规 pytest 因 honest E2E 超时而失败，或失败原因被 ctest 总超时掩盖。
- 根因：重 E2E 预算约 500s，混在目录级 pytest 的短 timeout 里不够。
- 修法：CMake 中拆分注册：
  ```cmake
  ament_add_pytest_test(${PROJECT_NAME}_pytest test/
    ENV "PYTEST_ADDOPTS=-p no:anyio --ignore=${CMAKE_CURRENT_SOURCE_DIR}/test/test_honest_e2e_launch.py"
    TIMEOUT 120)
  ament_add_pytest_test(${PROJECT_NAME}_honest_e2e
    test/test_honest_e2e_launch.py
    ENV "PYTEST_ADDOPTS=-p no:anyio"
    TIMEOUT 600)
  ```

- 症状：E2E “看似 DONE”但走了作弊路径。
- 根因：只断言终态不够，可能读 truth pose、启 attach bridge、关闭 gravity、没有真实 camera detection。
- 修法：反作弊断言至少覆盖：
  - `use_gazebo_model_pose=False`
  - node list 不含 `gripper_attach_bridge`
  - `base_link` frame ArUco detection 数量 > 0
  - 样件最终在 B 台面盒内
  - `gravity_mode=True`
  - DL 模型存在时 `/perception/objects` 有消息；不存在时 `object_detector` 不运行
  - 多物体后任何 `attached ...` 都必须等于 `attached aruco_sample`
