# 多任务

## ROS2 协作机器人多任务操作与验证

本文档总结当前 ROS2 协作机器人工作区中多任务操作分支的最终交付状态，记录已经实现的任务链、验证工具、基准测试结果以及运行时清理结论，用于比赛队员、评委和后续维护人员快速理解当前版本。

## 1. 项目概述

本分支在原工作区基础上新增并完善了以下多任务操作与验证能力：

- 正式 A→B 物料搬运任务。
- 黄色方块卡槽插入验证任务。
- 试管插入验证任务。
- 多目标抓取验证。
- 抓取基准测试执行与分析。
- 机械臂操作重复性验证。
- 接触与力反馈验证。
- 面向验证任务的导航与定位集成。
- 控制器启动稳定化。
- Gazebo 抓取/接触集成与 fixed-joint 物体保持。

验证模块与正式 `/task/instruction` 任务路径保持隔离，使基准测试和实验逻辑不会重写比赛正式任务接口。

## 2. 系统架构

当前系统按以下层级组织：

- 任务层：`mission_node.py`、`task_planner.py`、`task_state_machine.py` 以及各任务专用验证节点。
- 导航层：Nav2 launch 与路线配置，包括用于验证起始位姿的 AMCL initial pose 支持。
- 操作层：`pick_place_node.py`、MoveIt2 规划、Cartesian 接近/抬升执行以及任务专用几何配置。
- 接触/抓取层：`gripper_driver.py`、手指接触 topic、holding validation 以及 fixed-joint attach 确认。
- Gazebo 集成层：world launch、物体模型、接触/抓取插件以及机器人描述中的接触传感配置。
- 验证/基准层：抓取验证、抓取 benchmark、重复性验证、力控分析和结果资产。

## 3. 正式 A→B 任务

正式任务接口保持为：

- Topic: `/task/instruction`
- 流程：指令解析 → 导航 → 感知 → 抓取 → 搬运 → 放置 → 返回/home

主要文件：

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_task/lab_cobot_task/task_planner.py`
- `src/lab_cobot_task/lab_cobot_task/task_state_machine.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`

正式 A→B 接口保持不变。独立验证任务需要显式启动，不替代正式任务路径。

## 4. 黄色方块卡槽插入任务

- Topic: `/yellow_cube_slot_validation/target`
- Command: `insert_yellow_cube`
- 成功状态：`YELLOW_CUBE_SLOT_SUCCESS`

最终任务结构：

- 安全 Station A 初始生成。
- 基于方块相对位姿的精对接。
- 预抓取定位。
- Cartesian 下降。
- 基于 tactile/contact 的抓取。
- fixed-joint attach。
- holding validation。
- Cartesian 抬升。
- 抓取后的后退动作。
- 公共 aging-zone 导航路线。
- AMCL 初始位姿对齐。
- aging-zone 精对接。
- 基于卡槽相对位姿的放置。
- -90 degree 旋转后的深度插入。
- 方形物体 yaw 对称性验证。
- 释放。
- 垂直撤离。

公共 aging route 保持为：

`aging_zone_south_entry -> aging_zone_east_corridor -> aging_zone`

## 5. 试管插入任务

- Topic: `/tube_insert_validation/target`
- Command: `insert_test_tube`

主要文件：

- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_validation_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_config.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_base_feasibility.py`
- `src/lab_cobot_moveit/lab_cobot_moveit/tube_insert_scene.py`

试管任务包含专用目标几何配置、MoveIt planning-scene 支持、底盘可达性评估、插入执行与验证。

## 6. 多目标抓取验证

- Topic: `/grasp_validation/target`

支持的抓取目标：

- `tooling_fixture_box`
- `tooling_hand_tools`
- `board_test_fixture`
- `high_voltage_probe_kit`
- `material_spare_igbt`

最终验证基准目标：

- `high_voltage_probe_kit`
- `tooling_fixture_box`
- `material_spare_igbt`

“支持的目标”和“最终验证基准目标”是两个不同概念。历史兼容目标仍可用于手动验证，但只有建立了冻结验证结果集的目标才计入最终基准指标。

## 7. 最终抓取基准结果

冻结结果目录：

`results/chapter6/final_baseline/`

| 目标 | 成功次数 | 试验次数 | 成功率 |
| --- | ---: | ---: | ---: |
| `high_voltage_probe_kit` | 10 | 10 | 100.00% |
| `tooling_fixture_box` | 10 | 10 | 100.00% |
| `material_spare_igbt` | 8 | 10 | 80.00% |
| Overall | 28 | 30 | 93.33% |

补充 lift 统计：

- Primary lift: 20/30。
- State-verified lift: 8/30。
- 最终失败：`CONTACT_LEFT_ONLY = 2`。

总体抓取成功率为 28/30 = 93.33%，满足复杂物体抓取成功率不低于 90% 的指标要求。

## 8. Fixture 抓取几何配置

冻结的 fixture 抓取配置采用有意设置的偏置 local grasp point：

`grasp_point_local = (0.0, 0.06000000, 0.00007236)`

该配置对应 `tooling_fixture_box` 最终 10/10 SUCCESS 结果。旧的中心线 fixture 期望仅作为历史背景保留，不再作为当前验证几何契约。

## 9. 保留的历史兼容目标

`tooling_hand_tools` 仍然：

- 由抓取目标配置支持。
- 作为 Gazebo simulation asset 保留。
- 可通过抓取验证接口进行手动 validation。

它不属于最终验证 baseline、最终报告指标、正式 A→B 任务、Yellow task 或 Tube task。历史验证未为该目标建立成功的冻结 baseline，因此测试只将其作为受支持的历史兼容目标进行基本一致性检查，而不是作为已验证成功契约。

## 10. 基准测试与重复性验证

主要文件：

- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_benchmark_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/manipulation_repeatability_node.py`
- `tools/manipulation_validation_analyze_results.py`

正式 executable：

- `grasp_benchmark_node`
- `manipulation_repeatability_node`

旧的 `chapter6_grasp_benchmark_node` 和 `chapter6_repeatability_node` executable aliases 已删除。新的验证输出默认使用 `results/manipulation_validation/`，历史 `results/chapter6/` 数据继续保留。

## 11. 力与接触反馈验证

主要文件：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_validation_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_benchmark_node.py`
- `tools/force_control_validation_analysis.py`

已实现能力：

- 通过 `/gripper/contact/fingers` 获取手指接触状态。
- 基于接触状态的夹爪闭合。
- fixed-joint attach 确认。
- 抓取后的 holding validation。
- force/contact 时间序列记录。
- 实验性的 force-feedback gripper closure mode。

当前 force-control validation 使用 `VIRTUAL_ESTIMATE` 作为 force source。它是夹爪力反馈闭合的仿真原型，不表示已经完成真实 FSR402 硬件力控，也不表示实现了机械臂完整 Cartesian hybrid force-position control。

force-control 原型汇总：

| 目标 | 试验次数 | 力感知决策 | 主动调整 | 力保持 | 任务成功 |
| --- | ---: | ---: | ---: | ---: | ---: |
| `high_voltage_probe_kit` | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| `tooling_fixture_box` | 3 | 3/3 | 0/3 | 3/3 | 3/3 |
| `material_spare_igbt` | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| Overall | 9 | 9/9 | 6/9 | 9/9 | 9/9 |

Fixture 三次试验均通过检测当前 force estimate 已经达到或超过保持条件进入 force hold，因此不计为存在非零命令增量的主动调整试验。

## 12. 导航与定位

主要 launch 文件：

- `src/lab_cobot_navigation/launch/navigation.launch.py`

导航层支持 AMCL initial pose launch configuration。默认 HOME 行为保持不变。Yellow validation 可以将 Gazebo initial pose 与 AMCL initial pose 对齐，同时继续使用共享 public aging-zone route。

## 13. 控制器启动

主要文件：

- `src/lab_cobot_gazebo/scripts/controller_bootstrap.py`

controller bootstrap sequence 对以下 controller 执行串行 load、configure、activate 和 verification：

- `joint_state_broadcaster`
- arm joint trajectory controller
- gripper controller
- wheel controller

该机制用于避免 launch 过程中的 controller startup ordering races。

## 14. Gazebo 抓取集成

主要文件：

- `src/lab_cobot_gazebo/src/lab_cobot_grasp_fix.cpp`
- `src/lab_cobot_gazebo/launch/world.launch.py`
- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`

Gazebo 集成提供 contact detection、fixed-joint object attach、release/detach handling 以及面向多物体候选目标的支持。

## 15. 运行时清理

已删除的运行时组件：

- `aging_rack_insert_validation_node`
- `aging_rack_insert_config`
- `chapter6_grasp_benchmark_node` executable alias
- `chapter6_repeatability_node` executable alias

保留的共享基础设施：

- aging_rack Gazebo model 与 world objects。
- 共享 rack 与 slot scene geometry。
- `table_scene_initializer.py`
- `build_aging_rack_insert_validation_planning_scene(...)`
- `validation_scene_kind == aging_rack_insert`

shared aging rack infrastructure 仍然保留，因为 Yellow cube slot validation 仍使用 shared rack scene。

## 16. 重要新增与修改文件

Bringup：

- `src/lab_cobot_bringup/launch/lab_cobot.launch.py`：validation launch switches 与任务专用启动连接。
- `src/lab_cobot_bringup/CMakeLists.txt`：formal、grasp、tube、Yellow、benchmark 和 repeatability nodes 的 executable registration。
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_validation_node.py`：独立 grasp validation node。
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_target_config.py`：grasp target configuration 与 geometry helpers。
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_benchmark_node.py`：重复抓取 benchmark runner。
- `src/lab_cobot_bringup/lab_cobot_bringup/manipulation_repeatability_node.py`：manipulation repeatability validation node。
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_validation_node.py`：test tube insertion validation node。
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_config.py`：tube insertion task configuration。
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_base_feasibility.py`：tube base feasibility tool。
- `src/lab_cobot_bringup/lab_cobot_bringup/yellow_cube_slot_validation_node.py`：Yellow cube slot validation node。

Manipulation：

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`：pick/place 与 validation manipulation integration。
- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`：contact-aware gripping 与实验性 force-feedback gripper closure。
- `src/lab_cobot_manipulation/lab_cobot_manipulation/yellow_cube_slot_config.py`：Yellow cube slot task configuration。

MoveIt：

- `src/lab_cobot_moveit/lab_cobot_moveit/table_scene_initializer.py`：shared planning scene initialization。
- `src/lab_cobot_moveit/lab_cobot_moveit/tube_insert_scene.py`：tube insertion planning scene support。
- `src/lab_cobot_moveit/lab_cobot_moveit/__init__.py`：package module initialization。

Navigation：

- `src/lab_cobot_navigation/launch/navigation.launch.py`：AMCL initial pose configuration support。

Gazebo：

- `src/lab_cobot_gazebo/launch/world.launch.py`：面向 contact/grasp validation 的 world launch integration。
- `src/lab_cobot_gazebo/src/lab_cobot_grasp_fix.cpp`：contact 与 fixed-joint grasp integration。
- `src/lab_cobot_gazebo/scripts/controller_bootstrap.py`：controller startup sequencing。
- `src/lab_cobot_gazebo/worlds/*.world`：world object 与 task scene updates。
- `src/lab_cobot_gazebo/models/aging_rack/model.sdf`：保留的 shared rack scene asset。

Description/URDF：

- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`：面向 contact/grasp integration 的 robot description updates。

Tests：

- `src/lab_cobot_bringup/test/test_grasp_validation_node.py`：active validated target 与 legacy supported target tests。
- `src/lab_cobot_bringup/test/test_grasp_target_config.py`：grasp target configuration contract tests。
- `src/lab_cobot_bringup/test/test_tube_insert_validation_node.py`：tube task validation tests。
- `src/lab_cobot_bringup/test/test_yellow_cube_slot_validation_node.py`：Yellow task validation tests。
- `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`：launch registration tests。
- `src/lab_cobot_gazebo/test/*.py`：Gazebo、perception model、grasp plugin 与 world launch tests。
- `src/lab_cobot_manipulation/test/test_gripper_driver.py`：gripper/contact behavior tests。
- `src/lab_cobot_moveit/test/test_table_scene_initializer.py`：shared planning scene tests。

Tools：

- `tools/manipulation_validation_analyze_results.py`：manipulation validation result analysis。
- `tools/force_control_validation_analysis.py`：force-control time-series analysis 与 plotting。

Results：

- `results/chapter6/final_baseline/`：冻结的 30-trial grasp baseline。
- `results/chapter6/force_control/`：筛选后的 force-control prototype evidence。
- `results/chapter6/report_assets/`：报告可用的表格与图片。
- `results/manipulation_validation/`：未来经过去章节化命名后的 manipulation validation outputs 默认位置。

Documentation：

- `MULTIPLE TASKS.md`：最终分支说明文档。
- 历史 Chapter 6 documents and logs 保留用于可追溯性。

## 17. 验证状态

冻结前最近一次静态与回归验证结果：

- 定向抓取测试：35 passed。
- 回归测试：245 passed。
- Build: `lab_cobot_manipulation` PASS。
- Build: `lab_cobot_bringup` PASS。

freeze/upload 阶段未重新运行真实 Gazebo 实验。

## 18. 结果资产

正式保留的结果资产：

- `results/chapter6/final_baseline/`
- `results/chapter6/force_control/probe_n3_verified/`
- `results/chapter6/force_control/fixture_n3_verified/`
- `results/chapter6/force_control/igbt_n3_verified/`
- `results/chapter6/force_control/force_control_n3_summary.md`
- `results/chapter6/report_assets/`

历史 `results/chapter6/` 数据保留用于可追溯性。新的 manipulation validation outputs 应使用 `results/manipulation_validation/`。

## 19. 不纳入版本控制的文件

以下生成文件或本地临时产物不应纳入 Git version control：

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `*.pyc`
- 大部分 raw `.ros_logs/`
- temporary debug logs
- 未被选为 final evidence 的 temporary CSV/PNG outputs

最终 baseline 与报告结果资产独立保留，不与 transient build/runtime outputs 混用。

## 20. 任务入口

正式 A→B：

- Topic: `/task/instruction`

Yellow cube slot validation：

- Topic: `/yellow_cube_slot_validation/target`
- Command: `insert_yellow_cube`

Test tube insertion validation：

- Topic: `/tube_insert_validation/target`
- Command: `insert_test_tube`

Grasp validation：

- Topic: `/grasp_validation/target`

Benchmark 与 repeatability executables：

- `ros2 run lab_cobot_bringup grasp_benchmark_node`
- `ros2 run lab_cobot_bringup manipulation_repeatability_node`

## 21. 最终状态

- Formal A-to-B: PRESERVED。
- Yellow cube slot: VALIDATED。
- Tube insertion: VALIDATED。
- Grasp baseline: 28/30 = 93.33%。
- Force-control prototype: 9/9 task success with virtual force estimate。
- Targeted grasp tests: 35 passed。
- Regression: 245 passed。
- Build: PASS。
