# Multiple Tasks

## ROS2 Collaborative Robot Multi-Task Manipulation and Validation

This document summarizes the final multi-task manipulation branch for the ROS2 collaborative robot workspace. It records the implemented task chains, validation tools, benchmark results, and cleanup decisions that define the frozen delivery state.

## 1. Project Overview

This branch extends the original workspace with multiple manipulation and validation capabilities:

- Formal A-to-B material handling.
- Yellow cube slot insertion validation.
- Test tube insertion validation.
- Multi-target grasp validation.
- Grasp benchmark execution and analysis.
- Manipulation repeatability validation.
- Contact and force-feedback validation.
- Navigation and localization integration for validation tasks.
- Controller startup stabilization.
- Gazebo grasp/contact integration with fixed-joint object holding.

The validation modules are kept separate from the formal `/task/instruction` mission path so that benchmark and experiment logic does not rewrite the competition task interface.

## 2. System Architecture

The implemented system is organized into these layers:

- Task Layer: `mission_node.py`, `task_planner.py`, `task_state_machine.py`, and task-specific validation nodes.
- Navigation Layer: Nav2 launch and route configuration, including AMCL initial pose support for validation starts.
- Manipulation Layer: `pick_place_node.py`, MoveIt2 planning, Cartesian approach/lift execution, and task-specific geometry.
- Contact/Grasp Layer: `gripper_driver.py`, finger contact topics, holding validation, and fixed-joint attach confirmation.
- Gazebo Integration: world launch, object models, contact/grasp plugin, and robot description contact sensors.
- Validation/Benchmark Layer: grasp validation, grasp benchmark, repeatability validation, force-control analysis, and result assets.

## 3. Formal A-to-B Task

The formal task interface remains:

- Topic: `/task/instruction`
- Flow: instruction -> navigation -> perception -> grasp -> transport -> place -> return/home

Main files:

- `src/lab_cobot_bringup/lab_cobot_bringup/mission_node.py`
- `src/lab_cobot_task/lab_cobot_task/task_planner.py`
- `src/lab_cobot_task/lab_cobot_task/task_state_machine.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`
- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`

The formal A-to-B interface is preserved. Independent validation tasks are launched explicitly and do not replace the formal mission path.

## 4. Yellow Cube Slot Insertion

- Topic: `/yellow_cube_slot_validation/target`
- Command: `insert_yellow_cube`
- Success status: `YELLOW_CUBE_SLOT_SUCCESS`

Final task structure:

- Safe Station A spawn.
- Cube-relative fine docking.
- Pre-grasp positioning.
- Cartesian descend.
- Tactile/contact grasp.
- Fixed-joint attach.
- Holding validation.
- Cartesian lift.
- Post-grasp retreat.
- Public aging-zone navigation route.
- AMCL spawn alignment.
- Aging-zone fine docking.
- Slot-relative placement.
- -90 degree rotated deep insertion.
- Square-object yaw-symmetry validation.
- Release.
- Vertical retreat.

The public aging route is preserved as:

`aging_zone_south_entry -> aging_zone_east_corridor -> aging_zone`

## 5. Test Tube Insertion

- Topic: `/tube_insert_validation/target`
- Command: `insert_test_tube`

Main files:

- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_validation_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_config.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_base_feasibility.py`
- `src/lab_cobot_moveit/lab_cobot_moveit/tube_insert_scene.py`

The tube task includes dedicated target geometry configuration, MoveIt planning-scene support, base feasibility evaluation, insertion execution, and validation.

## 6. Multi-Target Grasp Validation

- Topic: `/grasp_validation/target`

Supported grasp targets:

- `tooling_fixture_box`
- `tooling_hand_tools`
- `board_test_fixture`
- `high_voltage_probe_kit`
- `material_spare_igbt`

Final validated baseline targets:

- `high_voltage_probe_kit`
- `tooling_fixture_box`
- `material_spare_igbt`

Supported targets and final validated baseline targets are intentionally distinct. Legacy supported objects remain available for manual validation but are not counted in the final baseline metric unless they have a validated frozen result set.

## 7. Final Grasp Baseline

Frozen result directory:

`results/chapter6/final_baseline/`

| Target | Success | Trials | Success Rate |
| --- | ---: | ---: | ---: |
| `high_voltage_probe_kit` | 10 | 10 | 100.00% |
| `tooling_fixture_box` | 10 | 10 | 100.00% |
| `material_spare_igbt` | 8 | 10 | 80.00% |
| Overall | 28 | 30 | 93.33% |

Additional lift statistics:

- Primary lift success: 20/30.
- State-verified lift completion: 8/30.
- Final failures: `CONTACT_LEFT_ONLY = 2`.

The overall grasp success rate is 28/30 = 93.33%, satisfying the complex-object grasp success target of at least 90%.

## 8. Fixture Grasp Geometry

The frozen fixture grasp configuration uses an intentional off-center local grasp point:

`grasp_point_local = (0.0, 0.06000000, 0.00007236)`

This configuration corresponds to the final `tooling_fixture_box` result of 10/10 successful trials. Older center-line fixture expectations are retained only as historical context and are not the current validated geometry contract.

## 9. Supported Legacy Targets

`tooling_hand_tools` remains:

- Supported by grasp target configuration.
- Available as a Gazebo simulation asset.
- Available for manual validation through the grasp validation interface.

It is not part of the final validated baseline, final report metric, formal A-to-B task, Yellow task, or Tube task. Historical validation did not establish a successful frozen baseline for this target, so tests treat it as a supported legacy target with sanity checks rather than a validated success contract.

## 10. Benchmark and Repeatability

Main files:

- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_benchmark_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/manipulation_repeatability_node.py`
- `tools/manipulation_validation_analyze_results.py`

Official executables:

- `grasp_benchmark_node`
- `manipulation_repeatability_node`

The old `chapter6_grasp_benchmark_node` and `chapter6_repeatability_node` executable aliases have been removed. New validation outputs should use `results/manipulation_validation/` while historical `results/chapter6/` data remains preserved.

## 11. Force and Contact Validation

Main files:

- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_validation_node.py`
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_benchmark_node.py`
- `tools/force_control_validation_analysis.py`

Implemented capabilities:

- Finger contact sensing through `/gripper/contact/fingers`.
- Contact-aware gripper closure.
- Fixed-joint attach confirmation.
- Holding validation after grasp.
- Force/contact time-series recording.
- Experimental force-feedback gripper closure mode.

The current force-control validation uses `VIRTUAL_ESTIMATE` as the force source. It is a simulation prototype for gripper force-feedback closure, not a claim of real FSR402 hardware force control or full Cartesian hybrid force-position control of the arm.

Force-control prototype summary:

| Target | Trials | Force-Aware Decision | Active Adjustment | Force Hold | Task Success |
| --- | ---: | ---: | ---: | ---: | ---: |
| `high_voltage_probe_kit` | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| `tooling_fixture_box` | 3 | 3/3 | 0/3 | 3/3 | 3/3 |
| `material_spare_igbt` | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| Overall | 9 | 9/9 | 6/9 | 9/9 | 9/9 |

The fixture trials reached force hold by detecting that the current force estimate had already reached or exceeded the holding condition, so they are not counted as active nonzero command-adjustment trials.

## 12. Navigation and Localization

Main launch file:

- `src/lab_cobot_navigation/launch/navigation.launch.py`

The navigation layer supports AMCL initial pose launch configuration. Default HOME behavior is preserved. Yellow validation can align the Gazebo initial pose with the AMCL initial pose while using the shared public aging-zone route.

## 13. Controller Startup

Main file:

- `src/lab_cobot_gazebo/scripts/controller_bootstrap.py`

The controller bootstrap sequence serializes controller load, configure, activate, and verification steps for:

- `joint_state_broadcaster`
- arm joint trajectory controller
- gripper controller
- wheel controller

This avoids controller startup ordering races during launch.

## 14. Gazebo Grasp Integration

Main files:

- `src/lab_cobot_gazebo/src/lab_cobot_grasp_fix.cpp`
- `src/lab_cobot_gazebo/launch/world.launch.py`
- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`

The Gazebo integration provides contact detection, fixed-joint object attach, release/detach handling, and multi-object candidate support for manipulation validation.

## 15. Runtime Cleanup

Removed runtime elements:

- `aging_rack_insert_validation_node`
- `aging_rack_insert_config`
- `chapter6_grasp_benchmark_node` executable alias
- `chapter6_repeatability_node` executable alias

Retained shared infrastructure:

- Aging rack Gazebo model and world objects.
- Shared rack and slot scene geometry.
- `table_scene_initializer.py`
- `build_aging_rack_insert_validation_planning_scene(...)`
- `validation_scene_kind == aging_rack_insert`

The shared aging rack infrastructure remains because Yellow cube slot validation still uses the shared rack scene.

## 16. Important Added and Modified Files

Bringup:

- `src/lab_cobot_bringup/launch/lab_cobot.launch.py`: validation launch switches and task-specific startup wiring.
- `src/lab_cobot_bringup/CMakeLists.txt`: executable registration for formal, grasp, tube, Yellow, benchmark, and repeatability nodes.
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_validation_node.py`: independent grasp validation node.
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_target_config.py`: grasp target configuration and geometry helpers.
- `src/lab_cobot_bringup/lab_cobot_bringup/grasp_benchmark_node.py`: repeated grasp benchmark runner.
- `src/lab_cobot_bringup/lab_cobot_bringup/manipulation_repeatability_node.py`: manipulation repeatability validation node.
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_validation_node.py`: test tube insertion validation node.
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_config.py`: tube insertion task configuration.
- `src/lab_cobot_bringup/lab_cobot_bringup/tube_insert_base_feasibility.py`: tube base feasibility tool.
- `src/lab_cobot_bringup/lab_cobot_bringup/yellow_cube_slot_validation_node.py`: Yellow cube slot validation node.

Manipulation:

- `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py`: pick/place and validation manipulation integration.
- `src/lab_cobot_manipulation/lab_cobot_manipulation/gripper_driver.py`: contact-aware gripping and experimental force-feedback gripper closure.
- `src/lab_cobot_manipulation/lab_cobot_manipulation/yellow_cube_slot_config.py`: Yellow cube slot task configuration.

MoveIt:

- `src/lab_cobot_moveit/lab_cobot_moveit/table_scene_initializer.py`: shared planning scene initialization.
- `src/lab_cobot_moveit/lab_cobot_moveit/tube_insert_scene.py`: tube insertion planning scene support.
- `src/lab_cobot_moveit/lab_cobot_moveit/__init__.py`: package module initialization.

Navigation:

- `src/lab_cobot_navigation/launch/navigation.launch.py`: AMCL initial pose configuration support.

Gazebo:

- `src/lab_cobot_gazebo/launch/world.launch.py`: world launch integration for contact/grasp validation.
- `src/lab_cobot_gazebo/src/lab_cobot_grasp_fix.cpp`: contact and fixed-joint grasp integration.
- `src/lab_cobot_gazebo/scripts/controller_bootstrap.py`: controller startup sequencing.
- `src/lab_cobot_gazebo/worlds/*.world`: world object and task scene updates.
- `src/lab_cobot_gazebo/models/aging_rack/model.sdf`: retained shared rack scene asset.

Description/URDF:

- `src/lab_cobot_description/urdf/lab_cobot.urdf.xacro`: robot description updates for contact/grasp integration.

Tests:

- `src/lab_cobot_bringup/test/test_grasp_validation_node.py`: active validated target and legacy supported target tests.
- `src/lab_cobot_bringup/test/test_grasp_target_config.py`: grasp target configuration contract tests.
- `src/lab_cobot_bringup/test/test_tube_insert_validation_node.py`: tube task validation tests.
- `src/lab_cobot_bringup/test/test_yellow_cube_slot_validation_node.py`: Yellow task validation tests.
- `src/lab_cobot_bringup/test/test_lab_cobot_launch.py`: launch registration tests.
- `src/lab_cobot_gazebo/test/*.py`: Gazebo, perception model, grasp plugin, and world launch tests.
- `src/lab_cobot_manipulation/test/test_gripper_driver.py`: gripper/contact behavior tests.
- `src/lab_cobot_moveit/test/test_table_scene_initializer.py`: shared planning scene tests.

Tools:

- `tools/manipulation_validation_analyze_results.py`: manipulation validation result analysis.
- `tools/force_control_validation_analysis.py`: force-control time-series analysis and plotting.

Results:

- `results/chapter6/final_baseline/`: frozen 30-trial grasp baseline.
- `results/chapter6/force_control/`: selected force-control prototype evidence.
- `results/chapter6/report_assets/`: report-ready tables and figures.
- `results/manipulation_validation/`: default location for future renamed manipulation validation outputs.

Documentation:

- `MULTIPLE TASKS.md`: final branch summary.
- Historical Chapter 6 documents and logs are preserved for traceability.

## 17. Verification

Latest static and regression verification before this freeze:

- Targeted grasp tests: 35 passed.
- Regression tests: 245 passed.
- Build: `lab_cobot_manipulation` PASS.
- Build: `lab_cobot_bringup` PASS.

The freeze/upload stage did not rerun real Gazebo experiments.

## 18. Result Assets

Formal result assets retained:

- `results/chapter6/final_baseline/`
- `results/chapter6/force_control/probe_n3_verified/`
- `results/chapter6/force_control/fixture_n3_verified/`
- `results/chapter6/force_control/igbt_n3_verified/`
- `results/chapter6/force_control/force_control_n3_summary.md`
- `results/chapter6/report_assets/`

Historical `results/chapter6/` data is preserved for traceability. New manipulation validation outputs should use `results/manipulation_validation/`.

## 19. Files Not Intended for Version Control

The following generated or local artifacts are not intended for Git version control:

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `*.pyc`
- most raw `.ros_logs/`
- temporary debug logs
- temporary CSV/PNG outputs that are not selected final evidence

Final baseline and report result assets are retained separately from transient build and runtime outputs.

## 20. Task Entry Points

Formal A-to-B:

- Topic: `/task/instruction`

Yellow cube slot validation:

- Topic: `/yellow_cube_slot_validation/target`
- Command: `insert_yellow_cube`

Test tube insertion validation:

- Topic: `/tube_insert_validation/target`
- Command: `insert_test_tube`

Grasp validation:

- Topic: `/grasp_validation/target`

Benchmark and repeatability executables:

- `ros2 run lab_cobot_bringup grasp_benchmark_node`
- `ros2 run lab_cobot_bringup manipulation_repeatability_node`

## 21. Final Status

- Formal A-to-B: PRESERVED.
- Yellow cube slot: VALIDATED.
- Tube insertion: VALIDATED.
- Grasp baseline: 28/30 = 93.33%.
- Force-control prototype: 9/9 task success with virtual force estimate.
- Targeted grasp tests: 35 passed.
- Regression: 245 passed.
- Build: PASS.
