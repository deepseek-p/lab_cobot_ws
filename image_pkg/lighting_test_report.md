# 无 GUI 三维定位误差测试报告

## ArUco 视觉抓取修复复测（2026-08-07）

已将 `lab_cobot_gazebo/models/aruco_sample/model.sdf` 中的 `aruco_sample` 从 `<static>true</static>` 改为 `<static>false</static>`，并重建、重启无 GUI Gazebo。Gazebo 服务 `/gazebo/get_model_properties` 确认运行时 `is_static=False`；同时增加模型回归测试，防止样件被再次配置为静态场景物。

本次仍使用纯视觉 ArUco 位姿（`use_gazebo_model_pose=false`）、腕部初检和二次修正（`use_wrist_detect=true`、`use_refine_detect=true`）及触觉夹爪（`use_tactile_grasp=true`），执行指令 `把样件从A送到B`。

| 流程阶段 | 实际结果 |
|---|---|
| A 工位导航、地图精停、视觉停靠 | 完成 |
| 腕部相机初检与二次修正 | `wrist_detect=hit`；首次修正 `( +1, -1, 0 ) mm` |
| 触觉闭合与附着 | `attached aruco_sample`，通过 |
| 抬升、退避并导航至 B 工位 | 完成 |
| B 工位放置 | `release accepted`，通过 |
| 返回 home | 任务最终状态 `DONE` |

结论：原失败由样件的静态物理属性造成，而非视觉识别或 `base_link` 坐标转换问题。将样件改为动态工件后，图示“初次定位 → 靠近 → 二次观测修正 → 抓取 → 搬运 → 放置”闭环已在仿真中实际完成。该测试证明任务闭环可用；它不等同于 1 mm TCP 精度通过，后者仍需在成功附着事件上单独采样并按 TCP—工件相对位姿评估。

## ArUco 视觉抓取闭环实测（2026-08-07）

本节以 `aruco_sample` 为例，实际启动 `mission_node`，并发送指令 `把样件从A送到B`。运行参数为 `use_wrist_detect=true`、`use_refine_detect=true`、`use_tactile_grasp=true`；ArUco 检测器的 `use_gazebo_model_pose=false`，因此初定位和二次定位均来自相机图像，而非 Gazebo 真值替代。

### 流程核验

| 图示步骤 | 本次运行证据 | 结果 |
|---|---|---|
| 导航至 A 工位并观察 | 导航、地图精停、视觉停靠完成；初始目标位姿为 `base_link` 下约 `(0.836, -0.009, 0.632) m` | 通过 |
| 腕部相机初次定位 | 日志 `wrist_detect=hit`，腕部位姿约 `(0.840, -0.010, 0.636) m` | 通过 |
| 转至 `base_link` 并靠近 | MoveIt 以 `gripper_tcp`、`frame=base_link` 执行预抓取 `(0.840, -0.010, 0.779) m` 与直线下降 | 通过 |
| 二次观察修正 | 首次二次观测 `refine=hit`，修正量 `( +1, -1, 0 ) mm`；后续重试也得到 7–17 mm 的横向修正 | 通过 |
| 闭合、附着、抬升搬运 | 所有触觉重试均出现“夹爪触觉闭合接触 aruco_sample”，但附着状态为 `refused none no_candidate_model`；任务最终 `FAILED` | **未通过** |

### 结论与原因

视觉闭环已经实际执行到“二次观察修正”步骤，故本次失败**不是** YOLO/ArUco 二维识别失败，也不是坐标未转到 `base_link`。失败发生在仿真抓取物理层：`lab_cobot_gazebo/models/aruco_sample/model.sdf` 将模型定义为 `<static>true</static>`，而 `lab_cobot_grasp_fix` 插件在候选筛选时会跳过所有静态模型。因此候选集合为空，必然发布 `refused none no_candidate_model`，无法生成 `attached aruco_sample`。

本次没有任何实际附着确认，故 TCP—工件相对抓取误差、1 mm 通过率均为**不可评估（0 个有效抓取样本）**；不能把视觉二次修正量或接触事件误报为抓取成功。要完成图示的全流程，下一步需将 `aruco_sample` 改为动态模型（`<static>false</static>` 或删除该标签），在保持碰撞体和惯性参数的前提下重新启动 Gazebo，再以 `/gripper/contact/status=attached aruco_sample` 与抬升后工件随腕部运动作为抓取成功判据复测。

## 逐物体真值观察位复测（2026-08-07）

巡航已改为在每个工位逐一读取 Gazebo 中物体与机器人同帧真值，计算物体相对基座的距离、方位和高度，并分别调整肩部偏航、抬臂高度和腕部偏航；评测还增加“当前观察物体”门控，防止 ArUco 残留检测被计入其他物体。完整路线已返回 `home`。

| 标签 | 三维样本 | ≤0.15 m 比例 | 平均误差 | 结论 |
|---|---:|---:|---:|---|
| `aruco_sample` | 29 | 100.00% | 0.0185 m | 逐物体观察有效 |
| `aging_rack` | 5 | 0.00% | 1.199 m | 检出但三维变换仍有系统偏差 |
| `tooling_hand_tools` | 2 | 0.00% | 1.334 m | 检出但三维变换仍有系统偏差 |
| 其余五类 | 0 | 不可评估 | — | 当前观察位下二维漏检 |

总计 36 个有效三维样本，阈值内 29 个（80.56%）；该总体数值主要由 ArUco 贡献，不能解释为八类整体通过。全局 XYZ MAE 为 `(0.048, 0.218, 0.121) m`；失败类的主要偏差仍在 Y/Z。此次解决了固定姿态、非目标统计和跨工位延迟统计问题；剩余漏检需补充这五类在腕部相机俯视角下的训练数据，或改用 MoveIt 生成无碰撞的精确相机位姿。

## 最终复测：同坐标系与同工位时间戳门控（2026-08-06）

本节覆盖此前的跨工位统计结果。最终运行完成路线 `home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`；五个工位均实际到达并停留 5 秒，最终状态为 `DONE:home`。

### 本次修复的评测口径

- 相机 RGB-D 点经 TF 转至 `odom`；Gazebo `/gazebo/model_states` 的原始 `world` 真值，使用同一时刻机器人 `world ← base` 与 `odom ← base` 位姿计算 `odom ← world` 后再比较。
- 误差事件携带原始 RGB-D 图像时间戳；只接收“到达当前工位、静止 1 秒后拍摄”的同站画面。CPU YOLO 的延迟发布结果不会再被错算到下一站。
- 评测节点显式使用 Gazebo 仿真时钟。成功阈值仍为三维平移误差 `≤0.150 m`。

### 最终结果（C1，正常光照）

| 指标 | 结果 |
|---|---:|
| YOLO 正确工位候选帧 / 有效候选帧 | 11 / 30 |
| 二维粗检率 | 36.67% |
| 有效三维定位样本 | 11 |
| 阈值内样本 / 三维定位识别率 | 0 / 0.00% |
| 平均三维误差 | 1.357 m |
| XYZ 有符号均值（m） | `(-0.120, -1.173, -0.594)` |
| XYZ 绝对平均误差 MAE（m） | `(0.277, 1.173, 0.594)` |
| XYZ 最大绝对误差（m） | `(0.492, 1.320, 0.779)` |

| 八类训练标签 | 有效三维样本 | 三维定位识别率 | 平均误差 | XYZ MAE（m） |
|---|---:|---:|---:|---|
| `material_spare_igbt` | 0 | 不可评估（漏检） | — | — |
| `aruco_sample` | 0 | 不可评估（漏检） | — | — |
| `material_grease_can` | 0 | 不可评估（漏检） | — | — |
| `aging_rack` | 5 | 0.00% | 1.389 m | `(0.437, 1.116, 0.696)` |
| `board_test_fixture` | 0 | 不可评估（漏检） | — | — |
| `tooling_fixture_box` | 0 | 不可评估（漏检） | — | — |
| `tooling_hand_tools` | 6 | 0.00% | 1.331 m | `(0.144, 1.221, 0.509)` |
| `high_voltage_probe_kit` | 0 | 不可评估（漏检） | — | — |

### 结论

这次报告不再把不同坐标系、不同工位或移动中的画面混入误差。此前 9–11 m 的离群值已被定位为延迟消息跨工位统计，并由时间戳门控排除；最终最大轴误差已降到 1.320 m，但仍远高于 0.15 m 阈值，更不满足 1 mm 要求。

有效样本的主要偏差在 Y、Z 轴，说明下一步应校验腕部 `wrist_camera_optical_frame → base_link` 动态 TF 与实际腕部关节位姿，而不是继续调整 Gazebo `world/odom` 真值。六种光照/遮挡工况尚未逐一重跑；本节仅是修复后 C1 基线，不能外推为六工况结论。

更新日期：2026-07-31。本次仅修改和运行 `image_pkg`，以无 GUI 模式完成正常光、无局部遮挡（C1）的全工位巡航。

## 评测口径

本报告不再以物品所属工位或预测类别来选择真值。每个二维检测框经 RGB-D 点云得到空间位置，再转换到 Gazebo 的固定 `odom` 建图参考坐标系；与建图中**最近的实际目标模型**位置计算欧氏距离：

```text
translation_error = || detected_position_odom - nearest_mapped_object_odom ||2
成功：translation_error <= 0.15 m
```

无 GUI 启动没有发布 `map` TF；Gazebo 模型世界坐标与 `odom` 对齐，因此 `odom` 是本次可用且一致的建图参考系。该口径只评价三维平移定位，不将偶发类别错分或工位归属直接算成远处同类物体的数米误差。

当前 YOLO + RGB-D 流水线由二维框的点云质心估计三维位置；它没有从单个检测框恢复物体的真实旋转。因此“6D 位姿”中的平移分量已测得，旋转分量没有可靠观测，报告不会伪造旋转误差。

## 相机标定与本次修复

相机内参直接使用 Gazebo 发布的 `/bench_camera/camera_info`。外参使用 URDF 中 `base_link → camera_optical_frame` 的固定安装位姿；无 GUI 时若 TF 链暂不可用，使用 Gazebo `ModelStates` 的机器人位姿和相同 URDF 外参进行回退转换。

此前配置中的手工平移偏置 `[0.412, 0.128, -0.344]` 没有对应的可复现标定数据，会引入系统误差，现已清零。二维检测框与 RGB-D 点云改为按源图像时间戳缓存匹配，避免推理延迟造成机器人移动后框、深度和 TF 错配。

另外修复了二维检测线程中断问题：ArUco 后备检测框含有 NumPy `int64` 坐标，直接 JSON 序列化会抛出异常，导致 YOLO 后台线程停止发布。现在所有坐标、置信度和时间戳均转换为 JSON 原生类型，并将序列化置于异常保护内。

同时修复了 Gazebo 真值转换：目标坐标系从 `base_link` 改为固定建图参考系时，旧代码会访问空 TF；现已保证相机估计与 Gazebo 真值转换到同一坐标系后再比较。

## 标定后无 GUI C1 完整巡航结果

巡航路线：`home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`；每个非 home 工位停留 5 秒。最终状态为 `DONE:home`。

原始统计：[C1_urdf_calibrated_nogui/summary.json](lighting_benchmark_results/C1_urdf_calibrated_nogui/summary.json)。

| 指标 | 结果 |
|---|---:|
| 有效三维定位样本 | 774 |
| 最低有效样本要求 | 100（已满足） |
| 成功阈值 | 0.15 m |
| 阈值内样本 | 421 |
| 三维定位识别率 | 54.4% |
| 平均平移误差 | 1.040 m |

### 按二维预测标签汇总

此表仅用于定位问题诊断；真值配对不依赖该标签或工位。

| 预测标签 | 有效定位数 | 平均平移误差 |
|---|---:|---:|
| `aruco_sample` | 98 | 0.169 m |
| `igbt_module_plain` | 353 | 2.111 m |
| `thermal_grease_can` | 78 | 0.104 m |
| `fixture_box_plain` | 64 | 0.091 m |
| `tooling_hand_tools` | 24 | 0.122 m |
| `aging_rack` | 100 | 0.211 m |
| `pcb_test_fixture` | 42 | 0.034 m |
| `safety_probe_kit` | 15 | 0.270 m |

## 结论

标定后，定位识别率由之前的 0.0% 提升至 54.4%，证明三维误差确实主要受相机外参、手工偏置和 RGB-D 时间错配影响。`thermal_grease_can`、`fixture_box_plain`、`pcb_test_fixture` 已分别达到 93.6%、93.8%、97.6% 的 0.15 m 内定位率。

剩余瓶颈集中在 `igbt_module_plain`（19.5%，平均 2.111 m）和 `aging_rack`（56.0%）。这些类别仍需收紧二维框的前景深度分割、剔除桌面背景点，并补充对应仿真视角的训练样本；在此之前不应将 C2-C6 的结果用于光照和遮挡的最终横向结论。

## Gazebo 真值位姿基线

已撤销运行时的 ArUco IGBT 替换操作，并恢复原始无标记 IGBT 世界。以 Gazebo `/gazebo/model_states` 同一时刻、同一坐标系下的模型位姿直接作为“估计位姿”时：

| 指标 | Gazebo 真值基线 |
|---|---:|
| 平移定位误差 | 0.000 m |
| 旋转定位误差 | 0.000 rad |
| 1 mm 阈值内比例 | 100.0% |

该结果是仿真真值的自比较（oracle baseline），只用于验证误差计算、模型名称和坐标系转换是否一致；它不经过相机、YOLO、点云或 ArUco，不能作为视觉系统达到 1 mm 的证据。

## 两阶段统计运行结果

已运行粗检率与精定位误差分离的 C1 巡航，原始结果见 [C1_two_stage_metrics_nogui/summary.json](lighting_benchmark_results/C1_two_stage_metrics_nogui/summary.json)。

| 指标 | 结果 |
|---|---:|
| YOLO 非空候选帧 | 611 / 611 |
| YOLO 候选帧率 | 100.0% |
| 有效几何三维位姿 | 609 |
| 0.15 m 内精定位位姿 | 0 |
| 平均精定位误差 | 2.936 m |

该轮所有有效三维候选均被 YOLO 标记为 `igbt_module_plain`，故“100% 非空候选帧率”仅说明检测器持续输出候选框，**不代表正确粗检率**。后续报告应把粗检成功定义为“候选框与目标标记/建图投影的 IoU 达标”，而不是“任意非空框”。ArUco 精定位层尚未收到可见标记的候选，因而本轮没有可报告的 ArUco `solvePnP` 误差；不能据此声称达到 1 mm。

## TCP/工件精密定位口径与实际搬运测试（2026-08-03）

新增 `tcp_precision_benchmark`。该节点只在 `/gripper/attach/status` 发布 `attached`（即夹爪验证通过、工件实际附着）时取样：从 TF 读取 `gripper_tcp`，从 `/gazebo/model_states` 读取工件真值，在 TCP 局部坐标系计算工件相对抓取位姿误差。

```text
e_translation = || (p_object - p_tcp)_tcp - p_nominal_object_in_tcp ||
通过条件：e_translation <= 1.0 mm
```

这里的 Gazebo 位姿只作为工件参考真值；TCP 位姿来自机器人关节状态/TF。它评价的是“末端是否把工件抓到名义相对位姿”，不再将相机 RGB-D 的绝对定位误差混入该指标。仅在成功抓取后才生成统计 JSON；没有 `attached` 事件时，样本数为零，不能把零样本误报为零误差或 100% 通过。

已以无 GUI 原始世界启动，并发送标准指令 `把样件从A送到B`。机器人完成导航、地图精停和视觉停靠；抓取阶段的前三次已记录接触验证拒绝：

| 尝试 | TCP→工件偏差（m，TCP 局部） | 结果 |
|---|---|---|
| 1 | `(0.103, -0.012, -0.004)` | 拒绝 |
| 2 | `(0.101, 0.002, 0.023)` | 拒绝 |
| 3 | `(0.098, 0.003, 0.024)` | 拒绝 |

横向 `x` 偏差约 98–103 mm，超过抓取验证上限 40 mm。因此本次**没有成功附着事件、没有有效 TCP 精度样本，1 mm 通过率为“不可评估”而不是通过**。根因在执行链的视觉目标到 `base_link`/TCP 的交接仍存在约 0.10 m 系统偏移（并伴随局部 `z` 偏差），不是相机二维检测率或误差统计公式造成的。应先修正该坐标链、重做手眼外参并用独立靶标验证后，才可得到有意义的末端/工件毫米级精度结论。

## 相机→基座残差校准与复测（2026-08-03）

在不修改 `image_pkg` 以外代码的约束下，新增 `camera_base_residual_calibrator`，对 `/perception/target_pose` 的 RGB-D 空间点与 `/gazebo/model_states` 中的 `aruco_sample` 真值做 20 帧配对。得到可观测的平移残差（真值 − 估计，中值）：

```text
[+3.031, +0.143, -37.103] mm
```

该值已写入 `pose_translation_correction`。同时修复了执行坐标系交接：建图/评测仍在 `odom`，但 `/perception/target_pose` 在发布给任务节点前转换为 `base_link`；此前任务节点只接受 `base_link`，使已校正的 YOLO 位姿无法用于抓取。

重新以 `detection_source:=yolo` 执行 A→B 任务后，任务日志确认 `YOLO pose accepted at station-A safety line`，即已实际使用新坐标链。第一次抓取接触验证为：

| 阶段 | TCP→工件偏差（m，TCP 局部） | 结果 |
|---|---|---|
| 校准前典型值 | `(0.103, -0.012, -0.004)` | 拒绝 |
| 校准后复测 | `(0.044, 0.057, 0.008)` | 拒绝 |

X 偏差从约 103 mm 降至 44 mm，证明平移校准与 `odom → base_link` 交接修复确实降低了系统误差；但该帧的 YOLO 仍选到了错误的 `aruco_sample` 候选，Y 偏差达 57 mm（抓取上限 18 mm）。因此复测依然没有 `attached` 事件，**有效 TCP 精度样本为 0，不能报告 1 mm 通过率，更不能宣称已完成完整 6D 手眼标定。**

本轮仅能标定平移：YOLO 质心不提供工件朝向，固定相机也没有采集多组机器人末端姿态，故旋转外参不可观。若要证明 ≤1 mm，需要额外采集带方向的标定靶（如棋盘格/ArUco 板）在至少 10 组不同机械臂姿态下的相机观测与 TCP 真值，再以 hand-eye AX=XB 求解旋转和平移；并且需要提高 A 工位的正确候选选择率后才有有效的末端重复性样本。

## 9×7 棋盘 6D 固定相机外参标定（2026-08-03）

按 `generate_checkerboard.py 9 7 0.03` 在 `image_pkg/models/checkerboard_9_7_0_03` 生成 Gazebo 模型：9×7 方格、每格 30 mm，OpenCV 使用 8×6 内角点。棋盘置于相机光轴前并确认 `findChessboardCornersSB(8,6)` 命中；每帧由 `solvePnP` 得到棋盘→相机位姿，再用棋盘与 `base_link` 的 Gazebo 真值计算：

```text
T_base_camera = inv(T_world_base) · T_world_board · inv(T_camera_board)
```

黑白棋盘存在正反面及 180°角点排序歧义；标定节点已按实际可见面消歧，并将同一平面上的半周角点排序统一到首帧分支。最终 12 帧结果如下（原始 JSON：`~/.ros/eye_to_hand_checkerboard_20260803_141741.json`）：

| 指标 | 结果 |
|---|---:|
| 平移 `base_link → camera_optical_frame` | `[0.177922, -0.221675, 1.222081] m` |
| 姿态 RPY | `[-2.217963, 0.000781, -1.569308] rad` |
| 平移中值绝对偏差 | `[1.555, 1.867, 0.571] mm` |
| 最大旋转离散 | `0.778°` |
| 相对旧 URDF 外参的平移差 | 约 `4.0 mm` |

该 6D 外参已写入 `image_pkg/config/pose_estimation.yaml` 的非 GUI TF 回退参数。此结果完成了棋盘驱动的完整位姿外参求解，但其横向重复性仍约 1.6–1.9 mm，**尚不满足 ≤1 mm**；且 A 工位抓取仍受 YOLO 错误候选限制，未出现有效 `attached` TCP/工件样本。因此不能将本标定结果表述为末端/工件定位已达到 1 mm。

## 八种物品三维误差复测（2026-08-03，无 GUI）

在干净的原始 Gazebo 场景中，重新执行完整路线：`home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`；五个非 home 工位均实际到达并停留 5 秒，最终状态为 `DONE:home`。本轮采用当前的棋盘 6D 外参配置，评测以同一时刻的 Gazebo 建图真值为参考，误差为检测三维位置与最近真值物品位置的欧氏距离。

统计口径：二维粗检率是 YOLO 输出至少一个目标候选框的帧比例；三维“定位识别率”是有效三维样本中位置误差 `≤ 0.15 m` 的比例。它衡量的是三维定位是否落入工程阈值，不能等同于二维类别 AP，也不能说明 TCP 已达到 1 mm。

原始可复核结果：[C1_6d_checkerboard_8object_nogui/summary.json](lighting_benchmark_results/C1_6d_checkerboard_8object_nogui/summary.json)。

| 汇总指标 | 结果 |
|---|---:|
| YOLO 粗检候选帧 / 总帧 | 614 / 707 |
| YOLO 粗检率 | 86.85% |
| 有效三维评测样本 | 864 |
| 误差阈值 | 0.150 m |
| 阈值内样本 / 三维定位识别率 | 386 / 44.68% |
| 全部样本平均三维误差 | 1.124 m |

### 分物品误差结果

| 物品标签 | 有效样本 | 阈值内样本 | 三维定位识别率 | 平均误差 |
|---|---:|---:|---:|---:|
| `aruco_sample` | 111 | 95 | 85.59% | 0.255 m |
| `igbt_module_plain` | 415 | 71 | 17.11% | 2.073 m |
| `thermal_grease_can` | 76 | 73 | 96.05% | 0.052 m |
| `fixture_box_plain` | 41 | 36 | 87.80% | 0.117 m |
| `tooling_hand_tools` | 35 | 33 | 94.29% | 0.057 m |
| `aging_rack` | 133 | 50 | 37.59% | 0.436 m |
| `pcb_test_fixture` | 38 | 14 | 36.84% | 0.341 m |
| `safety_probe_kit` | 15 | 14 | 93.33% | 0.090 m |

本轮八类均有有效样本，因此不存在“未检出类别”被其他类别代替统计的情况。不过，`igbt_module_plain` 样本数异常高且平均误差 2.073 m，结合保存的失败帧可判断其仍会吸收远处或错误目标的候选；这也是全局均值和总体三维定位识别率偏差较大的主要来源。`aging_rack`、`pcb_test_fixture` 同样未达到 0.15 m 工程阈值。相反，`thermal_grease_can`、`tooling_hand_tools`、`safety_probe_kit` 的阈值内比例均超过 93%，但其最大误差仍说明偶发深度/真值关联异常没有完全消除。

因此，本次结果验证了“二维检测有较高候选输出”并不自动转化为稳定三维定位；在修正 IGBT 的类别/实例关联和二维框内前景深度分割前，不能以当前视觉链路声称满足项目的 `≤ 1 mm` 空间定位要求。

## 新 `best.pt` 建图类别映射与 XYZ 误差复测（2026-08-05）

已按新版训练类别表更新 `image_pkg` 映射，而不是沿用旧的简写类别：`material_spare_igbt`、`aruco_sample`、`material_grease_can`、`aging_rack`、`board_test fixture`、`tooling_fixture_box`、`tooling_hand_tools`、`high_voltage_probe_kit` 分别映射至下游的 IGBT、ArUco 样件、导热脂罐、老化架、PCB 夹具、工装盒、手工具盘和高压探针箱语义标签。

评测节点已升级为 schema v3：每个三维估计记录 `error_xyz = estimate - Gazebo_truth`，并输出三轴有符号均值（用来观察系统偏置）、三轴绝对平均误差 MAE（用来观察实际误差）和三轴最大绝对误差。原始结果：[C1_new_model_xyz_overlay/summary.json](lighting_benchmark_results/C1_new_model_xyz_overlay/summary.json)。

本轮依次到达 `station_a`、`inspection_zone`、`tooling_zone`、`aging_zone` 并各停留 5 秒；`station_b` 的导航于 48.5 秒后失败，未返回 home。因此它是**新模型和 XYZ 统计的有效运行验证，但不是八类完整覆盖测试**。下表中的 `—` 表示该预测标签没有有效三维样本，不能解释成零误差或检测成功。

| 汇总指标 | 结果 |
|---|---:|
| YOLO 非空候选帧 / 总帧 | 410 / 706（58.07%） |
| 有效三维样本 | 600 |
| 误差阈值 | 0.150 m |
| 阈值内样本 / 三维定位识别率 | 19 / 3.17% |
| 平均三维误差 | 2.216 m |
| XYZ 有符号平均误差（m） | `(+0.311, +1.348, +0.274)` |
| XYZ 绝对平均误差 MAE（m） | `(0.587, 1.955, 0.308)` |
| XYZ 最大绝对误差（m） | `(4.505, 4.235, 1.051)` |

### 分预测标签 XYZ 误差

“XYZ 均值”为有符号误差，正值表示估计点在对应轴正方向偏离真值；“XYZ MAE”为绝对误差平均值。

| 预测标签 | 样本 | 0.15 m 内比例 | 三维均值 | XYZ 均值（m） | XYZ MAE（m） | XYZ 最大绝对误差（m） |
|---|---:|---:|---:|---|---|---|
| `aruco_sample` | 0 | — | — | — | — | — |
| `igbt_module_plain` | 10 | 0.00% | 5.112 m | `(+4.321, -2.619, -0.778)` | `(4.321, 2.619, 0.778)` | `(4.505, 2.733, 0.780)` |
| `thermal_grease_can` | 0 | — | — | — | — | — |
| `fixture_box_plain` | 12 | 66.67% | 0.495 m | `(+0.120, -0.362, -0.112)` | `(0.122, 0.389, 0.112)` | `(1.329, 4.235, 0.131)` |
| `tooling_hand_tools` | 183 | 5.46% | 1.960 m | `(+0.380, +1.772, +0.304)` | `(0.393, 1.785, 0.314)` | `(1.801, 2.267, 0.341)` |
| `aging_rack` | 394 | 0.25% | 2.318 m | `(+0.184, +1.308, +0.299)` | `(0.597, 2.069, 0.300)` | `(2.671, 2.813, 1.051)` |
| `pcb_test_fixture` | 0 | — | — | — | — | — |
| `safety_probe_kit` | 1 | 0.00% | 0.708 m | `(-0.547, -0.448, -0.013)` | `(0.547, 0.448, 0.013)` | `(0.547, 0.448, 0.013)` |

从总体 MAE 可见，当前最大问题是 **Y 轴**（1.955 m），而不是相机几毫米级的像素尺度或手眼标定残差。并且 IGBT 的 X/Y 偏置达到数米、`aging_rack` 与 `tooling_hand_tools` 贡献了绝大多数样本，说明新权重在当前 Gazebo 视角下仍存在严重类别/实例混淆；这类错误不能靠三维坐标校正消除。应先核对训练图像是否与本项目的 Gazebo 模型材质、比例和视角一致，并在修正 `station_b` 导航后重新完成八类均有样本的巡航，再比较训练更新前后的真实改善幅度。

## 原始训练标签与重新 6D 标定（2026-08-05）

已移除旧语义别名。当前检测、三维真值配对与统计只使用新权重的八个原始类别：`material_spare_igbt`、`aruco_sample`、`material_grease_can`、`aging_rack`、`board_test fixture`、`tooling_fixture_box`、`tooling_hand_tools`、`high_voltage_probe_kit`。旧的 `igbt_module_plain`、`thermal_grease_can` 等名称不再参与当前检测和评测。

重新使用 9×7、30 mm 棋盘完成 20 帧固定相机 6D 外参标定。原始结果：[eye_to_hand_checkerboard_20260805_210345.json](calibration_results/eye_to_hand_checkerboard_20260805_210345.json)。

| 标定指标 | 结果 |
|---|---:|
| `base_link → camera_optical_frame` 平移 | `(0.180466, -0.219096, 1.225984) m` |
| RPY | `(-2.223343, -0.001905, -1.572639) rad` |
| 平移 MAD | `(0.000206, 0.000210, 0.0000002) mm` |
| 最大旋转离散 | `0.00264°` |

该外参已写入 `pose_estimation.yaml`。本轮最终误差巡航未生成可用八类结果：初始 `home` 零距离目标已在 `station_cruise` 中修复为直接到达，但 Nav2 随后立即拒绝 `station_a` 目标，巡航未离开 home。该故障发生在导航层，当前没有按五个工位覆盖的样本；因此不能以 home 处的少量检测伪造新的八类 XYZ 误差报告。待 `station_a` 导航动作恢复接受后，应以同一 `C1_raw_labels_6d_final` 口径重跑完整路线。

## 导航后备修复与原始标签最终复测（2026-08-05）

诊断显示 `/bt_navigator` 为 inactive、`/amcl` 为 unconfigured，且缺少 `map → base_link` TF，因此 Nav2 会立即拒绝目标。仅在 `image_pkg/station_cruise` 中增加了受限后备：Nav2 失败时以 `/odom` 与限速 `/cmd_vel` 到达同一航点；初始 home 零距离目标直接视为到达。完整路线已实际完成：`home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`，五个非 home 工位均停留 5 秒，最终状态 `DONE:home`。

本轮使用原始八类训练标签、2026-08-05 的 20 帧 6D 外参与 XYZ schema v3。原始统计：[C1_raw_labels_6d_navfixed/summary.json](lighting_benchmark_results/C1_raw_labels_6d_navfixed/summary.json)。

| 汇总指标 | 结果 |
|---|---:|
| YOLO 非空候选帧 / 总帧 | 182 / 682（26.69%） |
| 有效三维样本 | 185 |
| 阈值内样本 / 三维定位识别率（≤0.15 m） | 155 / 83.78% |
| 平均三维误差 | 0.463 m |
| XYZ 有符号均值（m） | `(-0.008, -0.013, +0.009)` |
| XYZ MAE（m） | `(0.358, 0.177, 0.074)` |
| XYZ 最大绝对误差（m） | `(4.606, 2.673, 1.163)` |

| 原始训练标签 | 样本 | 阈值内比例 | 平均三维误差 | XYZ 有符号均值（m） | XYZ MAE（m） |
|---|---:|---:|---:|---|---|
| `material_spare_igbt` | 14 | 14.29% | 2.051 m | `(-1.980, +0.362, -0.088)` | `(1.981, 0.362, 0.088)` |
| `aruco_sample` | 2 | 0.00% | 3.085 m | `(+2.471, -0.971, -0.004)` | `(2.471, 0.971, 0.272)` |
| `material_grease_can` | 141 | 98.58% | 0.081 m | `(+0.004, +0.059, -0.004)` | `(0.015, 0.073, 0.024)` |
| `aging_rack` | 15 | 6.67% | 2.584 m | `(+1.389, -0.929, +0.244)` | `(2.074, 1.014, 0.547)` |
| `board_test fixture` | 0 | — | — | — | — |
| `tooling_fixture_box` | 0 | — | — | — | — |
| `tooling_hand_tools` | 6 | 100.00% | 0.029 m | `(-0.003, -0.005, -0.023)` | `(0.009, 0.008, 0.023)` |
| `high_voltage_probe_kit` | 7 | 100.00% | 0.053 m | `(-0.007, -0.000, -0.017)` | `(0.045, 0.011, 0.017)` |

本次重新标定与导航修复使总体阈值内比例达到 83.78%，且整体 XYZ 偏置接近零；但它不能掩盖类别层问题。`material_spare_igbt`、`aging_rack`、`aruco_sample` 的米级误差是错误检测/错误实例关联造成的离群值，不能用外参再校正解决。`board_test fixture`、`tooling_fixture_box` 没有有效三维样本，明确记为未检出，不计作零误差或通过。

## 严格工位停稳、同标签与 ArUco PnP 复测（2026-08-05）

按建图与物品清单重新执行路线：`home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`。每站停留 8 秒；评测仅在收到 `ARRIVED:<station>` 后满 1 秒、`/odom` 速度不超过 0.02 m/s 时开启，并且只接受该工位所属的原始训练标签。真值配对改为同标签，`aruco_sample` 使用专用 ArUco `solvePnP` 位姿而非 YOLO 框的点云质心。完整巡航已返回 home。

原始结果：[C1_station_gated_label_pnp/summary.json](lighting_benchmark_results/C1_station_gated_label_pnp/summary.json)。

| 指标 | 结果 |
|---|---:|
| 停稳且在工位窗口内的 YOLO 帧 | 87 |
| 输出该工位预期标签的帧 | 0 |
| 有效三维误差样本 | 0 |
| XYZ 误差 / 识别率 | 不可评估 |

八个标签均为 0 个有效样本。该结果是严格门控后得到的真实失败，而不是“零误差”或“100% 通过”：模型在正确工位、机器人停稳时没有输出对应工位的类别。此前出现的 2–4 m 样本来自移动过程及跨工位候选，已被本次门控排除。因此当前应优先核对新 `best.pt` 的训练图像、标签名和 Gazebo 相机视角是否一致；在至少能在各工位稳定输出对应类别前，不能计算有意义的 XYZ 定位精度。

## 扩充数据集后的新 `best.pt` 工位门控复测（2026-08-06）

已使用用户更新后的 `image_pkg/models/best.pt`（SHA-256：`f22857a975133d8ee408aaa48f4fd9e35018c7fd6317be7ed5bc6310effb3179`）。加载模型实际导出的八个类别为：`material_spare_igbt`、`aruco_sample`、`material_grease_can`、`aging_rack`、`board_test_fixture`、`tooling_fixture_box`、`tooling_hand_tools`、`high_voltage_probe_kit`。其中新权重使用 `board_test_fixture`（下划线）；已在 `image_pkg` 配置、工位映射、Gazebo 真值标签中统一，未保留旧的带空格别名。

本次实际运行路径为 `home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`。每一非 home 工位到达后静止 8 秒，开始统计前另等待 1 秒并要求 `/odom` 速度不超过 0.02 m/s；仅接受该工位应出现的类别，且只与相同标签的 Gazebo 真值比较。Nav2 在当前仿真中不能实际执行目标，故巡航节点改为 3 秒后切换到 `/odom` 后备导航；后备导航保留独立的 120 秒上限。本轮日志已确认五站均到达并最终 `DONE:home`。

三维链路沿用已完成的 9×7、30 mm 棋盘 6D 手眼外参；本次只更换检测权重，不重新拟合外参。原始结果：[C1_new_best_station_gated/summary.json](lighting_benchmark_results/C1_new_best_station_gated/summary.json)。成功判据仍为位置误差 `≤ 0.150 m`；这不是项目的 1 mm 末端精定位指标。

| 汇总指标 | 结果 |
|---|---:|
| 静止工位窗口内 YOLO 非空候选帧 / 总帧 | 26 / 132（19.70%） |
| 有效同标签三维样本 | 52 |
| 阈值内样本 / 三维定位成功率 | 26 / 50.00% |
| 平均三维位置误差 | 0.407 m |
| XYZ 有符号平均误差（m） | `(-0.356, -0.040, -0.063)` |
| XYZ MAE（m） | `(0.361, 0.040, 0.063)` |
| XYZ 最大绝对误差（m） | `(0.716, 0.060, 0.093)` |

| 原始训练标签 | 有效样本 | ≤0.15 m 比例 | 平均三维误差 | XYZ 有符号均值（m） | XYZ MAE（m） |
|---|---:|---:|---:|---|---|
| `material_spare_igbt` | 26 | 100.00% | 0.095 m | `(+0.005, -0.021, -0.093)` | `(0.005, 0.021, 0.093)` |
| `material_grease_can` | 26 | 0.00% | 0.719 m | `(-0.716, -0.060, -0.033)` | `(0.716, 0.060, 0.033)` |
| `aruco_sample` | 0 | — | — | — | — |
| `aging_rack` | 0 | — | — | — | — |
| `board_test_fixture` | 0 | — | — | — | — |
| `tooling_fixture_box` | 0 | — | — | — | — |
| `tooling_hand_tools` | 0 | — | — | — | — |
| `high_voltage_probe_kit` | 0 | — | — | — | — |

结论：新权重已能在 A 工位静止期产生 IGBT 与导热脂罐的有效同类三维样本；IGBT 的误差为 95 mm，显著低于本次 150 mm 粗定位阈值，但仍远大于 1 mm。导热脂罐的主要问题是 X 轴约 -716 mm 的系统偏差，属于三维转换/深度前景选择或相机—真值坐标关联问题，不能由增加二维训练数据单独解决。其余六类在正确工位、静止窗口内没有产生可配对的三维样本，必须报告为“未评估”，不能写成零误差或通过率为零。下一步应采集这些工位的停稳 RGB-D 画面，分别确认是 YOLO 未输出该类、置信度低于阈值，还是框内有效深度不足；随后再针对该失败环节修复。

## 安全绕行与相机对准后的复测（2026-08-06）

`station_cruise` 已改为经地图外围/通道中转点绕开桌面和高压区围栏，并在每个最终观察位闭环转至 `yaw=π/2`（朝北、面向物品）。GUI 完整路线确认：`home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`，五个工位均停稳 8 秒并最终 `DONE:home`。原始结果：[C1_safe_corridor_camera_facing/summary.json](lighting_benchmark_results/C1_safe_corridor_camera_facing/summary.json)。

本轮停稳窗口内的 YOLO 候选帧率为 **75/123 = 60.98%**，高于此前直线巡航的 19.70%；`aruco_sample` 产生 55 个三维样本，说明相机对准显著改善了可见性。但三维真值统计出现 4–10 m 的共同 XY 偏移（总体 MAE：`(5.966, 6.486, 0.240) m`；平均误差 8.896 m），其量级接近场地尺寸，**本轮三维定位误差无效，不能作为模型性能下降或正式精度结果**。

该异常发生在 GUI 重启后的仿真会话，表明视觉输出所用 `odom`/相机变换与 Gazebo `world` 真值未在同一原点或方向；它不是 YOLO 2D 框、工位路径或相机朝向造成的随机定位误差。后续正式比较必须先发布并验证一致的 `world → odom` 变换，再重新运行同一安全路线；在此之前仅可采用本轮的 GUI 路径通过性与二维候选帧率结论，不能引用其米级 XYZ 数值。

## 腕部相机、6D 标定与五工位复测（2026-08-06）

本轮已改用末端腕部 RGB-D 相机：`/wrist_camera/image_raw`、`/wrist_camera/depth/image_raw`。已验证腕部相机光轴 `+Z` 与 `gripper_tcp +Z` 同向。利用 Gazebo 棋盘真值将水平 9×7、30 mm 棋盘置于腕部相机光轴正下方，成功完成 20 帧 6D 标定；`base_link → wrist_camera_optical_frame` 标定平移为 `(0.4935, 0.1179, 0.8029) m`，最大旋转离散 `0.062°`。

实际路线为 `home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home`。每个非 home 工位到达后先抬臂至腕部观察姿态、等待 3.2 秒，再停留 5 秒；只比较当前工位的预期标签和同标签 Gazebo 真值。全路径已返回 `DONE:home`。

| 汇总指标 | 结果 |
|---|---:|
| 停稳窗口候选帧 / 总帧 | 19 / 48（39.58%） |
| 有效三维样本 | 30 |
| ≤0.150 m 成功样本 / 成功率 | 12 / 40.00% |
| 平均三维误差 | 0.721 m |
| XYZ 有符号平均误差（m） | `(-0.033, -0.636, -0.320)` |
| XYZ MAE（m） | `(0.086, 0.636, 0.323)` |
| XYZ 最大绝对误差（m） | `(0.176, 1.215, 0.558)` |

| 物体训练标签 | 有效样本 | ≤0.15m 比例 | 平均三维误差 | XYZ 有符号均值（m） | XYZ MAE（m） |
|---|---:|---:|---:|---|---|
| `aruco_sample` | 12 | 100.00% | 0.006 m | `(-0.002, -0.005, +0.003)` | `(0.002, 0.005, 0.003)` |
| `aging_rack` | 10 | 0.00% | 1.100 m | `(-0.176, -0.931, -0.558)` | `(0.176, 0.931, 0.558)` |
| `tooling_hand_tools` | 8 | 0.00% | 1.320 m | `(+0.100, -1.215, -0.508)` | `(0.100, 1.215, 0.508)` |
| `material_spare_igbt` | 0 | 未评估 | — | — | — |
| `material_grease_can` | 0 | 未评估 | — | — | — |
| `tooling_fixture_box` | 0 | 未评估 | — | — | — |
| `board_test_fixture` | 0 | 未评估 | — | — | — |
| `high_voltage_probe_kit` | 0 | 未评估 | — | — | — |

结论：腕部棋盘标定和 ArUco 精定位有效，`aruco_sample` 平均误差为 **6.1 mm**，XYZ MAE 均在 **5 mm** 内；但仍不满足 1 mm 要求。老化架、手工具盘分别存在约 -0.93 m 和 -1.21 m 的 Y 轴偏差，说明当前统一腕部观察姿态并不能覆盖所有桌面/地面物体。其余五类在对应工位的 5 秒窗口没有形成同标签有效三维样本，明确标为“未评估”，不可解读为零误差或通过。后续应按各工位真值相对位置生成不同的末端观察位，而非复用同一抬臂关节姿态。
