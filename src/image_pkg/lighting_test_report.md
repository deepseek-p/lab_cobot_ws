# 全工位巡航六工况三维定位识别率报告

更新日期：2026-07-21。所有改动位于 `image_pkg`。本版不再按固定 100 帧计算识别率，而是以检测物体的三维 TF 坐标与 Gazebo 建图真值坐标的误差作为成功判据。

## 检测类别与工作区

只检测建图中带箭头标注的八个模型；区域名和 `home` 仅用于导航，不是识别类别。

| 工作区 | 模型标签 | YOLO-World 实际提示词 |
|---|---|---|
| `station_a` | `aruco_sample` | `aruco marker`（ArUco 后备检测） |
| `station_a` | `igbt_module_plain` | `IGBT module` |
| `station_a` | `thermal_grease_can` | `thermal grease can` |
| `tooling_zone` | `fixture_box_plain` | `fixture box` |
| `tooling_zone` | `tooling_hand_tools` | `hand tool tray` |
| `aging_zone` | `aging_rack` | `aging rack` |
| `station_b` | `pcb_test_fixture` | `PCB test fixture` |
| `inspection_zone` | `safety_probe_kit` | `high voltage probe kit` |

YOLO-World 使用右栏自然语言提示词推理，检测结果在 `image_pkg` 内映射回中栏模型标签；这样既保持开放词表识别能力，也能与 Gazebo 的实际建图物体逐一匹配。

## 统一全工位巡航命令

每种工况从 `home` 出发，沿以下路径巡航后回到 `home`：

```text
home → station_a → inspection_zone → tooling_zone → aging_zone → station_b → home
```

```bash
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '巡航所有工位'}"
```

`inspection_zone` 为无桌、高压围栏区域，因此仅评估其中的 `safety_probe_kit`；不将围栏或区域本身当作物体检测目标。

## 三维误差成功判据

对每条由 RGB-D 点云生成的物体 TF，`image_pkg` 将其与同一物体的 Gazebo `ModelStates` 真值都转换到 `base_link`，计算：

```text
position_error = || detected_position_base_link - truth_position_base_link ||₂
```

默认阈值为 `0.15 m`，可通过 `lighting_benchmark` 的 `position_error_threshold_m` 参数调整。

```text
识别率 = position_error ≤ 0.15 m 的有效三维定位次数 / 含 Gazebo 真值的有效三维定位次数
```

这项指标评价“识别到的物体是否被正确定位”。完全漏检没有三维 TF，因而不会产生误差事件；报告必须同时查看每类有效定位次数和保存的失败图，不能把“没有数据”解释为高识别率。

## 六种工况

| 工况 | 光照 | 遮挡 | Gazebo 参数 |
|---|---|---|---|
| C1 | 正常光 | 无遮挡 | `lighting_profile:=normal enable_actor:=false` |
| C2 | 正常光 | 局部遮挡 | `lighting_profile:=normal enable_actor:=true` |
| C3 | 弱光 | 无遮挡 | `lighting_profile:=dark enable_actor:=false` |
| C4 | 弱光 | 局部遮挡 | `lighting_profile:=dark enable_actor:=true` |
| C5 | 强反射 | 无遮挡 | `lighting_profile:=reflective enable_actor:=false` |
| C6 | 强反射 | 局部遮挡 | `lighting_profile:=reflective enable_actor:=true` |

每次巡航开始前启动 `image_pkg` 的 YOLO 与三维定位节点，再启动 `lighting_benchmark`。完整巡航回到 `home` 后停止 benchmark；它会自动写出该工况的 `summary.json`、位置误差失败截图及汇总表。

以 C1 为例，启动 benchmark 后发送巡航命令；任务返回 `home` 后按 `Ctrl-C` 结束 benchmark：

```bash
ros2 run image_pkg lighting_benchmark --ros-args \
  -p condition:=C1_normal_visible \
  -p position_error_threshold_m:=0.15

ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '巡航所有工位'}"
```

同一张 YOLO 检测框即使与多个后续点云配对，也只计一次三维误差，避免点云频率人为放大识别率样本数。

## 六工况结果矩阵

新指标尚未完成六次实际巡航，不能填写虚假百分比。实际运行后由 `image_pkg/lighting_benchmark_results/lighting_benchmark_report.md` 自动汇总。

| 工况 | 有效三维定位数 | 阈值 | 识别率 | 平均位置误差 | 结论 |
|---|---:|---:|---:|---:|---|
| C1 正常光 / 无遮挡 | 待采集 | 0.15 m | — | — | 待巡航 |
| C2 正常光 / 局部遮挡 | 待采集 | 0.15 m | — | — | 待巡航 |
| C3 弱光 / 无遮挡 | 待采集 | 0.15 m | — | — | 待巡航 |
| C4 弱光 / 局部遮挡 | 待采集 | 0.15 m | — | — | 待巡航 |
| C5 强反射 / 无遮挡 | 待采集 | 0.15 m | — | — | 待巡航 |
| C6 强反射 / 局部遮挡 | 待采集 | 0.15 m | — | — | 待巡航 |

## 结果分析方法

1. 比较 C1、C3、C5 的识别率和平均位置误差，量化光照变化对三维定位的影响。
2. 比较 `C2-C1`、`C4-C3`、`C6-C5`，量化局部遮挡造成的定位成功率下降与误差增大。
3. 某类模型的有效定位数为零时，结论应为“未获得可评测检测”，而不是“识别率 0%”或“通过”。先检查该工作区停靠视角和检测提示词。
4. 若有有效定位但误差超过 0.15 m，优先检查检测框是否覆盖物体、RGB 图像与点云是否注册、以及 TF/Gazebo 真值是否处于同一时刻和同一 `base_link` 坐标系。

此前的固定帧 0% 记录使用旧关键词且目标未完整入镜，已不适用本报告，不能参与六工况比较。
