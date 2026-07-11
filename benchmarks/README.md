# benchmarks/ — 性能指标测量脚本（手动执行，不进 colcon test）

> 对标赛题评分"性能指标达成度（30 分）"。全部脚本遵守口径纪律：
> **报告均值/std/P95/最大值，拒绝单帧最优值话术**；测量边界如实标注。

| 脚本 | 对标指标 | 前置 | 产出 |
|---|---|---|---|
| `llm_plan_success_rate.py` | 任务分解与规划成功率 ≥95% | `export LLM_API_KEY=...` + `source install/setup.bash` | 三层判据（exact/semantic/defense）成功率 + LLM 延迟 P95 + markdown 报表 |
| `e2e_repeat_stats.sh` | 复杂物体抓取成功率 ≥90%（任务级口径） | `source install/setup.bash`；每轮自动清理 gzserver | 逐轮结果/耗时 + 成功率汇总（`-n` 指定轮数，单轮 ~2-8 min） |
| `perception_error_probe.py` | 目标空间定位误差 | 先起栈（`launch_mission:=false`），marker 需在视野内（建议 mission 停靠 A 工位后采样） | 3D 误差均值/std/P95/max + 分轴误差 |

结果统一落盘 `benchmarks/results/`（已 gitignore，报表按时间戳命名）。

## 待补脚本（随能力落地）

- 动态避障响应延时（≤200ms 指标）：测量方案设计中——注入动态障碍 → 计时 costmap 更新到 cmd_vel 响应
- 识别准确率矩阵（≥98% 指标）：依赖 Goal-P 落地后按 3.2 节工况矩阵批量采样
- 末端重复定位精度（±0.05mm 指标）：MoveIt 重复到位采样；实物级指标，仿真口径须在报告 5.7 声明
