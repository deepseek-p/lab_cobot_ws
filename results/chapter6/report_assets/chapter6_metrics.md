# Chapter 6 Metrics Summary

## Contact-Feedback Baseline

| 对象 | 实验次数 | 成功次数 | 失败次数 | 抓取成功率 |
|---|---:|---:|---:|---:|
| 高压探针工具 | 10 | 10 | 0 | 100.00% |
| 工装夹具 | 10 | 10 | 0 | 100.00% |
| IGBT 模块 | 10 | 8 | 2 | 80.00% |
| 总体 | 30 | 28 | 2 | 93.33% |

总体抓取成功率 = 28 / 30 * 100% = 93.33%，满足复杂物体抓取成功率不低于 90% 的指标要求。

## Lift Completion

| 指标 | 次数 | 比例 | 说明 |
|---|---:|---:|---|
| Primary lift success | 20/30 | 66.67% | Cartesian lift trajectory primary execution success |
| State-verified completion | 8/30 | 26.67% | controller abort 后依据 TCP 到位和 holding 状态判定任务级完成 |
| Final task completion | 28/30 | 93.33% | 最终完整抓取任务成功 |

部分 Cartesian lift trajectory 在执行末端返回 MoveIt error_code=-4 / GOAL_TOLERANCE_VIOLATED；该类样本不计为 primary trajectory execution success。若实际 TCP 已进入 lift final target tolerance 且 holding 状态保持健康，则记录为 STATE_VERIFIED_TASK_COMPLETION。

## Force-Control Prototype

| 对象 | N | 力反馈参与决策 | 主动控制调整 | FORCE_HOLD | 完整任务成功 |
|---|---:|---:|---:|---:|---:|
| 高压探针工具 | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| 工装夹具 | 3 | 3/3 | 0/3 | 3/3 | 3/3 |
| IGBT 模块 | 3 | 3/3 | 3/3 | 3/3 | 3/3 |
| 总体 | 9 | 9/9 | 6/9 | 9/9 | 9/9 |

Force source 为 VIRTUAL_ESTIMATE。主动控制调整定义为 force feedback -> force_error -> non-zero delta_q -> gripper command changed。Fixture 的 0/3 表示其三次进入 FORCE_REGULATION 时虚拟力已达到保持条件，控制器选择 delta_q=0 停止继续闭合。
