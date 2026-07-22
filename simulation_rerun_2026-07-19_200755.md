# 仿真复运行日志（2026-07-19 20:07）

## 执行命令

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false use_dl_perception:=false
```

## 结果

**未成功。** 本次在任务指令发布前即因导航栈初始化不稳定而停止，未出现 `任务结束: DONE`。

## 关键日志

- 四个 ros2_control 控制器均已正常激活。
- `lifecycle_manager_localization` 多次报告：`Have not received a heartbeat from map_server`，并重置 `map_server` / `amcl`。
- `planner_server` 持续报告 TF 时间外推错误：请求时间早于 `base_footprint -> map` 可用数据。
- 因导航栈未保持 active，未发布 `/task/instruction`，故本次不能作为取放任务的成功验证。

## 环境核查

- WSL：8 个逻辑 CPU；内存总量 11 GiB、可用约 8.4 GiB；Swap 4 GiB。
- 停止后未发现残留 ROS/Gazebo 进程。
- 当前工作区中为解决之前运动等待超时而调整的 `DEFAULT_MOVE_TIMEOUT_SEC=120.0` 已通过 `lab_cobot_manipulation` 测试：95 passed、1 skipped、0 failures。

## 原始 ROS 日志

`/home/zww/.ros/log/2026-07-19-20-07-55-645123-DESKTOP-MIE57FT-11704/launch.log`
