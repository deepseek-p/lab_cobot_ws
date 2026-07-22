# Lab Cobot 仿真复跑日志

- 日期：2026-07-19
- 工作区：`/home/zww/projects/lab_cobot_ws`
- 任务：`把样件从A送到B`
- 结果：**DONE**

## 运行环境

- Windows 电源计划：高性能
- WSL2：12GB 内存、8 个处理器、4GB swap
- GPU：NVIDIA GeForce GTX 1650 Ti，驱动 576.83
- PyTorch CUDA：可用（先前已验证）
- 启动方式：`ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false`

仅关闭 Gazebo GUI 以避免图形渲染负载；YOLO、ArUco、Nav2、MoveIt 和接触抓取均保持启用。

## 关键执行记录

```text
[mission_node] 导航到 station_a 已满足任务交接条件
[mission_node] 地图精停完成 base=(1.940,0.601,86.6deg)
[mission_node] 视觉停靠完成 obj=(0.829,-0.006,0.640)
[mission_node] wrist_detect=hit
[pick_place_node] Pick start detected=(0.823,-0.005,0.631)
[pick_place_node] 夹爪触觉闭合已附着 aruco_sample
[pick_place_node] 夹爪 contact attach aruco_sample accepted
[mission_node] 导航到 station_b 已满足任务交接条件
[mission_node] 地图精停完成 base=(-1.940,0.430,96.1deg)
[mission_node] 放置停靠完成 base=(-1.994,0.588,90.5deg)
[pick_place_node] 夹爪 contact release accepted
[gzserver] lab_cobot_grasp_fix released aruco_sample
[mission_node] 导航到 home 已满足任务交接条件
[mission_node] 地图精停完成 base=(-0.032,-0.024,8.1deg)
[mission_node] 任务结束: DONE
```

## 运行中告警及处理结果

本次仍出现过两次 `Timed out waiting for motion execution to finish`。控制器随后均报告 `Goal reached`，任务内置重试/清理逻辑继续执行，最终完成抓取、放置和返航；终态为 `DONE`。

首次复跑成功后，直接再次下发任务会失败在视觉定位阶段：样件已经被放到 B 站，A 站不再有样件。这不是环境故障。第二次完整测试前已重启 Gazebo 世界，恢复初始样件位置。

## 失败原因诊断

### 直接触发条件：机械臂执行超时

原始失败日志反复出现以下顺序：

```text
[pick_place_node] Timed out waiting for motion execution to finish.
...（任务将该动作判为失败并进入重试或清理）...
[joint_trajectory_controller] Goal reached, success!
```

这说明轨迹已经被 MoveIt 成功发送并被 `joint_trajectory_controller` 接受，但 Gazebo 中的仿真推进过慢或出现长暂停；任务节点采用墙钟计时，45 秒到期时控制器仍未完成。因此它失败的原因不是“无解的运动规划”，而是**仿真执行耗时超过任务等待上限**。

### 根因：本机 WSL/Gazebo 的资源和调度抖动

初始环境具有以下事实：

- Windows 使用“平衡”电源计划；
- 16GB 主机内存下，未配置 WSL 资源上限，Ubuntu 实际仅获得约 7.7GiB；
- `/clock` 曾测到单次约 18 秒的间隔；
- 同时运行 Gazebo、深度/点云相机、ArUco、YOLO、Nav2 和 MoveIt 时，`gzserver` 是主要 CPU 负载。

这会让仿真时钟、TF 和传感器消息滞后，继而造成 `感知位姿已过期`、`TF 已过期` 以及机械臂墙钟超时。更换兼容的 PyTorch/CUDA 后，目标检测可以使用 GPU；但无界面 Gazebo 的物理与传感器计算主要仍受 CPU/WSL 调度影响。

### 不是主因、但会降低成功率的现象

- MoveIt 偶尔报告 `ur_forearm_link` 与 `ur_wrist_3_link` 的自碰撞；内置重试成功后可继续，因此不是本次 `DONE` 任务的终止原因。
- 感知/TF 过期警告是短暂的重定位问题；视觉重试成功后会继续。
- 重复下发任务前没有重启世界时，样件仍位于 B 站，A 站没有目标可检测；这是世界状态未重置，不是软件损坏。

## 解决办法

### 已执行且应保留的环境设置

1. Windows 电源计划保持“高性能”。
2. 保留 Windows 用户目录的 `.wslconfig`：

   ```ini
   [wsl2]
   memory=12GB
   processors=8
   swap=4GB
   ```

3. 修改 `.wslconfig` 后需执行一次 `wsl --shutdown`，再重新进入 WSL。
4. 保持 NVIDIA 驱动 576.83 和当前可用的 CUDA PyTorch 环境。

### 每次运行的无代码操作

1. 用无 GUI 模式启动，减少无关渲染负担：

   ```bash
   ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false
   ```

2. 关闭其他高负载 Windows/WSL 程序（浏览器视频、编译、其他仿真或模型推理）。
3. 每完成一次 A→B 任务后，先停止并重新启动 launch，再发送下一次任务；不要在同一世界中直接重复下发，因为样件不会自动回到 A 站。
4. 若只是验证机械臂/导航而非验证 YOLO，可使用已有运行参数 `use_dl_perception:=false` 降低负载；这不是代码修改。

### 仍偶发超时时的结论

本次完整复跑虽然有两次超时告警，但靠现有重试最终达到 `DONE`。若要让每一次都严格避免这类边界超时，唯一彻底的手段是让任务的动作等待上限随轨迹/仿真速度调整；那属于代码参数逻辑修改，本次没有执行。当前不改代码的可行方案是保留上述环境设置、无 GUI 启动、减少后台负载，并在每次任务前重置仿真世界。

## 原始运行日志

本次完整原始输出保存在当前会话临时文件：`/tmp/lab_cobot-rerun.log`。
