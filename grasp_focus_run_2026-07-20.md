# 抓取方向专项仿真记录（2026-07-20）

## 范围

只启动 Gazebo、ros2_control 与 MoveIt；不启动 Nav2、YOLO 或任务编排。抓取对象为 `aruco_sample`，默认尝试双指触觉门控。

## 已确认结果

- Gazebo 四个控制器可正常激活。
- MoveIt 接近轨迹可执行；此前下降段被 MoveIt 的 `allowed_start_tolerance=0.01` 拒绝，日志中的肘关节残差为约 `0.022 rad`。
- 已将该有界校验改为 `0.05 rad`，并新增配置回归测试；`lab_cobot_moveit` 测试 13/13 通过。
- 新增 `world.launch.py` 的 `robot_x`、`robot_y`、`robot_yaw` 参数，默认仍为原点；`lab_cobot_gazebo` 测试 115 项通过（13 skipped）。

## 当前阻塞

在台边直接生成机器人时，初始机械臂姿态会与工位台面产生动力学干涉，导致底盘/物块被弹飞。触觉插件因此返回远离封套的偏移，不能作为有效的抓取失败归因。

## 后续验证方式

从原点生成机器人，先保持机械臂在安全姿态，再经底盘控制移动至 A 台精确停靠位，随后执行抓取。若触觉接触链路仍不稳定，使用项目明确标注的 `use_tactile_grasp:=false require_finger_contact:=false` 几何封套回退路径完成演示，并单独标注其非触觉性质。

## 回退抓取验证结果

**成功。** 在机器人原点安全生成后，将 A 台和样件置于机械臂可达的台面位置，以 `use_tactile_grasp=false`、`require_finger_contact=false` 执行抓取。运行日志依次出现：

```text
夹爪 contact attach aruco_sample accepted
Pick complete
GRASP_RESULT=True
```

该结果验证了 MoveIt 接近/下降、夹爪闭合、几何封套判定、contact fixed-joint attach 与抬升这一完整抓取链路。它是非触觉回退路径，不能作为双指真实接触门控成功率的数据。

## 原始 ROS 日志

- `/home/zww/.ros/log/2026-07-20-09-42-59-741008-DESKTOP-MIE57FT-5164/launch.log`
- `/home/zww/.ros/log/2026-07-20-09-47-06-371689-DESKTOP-MIE57FT-6552/launch.log`
- `/home/zww/.ros/log/2026-07-20-09-54-36-783618-DESKTOP-MIE57FT-8415/launch.log`

## 固定场景修复（10:14）

此前在运行中移动 A/B 台或动态样件，会让物理状态与 MoveIt 规划场景不同步。现新增 `worlds/grasp_place.world`：A、B 与样件在 Gazebo 启动前即固定，禁止运行中重定位模型。`world.launch.py` 新增 `world` 参数以选择该场景。

验证：`lab_cobot_gazebo` 测试结果为 115 passed、13 skipped、0 failures。

## 固定双工位抓放验证（10:22）

执行场景：`grasp_place.world`；抓取路径为几何封套回退模式（`require_finger_contact=false`、`use_tactile_grasp=false`）。

```text
夹爪 contact attach aruco_sample accepted
Pick complete
夹爪 contact release accepted
Place complete
PICK_RESULT=True
PLACE_RESULT=True
```

释放后 Gazebo 读回样件位姿为 `(0.845721, 0.217984, 0.784847)`，B 台中心为 `(0.82, 0.20, 0)`；样件已稳定落在 B 台面区域。

原始 ROS 日志：

- `/home/zww/.ros/log/2026-07-20-10-22-03-135716-DESKTOP-MIE57FT-15852/launch.log`
- `/home/zww/.ros/log/2026-07-20-10-22-38-691930-DESKTOP-MIE57FT-16172/launch.log`

## 默认触觉抓放验证（10:25）

同一固定场景下启用 `require_finger_contact=true` 与 `use_tactile_grasp=true`。结果：

```text
夹爪触觉闭合已附着 aruco_sample
夹爪 contact attach aruco_sample accepted
Pick complete
夹爪 contact release accepted
Place complete
TACTILE_PICK=True
TACTILE_PLACE=True
```

原始 ROS 日志：

- `/home/zww/.ros/log/2026-07-20-10-25-51-929853-DESKTOP-MIE57FT-16966/launch.log`
- `/home/zww/.ros/log/2026-07-20-10-26-28-438138-DESKTOP-MIE57FT-17292/launch.log`

## 触觉独立复现 #2（10:29）

**未通过。** 首次触觉 attach 被拒绝，封套偏移为 `(-15.201, 2.940, 2.108)`；横向偏差仍在项目的自动重试范围附近，但退让后重试时样件发生更大漂移。因此当前触觉路径已有一次完整成功和一次独立失败，不能报告为连续稳定成功。

原始 ROS 日志：

- `/home/zww/.ros/log/2026-07-20-10-29-02-535029-DESKTOP-MIE57FT-17946/launch.log`
- `/home/zww/.ros/log/2026-07-20-10-29-36-957772-DESKTOP-MIE57FT-18285/launch.log`

## 默认台面碰撞场景验证（10:37）

本轮启用正式默认的 `use_planning_scene_obstacles=true`。首次 approach 的 action 短暂失败由内置重试恢复；随后成功完成触觉附着、持物碰撞盒注入、B 位 release 与持物碰撞盒移除：

```text
SCENE_TACTILE_PICK=True
SCENE_TACTILE_PLACE=True
```

这表明在规划场景包含台面和持物样件障碍时，默认触觉抓放链路可完整执行。

原始 ROS 日志：

- `/home/zww/.ros/log/2026-07-20-10-37-38-847841-DESKTOP-MIE57FT-19987/launch.log`
- `/home/zww/.ros/log/2026-07-20-10-38-14-154980-DESKTOP-MIE57FT-20320/launch.log`

## 本轮结论与收尾（10:40）

本轮从 A 位抓取并在 B 位释放的完整链路已返回成功：

```text
SCENE_TACTILE_PICK=True
SCENE_TACTILE_PLACE=True
```

仿真中的抓取插件也记录了样件附着到 `ur_wrist_3_link`、释放后恢复碰撞响应。验证后已通过 `SIGINT` 正常停止 Gazebo；MoveIt 在退出析构阶段仍打印已知的段错误回溯。这发生在成功结果输出之后，**不影响本轮抓取/放置判定**。

说明：`grasp_place.world` 是将 A、B 台面布置在机械臂可达范围内的抓取专项验证场景；本记录验证的是机械臂 A→B 抓放，不包含 Nav2 的跨区域底盘导航。

## 默认台面碰撞场景独立复现 #2（10:45）

重新启动 Gazebo 与 MoveIt 后，以相同的默认配置（`require_finger_contact=true`、`use_tactile_grasp=true`、`use_planning_scene_obstacles=true`）再次执行。全过程没有使用前一次进程或物体状态，结果为：

```text
SCENE_TACTILE_PICK=True
SCENE_TACTILE_PLACE=True
```

日志依次确认了触觉附着、持物碰撞盒加入、B 位触觉释放、持物碰撞盒移除和 `Place complete`。本次运行脚本以退出码 `0` 正常结束。

原始 ROS 日志：

- `/home/zww/.ros/log/2026-07-20-10-43-15-778240-DESKTOP-MIE57FT-21501/launch.log`
- `/home/zww/.ros/log/2026-07-20-10-43-52-803982-DESKTOP-MIE57FT-21829/launch.log`

## GUI 可视化复现（10:52）

以 `gui:=true` 启动 Gazebo，并重新执行完整 A→B 触觉抓放。结果：

```text
GUI_SCENE_TACTILE_PICK=True
GUI_SCENE_TACTILE_PLACE=True
```

执行结束后保留 Gazebo 运行，样件处于 B 台面位置，便于直接目视检查。运行脚本以退出码 `0` 正常结束。

原始 ROS 日志：

- `/home/zww/.ros/log/2026-07-20-10-50-15-887074-DESKTOP-MIE57FT-23278/launch.log`
- `/home/zww/.ros/log/2026-07-20-10-50-54-350113-DESKTOP-MIE57FT-23641/launch.log`
