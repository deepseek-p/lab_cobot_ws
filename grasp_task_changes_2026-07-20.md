# 抓取方向：截至 2026-07-20 的修改清单

本文件记录本轮为跑通机械臂抓取与 A→B 放置而实际新增或修改的文件。内容以当前 Git 工作区差异为准；未改动 `.vscode/` 以及三份既有的 `simulation_*.md`，它们不属于本轮代码修改范围。

## 修改的代码与配置

| 文件 | 修改内容 | 解决的问题 |
|---|---|---|
| `src/lab_cobot_manipulation/lab_cobot_manipulation/pick_place_node.py` | 将 `DEFAULT_MOVE_TIMEOUT_SEC` 从 `45.0` 增至 `120.0` 秒，并补充原因注释。 | Gazebo 在低实时因子、MoveIt 与其他组件同时运行时，正常控制器轨迹可能超过 45 秒墙钟；旧超时会提前取消 action，导致重试与旧目标竞争。 |
| `src/lab_cobot_moveit/config/moveit_controllers.yaml` | 增加 `trajectory_execution.allowed_start_tolerance: 0.05`。 | 轨迹成功后，Gazebo 关节偶有约 `0.02 rad` 的短暂收敛残差；允许该有界残差，避免紧接的笛卡尔下降被 MoveIt 拒绝为非法起点。 |
| `src/lab_cobot_gazebo/launch/world.launch.py` | 将 world 文件、机器人初始 `x/y/yaw` 参数化；把参数传给 Gazebo 的实体生成器。默认值保持原有 `lab.world`、原点和零偏航。 | 可启动专用抓放场景以及指定机器人初始位姿，不改变原默认启动行为。 |

## 新增的专项场景

| 文件 | 内容 | 用途 |
|---|---|---|
| `src/lab_cobot_gazebo/worlds/grasp_place.world` | 新建固定的 A、B 双工位场景：A 台中心 `(0.80, 0.00)`、B 台中心 `(0.82, 0.20)`、样件初始位于 A 台上方 `z=0.785`；台面为静态模型。将原本会重叠的 `0.8×0.6m` 台面缩为相互分离的 `0.45×0.16m` 台面，并添加白色（A）/黄色（B）顶面识别条、深色背景、环境光、主/补光和高饱和度橙色 A 台/蓝色 B 台材质。 | 提供稳定、机械臂可达且便于目视检查的抓取专项验证场景，避免运行中移动样件或台面造成的物理漂移。该场景验证机械臂 A→B 抓放，不含底盘 Nav2 跨区域导航。 |

## 同步修改的测试

| 文件 | 修改内容 | 目的 |
|---|---|---|
| `src/lab_cobot_manipulation/test/test_pick_place_sequence.py` | 增加断言：默认 MoveIt 等待时间不少于 `120.0` 秒。 | 防止超时修复被未来改回较短值。 |
| `src/lab_cobot_moveit/test/test_moveit_config.py` | 新增测试，验证 `allowed_start_tolerance` 为 `0.05`。 | 防止 MoveIt 起点容差配置被意外删除或改坏。 |

## 运行记录文档

| 文件 | 内容 |
|---|---|
| `grasp_focus_run_2026-07-20.md` | 记录每轮抓取/放置运行、原始 ROS 日志目录、失败现象和两次独立成功复现结果。 |
| `grasp_task_changes_2026-07-20.md`（本文件） | 汇总本轮文件修改及其原因。 |

## 验证状态

- `lab_cobot_gazebo`：115 项测试，0 errors、0 failures、13 skipped。
- `lab_cobot_moveit`：13 项测试全部通过。
- `lab_cobot_manipulation`：95 项通过、1 项 skipped、0 failures。
- 固定双工位场景在 `require_finger_contact=true`、`use_tactile_grasp=true`、`use_planning_scene_obstacles=true` 下，两次独立运行均返回：

  ```text
  SCENE_TACTILE_PICK=True
  SCENE_TACTILE_PLACE=True
  ```

## 未解决但已隔离的现象

在 ROS 2 Humble / MoveIt 进程收到 `SIGINT` 退出时，`move_group` 仍会在析构阶段打印段错误回溯。该现象出现在抓放结果完成之后；不影响上述执行结果，且本轮没有为掩盖该退出问题而修改业务代码。

## 抓放专项运行与完整仿真的区别

抓放专项运行只验证“机械臂抓取并放置”这一方向；完整仿真验证移动机器人从任务触发到跨工位放置完成的端到端闭环。

| 维度 | 抓放专项运行 | 完整仿真运行 |
|---|---|---|
| 启动内容 | Gazebo、机器人控制器、MoveIt、抓放节点。 | Gazebo、MoveIt、Nav2、定位/TF、感知、任务状态机，以及按配置启用的其他组件。 |
| 场景 | `grasp_place.world`：A/B 台面位于机械臂可达的小范围内。 | 正式 `lab.world`：A、B 是分离工位，需要底盘跨区域移动。 |
| 目标坐标来源 | 测试脚本直接提供样件抓取坐标和 B 位放置坐标。 | 感知链路检测目标，任务状态机将结果交给抓取流程。 |
| 底盘导航 | 不参与；机器人保持固定。 | 需导航至 A、精停、撤离、导航至 B、精停。 |
| 验证范围 | MoveIt 规划、夹爪触觉附着、持物避障、B 位释放。 | 任务编排、感知、导航、TF/定位、工位交接、抓取、运输、放置的完整闭环。 |
| 当前结论 | A 抓取 → B 安全释放已独立成功复现。 | 不能仅凭专项成功判定端到端任务已经成功；仍需全栈启动和完整任务验证。 |

两种运行复用同一机械臂、MoveIt、夹爪与抓放逻辑。因此专项成功可以证明抓取方向的核心能力已跑通，但不能覆盖完整仿真中可能出现的 Nav2 生命周期、`map/odom/base_link` TF、底盘精停、感知延迟及资源负载问题。

完整仿真入口为：

```bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py
```

## 抓取方向成果如何合入完整仿真

可以合入，但不能把专项场景的成功结果直接等同为完整仿真通过。应合并的是抓放模块的通用代码、配置和测试；专项场景本身仅用于隔离验证。

### 可合入完整仿真的内容

- `pick_place_node.py` 中的 MoveIt 等待超时修复；
- `moveit_controllers.yaml` 中的 Gazebo 关节收敛容差；
- 抓取、触觉附着、放置与持物避障逻辑；
- 对应的单元测试和配置测试。

完整仿真的 `mission_node` 会复用同一抓放节点、MoveIt 配置与夹爪逻辑，因此这些修改可直接参与全栈运行。

### 不应替换到完整仿真的内容

不应将 `grasp_place.world` 替代正式的 `lab.world`。前者是让 A/B 台面均落在固定机械臂可达范围内的抓放专项场景；正式场景仍须包含分离工位、底盘导航空间与真实任务链路。

### 合并后的验证顺序

1. 保留抓放专项测试，作为抓取模块回归测试。
2. 将上述通用抓放代码、配置和测试合入主分支。
3. 使用正式 `lab.world` 启动全栈：

   ```bash
   ros2 launch lab_cobot_bringup lab_cobot.launch.py
   ```

4. 验证端到端序列：导航至 A → 感知目标 → 抓取 → 撤离 → 导航至 B → 放置。

专项成功意味着抓取模块已具备集成条件，并能缩小后续排查范围；全栈仍需单独验证导航、TF、感知、精停交接与系统资源负载。

## Ubuntu：启动抓放专项仿真

修改源码后，先在任意终端构建一次：

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lab_cobot_gazebo lab_cobot_moveit lab_cobot_manipulation
```

随后打开三个终端，按以下顺序执行。

### 终端 1：Gazebo 专项场景

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lab_cobot_gazebo world.launch.py \
  gui:=true \
  world:=$(ros2 pkg prefix lab_cobot_gazebo)/share/lab_cobot_gazebo/worlds/grasp_place.world \
  require_finger_contact:=true \
  use_refine_detect:=false \
  use_wrist_detect:=false
```

若不需要画面或机器没有桌面环境，将 `gui:=true` 改为 `gui:=false`。

### 终端 2：MoveIt

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 launch lab_cobot_moveit move_group.launch.py use_sim_time:=true
```

等待终端输出 `You can start planning now!`。

### 终端 3：执行 A 抓取 → B 放置

```bash
cd ~/projects/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

python3 - <<'PY'
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from lab_cobot_manipulation.pick_place_node import PickPlace

rclpy.init()
node = PickPlace(
    target_object='aruco_sample',
    use_tactile_grasp=True,
    use_planning_scene_obstacles=True,
)
node.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(node)
threading.Thread(target=executor.spin, daemon=True).start()

pick = node.pick([0.80, 0.00, 0.6299])
place = node.place([0.82, 0.20, 0.725]) if pick else False

print(f'SCENE_TACTILE_PICK={pick}')
print(f'SCENE_TACTILE_PLACE={place}')

executor.shutdown()
node.destroy_node()
rclpy.shutdown()
PY
```

成功判据：

```text
SCENE_TACTILE_PICK=True
SCENE_TACTILE_PLACE=True
```

脚本结束后保留终端 1，即可在 Gazebo 中观察物品是否留在 B 台面。需要停止时，依次在终端 2 和终端 1 按 `Ctrl+C`。
