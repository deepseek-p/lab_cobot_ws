# image_pkg：自训练 YOLO 与 RGB-D 三维定位

`image_pkg` 是本工作空间的视觉感知包。它使用项目自训练的 17 类 YOLO 模型取得二维检测框，将检测框与逐像素对齐的 RGB-D 点云关联，并通过 TF 把目标位置转换到 `world` 或 `base_link`，供巡航、主动观察和抓取模块使用。

## 主要功能

- 使用 `models/best.pt` 检测 `config/data.yaml` 定义的 17 类物体。
- 由 RGB、深度图和相机内参生成有组织彩色点云。
- 在检测框内过滤无效深度、背景和离群点，计算目标三维位置。
- 支持广角相机粗定位与腕部相机近距离精定位；最终抓取位置以腕部相机为准，不对两路坐标做简单平均。
- 支持 Station A 五个带标记抓取方块的 ArUco 精定位。
- 发布 `vision_msgs/Detection3DArray`、`PoseStamped` 及 JSON 诊断话题。
- 巡航时支持底盘到站、机械臂主动观察、目标居中确认和逐物体处理。
- 可选设备指示灯颜色与状态识别。

无标记物体默认使用 YOLO + RGB-D 质心定位。此时位置由点云计算，姿态方向为单位四元数，不代表已经估计物体朝向。带标记方块使用 ArUco 角点、相机内参、标记几何关系和 TF 链恢复模型原点。

## 17 个检测类别

模型类别名保持与训练数据及 Gazebo 实体名一致：

```text
aruco_sample
material_cube_red
material_cube_green
material_cube_blue
material_cube_yellow
aging_rack
pcb_board
test_tube_rack
test_tube
beaker
erlenmeyer_flask
graduated_cylinder
board_test_fixture
tooling_fixture_box
tooling_hand_tools
high_voltage_probe_kit
material_spare_igbt
```

部署节点不会使用旧类别别名，也不会按颜色或场景位置伪造 YOLO 检测。

## 数据流

```text
/bench_camera/*
    -> 广角 YOLO + RGB-D
    -> /yolo/bench/poses
    -> dual_camera_fusion_node（只提供观察引导）

/wrist_camera/*
    -> 腕部 YOLO + RGB-D / ArUco
    -> /yolo/poses
    -> /perception/dual_camera/confirmed_poses
    -> /perception/target_pose
```

巡航节点先利用工位和目标场景位姿生成底盘观察位，再通过 MoveIt 调整机械臂。只有底盘停止、关节稳定、目标框进入中央安全区域后，才进入精细观察阶段并允许下游使用定位结果。

## 目录说明

| 路径 | 内容 |
| --- | --- |
| `models/best.pt` | 项目自训练的 17 类 YOLO 权重。 |
| `config/data.yaml` | 训练类别和数据集配置参考。 |
| `config/pose_estimation.yaml` | 双相机、点云、定位和融合节点参数。 |
| `config/camera_visualization.rviz` | 相机与点云 RViz 显示配置。 |
| `launch/pose_estimation.launch.py` | 视觉链路启动文件。 |
| `image_pkg/yolo_world_detector.py` | YOLO 推理封装。 |
| `image_pkg/yolo_world_node.py` | ROS 2 二维检测节点。 |
| `image_pkg/rgbd_pointcloud_node.py` | RGB-D 有组织点云生成节点。 |
| `image_pkg/yolo_pointcloud_pose_node.py` | YOLO 检测框与点云关联的三维定位节点。 |
| `image_pkg/dual_camera_fusion_node.py` | 广角提示与腕部结果的双相机关联节点。 |
| `image_pkg/station_cruise_node.py` | 工位巡航、主动观察和到站确认节点。 |
| `image_pkg/truth_guided_wrist_observer.py` | 根据场景目标位姿生成腕部观察姿态。 |
| `image_pkg/indicator_state_node.py` | 指示灯颜色与状态识别节点。 |
| `CMakeLists.txt`、`package.xml` | ROS 2 构建与依赖声明。 |
| `setup.py`、`setup.cfg` | Python 包及可执行入口配置。 |

## 构建

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select image_pkg
source install/setup.bash
```

运行依赖包括 ROS 2 Humble、Gazebo、MoveIt 2、`cv_bridge`、`sensor_msgs_py`、`vision_msgs`、TF2、NumPy、OpenCV、PyTorch 和 Ultralytics YOLO。

## 启动与执行 Station A 视觉抓取

推荐使用 `lab_cobot_bringup` 的一体化启动文件。它会依次启动 Gazebo、控制器、MoveIt、Nav2、`image_pkg` 视觉链路和任务编排节点，避免分别启动时重复生成机器人或遗漏 TF。以下命令均在不同终端中执行；每个新终端都要先加载 ROS 2 和工作空间环境。

### 1. 构建并加载工作空间

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to image_pkg lab_cobot_bringup
source install/setup.bash
```

### 2. 启动仿真、导航、MoveIt 和 17 类视觉识别

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  gui:=true \
  use_rviz:=true \
  launch_navigation:=true \
  launch_moveit:=true \
  launch_perception:=true \
  use_dl_perception:=true \
  vision_backend:=eight_class \
  target_object:=aruco_sample \
  use_truth_pose:=false \
  use_refine_detect:=true \
  use_wrist_detect:=true \
  skip_visual_dock:=false \
  launch_g4g5_results:=false
```

`vision_backend:=eight_class` 是主启动文件保留的兼容参数名；当前实际加载的是 `image_pkg/models/best.pt` 对应的 17 类模型。`use_truth_pose:=false` 表示抓取位姿来自相机识别，而不是直接使用 Gazebo 真值。视觉节点会在整套系统启动约 30 秒后加入，任务节点约 60 秒后就绪。

不要在运行上述一体化命令时再次单独启动 `world.launch.py` 或 `pose_estimation.launch.py`，否则会产生重复节点、重复机器人或话题冲突。

### 3. 打开和检查视觉识别界面

一体化启动会打开导航 RViz 和视觉 RViz。也可以在新终端打开图像查看器：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run rqt_image_view rqt_image_view
```

在界面中选择 `/yolo/annotated_image` 查看腕部相机检测框，或选择 `/yolo/bench/annotated_image` 查看广角相机检测框。发送任务前可用以下命令确认相机、检测和任务节点已经就绪：

```bash
ros2 topic hz /wrist_camera/image_raw
ros2 topic echo --once /yolo/detections
ros2 topic echo --once /task/status
```

当 `/task/status` 输出 `IDLE` 后再发送抓取任务。

### 4. 使用视觉识别执行 Station A 样件抓取

在新终端执行：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '把样件从A送到B'}"
```

该指令执行完整闭环：导航到 `station_a`、视觉停靠、使用腕部相机识别并精定位 `aruco_sample`、MoveIt 抓取、导航到 `station_b` 放置，最后返回 `home`。任务状态可在另一个终端持续查看：

```bash
ros2 topic echo /task/status
```

仅检查 A 工位而不抓取时使用：

```bash
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '去A工位检查一下样件然后回来'}"
```

### 5. 六工况运行流程

六工况由三种光照与两种遮挡状态组合：

| 工况 | `lighting_profile` | `enable_actor` | 含义 |
| --- | --- | --- | --- |
| C1 | `normal` | `false` | 正常光照、无遮挡 |
| C2 | `normal` | `true` | 正常光照、动态遮挡 |
| C3 | `dark` | `false` | 弱光、无遮挡 |
| C4 | `dark` | `true` | 弱光、动态遮挡 |
| C5 | `reflective` | `false` | 强反射、无遮挡 |
| C6 | `reflective` | `true` | 强反射、动态遮挡 |

每个工况按照以下顺序独立运行：

1. 结束上一工况，在启动仿真的终端按 `Ctrl+C`，等待 Gazebo、Nav2、MoveIt 和视觉节点完全退出。
2. 使用该工况对应的 `lighting_profile` 和 `enable_actor` 参数重新启动完整系统。
3. 等待 `/task/status` 为 `IDLE`，并确认腕部相机及 YOLO 检测话题有数据。
4. 发布同一条 A 工位抓取指令。
5. 等待任务完成后再结束当前仿真，启动下一工况。

各工况启动时使用下面的统一命令，只替换最后两个工况参数：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch lab_cobot_bringup lab_cobot.launch.py \
  gui:=true \
  use_rviz:=true \
  launch_navigation:=true \
  launch_moveit:=true \
  launch_perception:=true \
  use_dl_perception:=true \
  vision_backend:=eight_class \
  target_object:=aruco_sample \
  use_truth_pose:=false \
  use_refine_detect:=true \
  use_wrist_detect:=true \
  skip_visual_dock:=false \
  launch_g4g5_results:=false \
  lighting_profile:=normal \
  enable_actor:=false
```

六次启动分别替换为：

```text
C1: lighting_profile:=normal     enable_actor:=false
C2: lighting_profile:=normal     enable_actor:=true
C3: lighting_profile:=dark       enable_actor:=false
C4: lighting_profile:=dark       enable_actor:=true
C5: lighting_profile:=reflective enable_actor:=false
C6: lighting_profile:=reflective enable_actor:=true
```

每次启动后执行就绪检查：

```bash
ros2 topic hz /wrist_camera/image_raw
ros2 topic echo --once /yolo/detections
ros2 topic echo --once /task/status
```

当状态为 `IDLE` 时执行相同的视觉抓取任务：

```bash
ros2 topic pub --once /task/instruction std_msgs/msg/String \
  "{data: '把样件从A送到B'}"
```

动态遮挡工况中，任务节点只应在底盘和机械臂停止、目标重新进入腕部画面并通过视觉确认后使用检测结果。若遮挡者正好完全挡住目标，应等待目标重新可见，不应改用 Gazebo 真值完成抓取。

### 6. 仅调试视觉链路

只有在未运行一体化启动文件时，才单独启动视觉包：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch image_pkg pose_estimation.launch.py rviz:=true
```

无图形界面运行：

```bash
ros2 launch image_pkg pose_estimation.launch.py rviz:=false
```

可选启动设备指示灯识别：

```bash
ros2 launch image_pkg pose_estimation.launch.py indicator_state:=true rviz:=false
```

### 7. 检查主要话题与 TF

```bash
ros2 topic echo /yolo/detections
ros2 topic echo /yolo/bench/detections
ros2 topic echo /yolo/bench/poses
ros2 topic echo /perception/dual_camera/confirmed_poses
ros2 topic echo /perception/objects
ros2 topic echo /perception/target_pose
ros2 run tf2_ros tf2_echo base_link wrist_camera_optical_frame
```

## 主要节点与话题

### `yolo_world_node`

订阅 RGB 图像，后台执行 YOLO 推理，只保留最新帧以避免消息积压。

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/bench_camera/image_raw` | `sensor_msgs/msg/Image` | 默认 RGB 输入。 |
| 发布 | `/yolo/detections` | `std_msgs/msg/String` | 类别、置信度、检测框和图像尺寸。 |
| 发布 | `/yolo/annotated_image` | `sensor_msgs/msg/Image` | 带检测框图像。 |

### `yolo_pointcloud_pose_node`

该节点检查检测时间、RGB 与点云尺寸和 TF，然后计算三维目标位置。

| 方向 | 话题 | 类型 | 说明 |
| --- | --- | --- | --- |
| 订阅 | `/yolo/detections` | `std_msgs/msg/String` | 二维检测。 |
| 订阅 | `/image_pkg/camera_points` | `sensor_msgs/msg/PointCloud2` | 与 RGB 对齐的点云。 |
| 订阅 | `/perception/aruco_0/pose` | `geometry_msgs/msg/PoseStamped` | 可选 ArUco 来源。 |
| 发布 | `/perception/objects` | `vision_msgs/msg/Detection3DArray` | 三维对象列表。 |
| 发布 | `/perception/yolo/pose` | `geometry_msgs/msg/PoseStamped` | YOLO 三维位置。 |
| 发布 | `/perception/target_pose` | `geometry_msgs/msg/PoseStamped` | 统一目标位姿。 |
| 发布 | `/yolo/poses` | `std_msgs/msg/String` | JSON 兼容输出。 |

### 双相机与主动观察

广角相机用于发现目标并引导机器人选择观察位；腕部相机负责近距离确认。巡航节点可保存观察图像和清单，用于检查目标是否入画、是否贴边以及像素占比。目标未居中时，节点依据腕部内参和实时 TF 做受限微调；无框时执行受限搜索。

Station A 的标记对应关系为：

```text
ArUco ID 0/1 -> aruco_sample
ArUco ID 2   -> material_cube_red
ArUco ID 3   -> material_cube_green
ArUco ID 4   -> material_cube_blue
ArUco ID 5   -> material_cube_yellow
```

节点保留同一物体的可见标记面，转换到 `world` 后选择合适的标记平面，再根据已知几何关系恢复抓取方块的模型原点。其余无标记类别继续使用 YOLO + RGB-D 链路。

## 关键参数

完整默认值见 [`config/pose_estimation.yaml`](config/pose_estimation.yaml)。

| 参数 | 作用 |
| --- | --- |
| `image_topic` | RGB 输入话题。 |
| `model_path` | YOLO 权重路径。相对路径从安装后的包目录解析。 |
| `target_classes` | 允许检测的类别集合。 |
| `confidence_threshold` | 最低检测置信度。 |
| `nms_iou_threshold` | 非极大值抑制 IoU 阈值。 |
| `inference_imgsz` | YOLO 推理尺寸。 |
| `pointcloud_topic` | 有组织 RGB-D 点云话题。 |
| `target_frame` | 定位结果输出坐标系。 |
| `min_points` | 检测框内参与质心计算的最小有效点数。 |
| `max_detection_age_sec` | 检测与点云关联允许的最大时间间隔。 |
| `pose_source` | 统一位姿来源，可选 `yolo` 或 `aruco`。 |
| `target_label` | 单目标输出所选择的类别。 |
| `aruco_pose_topic` | ArUco 位姿输入话题。 |
| `selected_pose_topic` | 统一目标位姿输出话题。 |
| `publish_annotated_image` | 是否发布带检测框图像。 |

## 常见问题

- **没有二维检测**：确认 `best.pt` 已安装到工作空间、类别名与模型一致、图像话题正在发布，并检查置信度阈值。
- **没有三维目标**：确认点云是有组织点云，包含 `x/y/z` 字段，且宽高与检测图像完全一致。
- **无法转换到目标坐标系**：检查点云帧到 `base_link` 或 `world` 的 TF 是否连续可用。
- **腕部相机看不到目标**：检查机器人到站朝向、机械臂可达性、相机光轴定义和目标居中门控状态。
- **RViz 没有图像或点云**：检查相机 RGB、深度、`camera_info` 和相机 TF；无图形环境时使用 `rviz:=false`。
- **模型加载失败**：重新构建并执行 `source install/setup.bash`；确认已安装 Ultralytics，CPU 环境可设置 `device:=cpu`。

## 训练模型说明

部署推理只需要 `models/best.pt`，不会读取训练图像。`config/data.yaml` 用于记录类别与训练配置；只有继续训练时才需要把其中的数据集路径改为本机实际路径。
