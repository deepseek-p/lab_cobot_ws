# image_pkg：YOLO-World 与 RGB-D 三维定位

`image_pkg` 是本工作空间的视觉包。它将 YOLO-World 的二维检测框和**与 RGB 图像逐像素对齐的有组织点云**关联，计算目标三维位置，再通过 TF 转换到 `base_link`，供抓取或任务模块使用。

默认链路不依赖 Gazebo 真值或模板模型：Gazebo `ModelStates` 仅用于记录误差评测，绝不会把真值发送给抓取模块。

## 功能与边界

- 以文本提示词运行 YOLO-World，输出类别、置信度和检测框。
- 校验 RGB 尺寸与点云宽高一致后，按检测框裁剪有组织点云。
- 忽略 `NaN`/`Inf` 点，并以中位数距离过滤框中的背景、桌面边缘等离群点，计算目标三维质心。
- 查询 `目标坐标系 <- 点云坐标系` 的 TF；默认输出到 `base_link`。
- 发布抓取可读取的标准 `vision_msgs/Detection3DArray` 和 `PoseStamped`。
- 可在 `yolo` 与 `aruco` 两种定位来源之间选择，并统一输出到一个目标位姿话题。
- 将算法估计值、可选 Gazebo 真值、XYZ 误差/总误差、抓取成功状态及失败文本写入 JSONL 日志。

当前默认的三维姿态消息中，**位置由点云计算，方向为单位四元数**。对无标记物体而言，该实现提供可靠的抓取位置，不应把单位方向误解为已完成物体朝向估计；`pcl_node.py` 中保留了 PCA 四元数工具函数，尚未接入默认发布链路。

## 数据流

```text
/bench_camera/image_raw                     /bench_camera/points（有组织 PointCloud2）
            │                                                   │
            ▼                                                   │
 yolo_world_node                                                  │
            │ /yolo/detections（JSON：框、类别、时间、图像尺寸）   │
            └──────────────────────────────┬────────────────────┘
                                           ▼
                         yolo_pointcloud_pose_node
                         框-像素关联 → 鲁棒质心 → TF 转 base_link
                                           │
              ┌────────────────────────────┼───────────────────────────┐
              ▼                            ▼                           ▼
 /perception/objects             /perception/yolo/pose       /yolo/poses
 Detection3DArray                PoseStamped                  JSON 位姿
              │
              └─ pose_source=yolo 时，同时发布 /perception/target_pose

/perception/aruco_0/pose ── pose_source=aruco ──► /perception/target_pose

/gazebo/model_states ─► 仅用于评测日志，不进入任何抓取位姿话题
```

## 目录与文件说明

| 路径 | 内容 |
| --- | --- |
| `image_pkg/__init__.py` | Python 包标记文件，目前不包含运行逻辑。 |
| `image_pkg/yolo_world_detector.py` | ROS 无关的 YOLO-World 封装；定义不可变的 `Detection`（类别、置信度、`xyxy` 框），负责加载模型、设置文本类别、选择 CPU/CUDA、执行推理。 |
| `image_pkg/yolo_world_node.py` | ROS 2 YOLO 节点。用后台线程推理，只保留最新图像以避免相机帧排队造成延迟；发布 JSON 检测结果和可选标注图。 |
| `image_pkg/pcl_node.py` | ROS 无关的点云数学工具。包含框内鲁棒质心、局部块质心、PCA 四元数和旋转矩阵转四元数函数，便于独立单元测试。 |
| `image_pkg/rgbd_pointcloud_node.py` | 用最新 RGB、深度图与相机内参生成与 RGB 像素对齐、带原始颜色的有组织 `/image_pkg/camera_points`。 |
| `image_pkg/yolo_pointcloud_pose_node.py` | 默认三维定位节点。订阅 YOLO JSON 与 PointCloud2，校验图像尺寸、裁剪点云、查 TF、发布三维目标消息，并写评测/抓取日志。 |
| `config/pose_estimation.yaml` | 三个视觉节点的默认 ROS 参数；生成点云默认发布至 `/image_pkg/camera_points`。 |
| `launch/pose_estimation.launch.py` | 默认启动两个视觉节点和 RViz；RViz 在主三维视图显示相机 RGB-D 点云。 |
| `config/camera_visualization.rviz` | RViz 预设，PointCloud2 显示订阅 `/image_pkg/camera_points`。 |
| `CMakeLists.txt` | 安装 Python ROS 节点、启动文件和配置。 |
| `package.xml` | 声明 ROS、TF、点云、视觉消息和 Gazebo 依赖。 |
| `setup.py` / `setup.cfg` | Python 包安装元数据及可执行文件安装目录。 |
| `resource/image_pkg` | ament 资源索引标记文件。 |
| `test/test_pcl_node.py` | 验证局部点云质心计算。 |
| `test/test_flake8.py`、`test/test_pep257.py` | 代码风格与文档字符串检查。 |
| `test/test_copyright.py` | 版权检查模板；当前显式跳过。 |

文件名带 `:Zone.Identifier` 的条目是从 Windows 文件系统带入的来源标记，不参与 ROS 构建和运行。

## 节点说明

### 1. `yolo_world_node`

用途：从 RGB 图像中检测配置的文本类别。

订阅：

| 话题 | 类型 | 默认值 |
| --- | --- | --- |
| 图像 | `sensor_msgs/msg/Image` | `/bench_camera/image_raw` |

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/yolo/detections` | `std_msgs/msg/String` | JSON 检测结果，是后续点云关联的输入。 |
| `/yolo/annotated_image` | `sensor_msgs/msg/Image` | 标注了检测框的 BGR 图像；可由参数关闭。 |

检测 JSON 约定如下。其中 `image_width` 与 `image_height` 用于阻止不同相机或缩放图像与点云发生错误关联。

```json
{
  "timestamp": 123.456,
  "frame_id": "camera_optical_frame",
  "image_width": 640,
  "image_height": 480,
  "detections": [
    {"label": "yellow toolbox", "confidence": 0.91,
     "box": [120, 80, 260, 390]}
  ]
}
```

运行机制：订阅回调不会直接推理，而是覆盖保存最新帧；后台线程取帧推理，避免慢模型导致 ROS 消息积压。`rgbd_pointcloud_node` 用每帧深度图与最新内参生成点云，RViz 在主三维视图显示 `/image_pkg/camera_points`；带检测框的图像仍发布至 `/yolo/annotated_image`。节点不再创建 OpenCV 相机窗口。

### 2. `yolo_pointcloud_pose_node`

用途：默认的 YOLO + 点云三维定位节点。

处理步骤：

1. 接收 `/yolo/detections`，检查 JSON 与框格式；缓存最近检测及其图像尺寸。
2. 接收有组织 `PointCloud2`。要求 `height > 1` 且包含 `x/y/z` 字段。
3. 若检测图像尺寸与点云 `width/height` 不一致，丢弃本次关联，防止像素误配。
4. 拒绝超过 `max_detection_age_sec` 的检测结果。
5. 在每个检测框内提取有限点，按到中位数点的距离保留最近 80%（最小阈值 3 mm），计算质心。
6. 查找 `target_frame <- cloud.header.frame_id` 的 TF，在目标坐标系中得到三维位置。
7. 为每个有效目标发布标准三维检测、位姿以及 JSON 兼容输出。

订阅：

| 话题 | 类型 | 用途 |
| --- | --- | --- |
| `/yolo/detections` | `std_msgs/msg/String` | YOLO 框、类别和时间戳。 |
| `/image_pkg/camera_points` | `sensor_msgs/msg/PointCloud2` | 由本包从 RGB-D 相机深度图生成的采样 XYZ 点云。 |
| `/perception/aruco_0/pose` | `geometry_msgs/msg/PoseStamped` | 当 `pose_source=aruco` 时转发的定位来源。 |
| `/gripper/status` | `std_msgs/msg/String` | 记录抓取成功或失败原因。 |
| `/gazebo/model_states` | `gazebo_msgs/msg/ModelStates` | 可选真值，仅评测使用。 |

发布：

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/perception/objects` | `vision_msgs/msg/Detection3DArray` | 抓取模块应优先读取的标准对象列表；消息坐标系为 `base_link`。 |
| `/perception/yolo/pose` | `geometry_msgs/msg/PoseStamped` | 每个有效 YOLO 目标分别发布一次。 |
| `/perception/target_pose` | `geometry_msgs/msg/PoseStamped` | 统一定位来源输出；`pose_source=yolo` 时来自本节点，`aruco` 时来自 ArUco 话题。 |
| `/yolo/poses` | `std_msgs/msg/String` | JSON 格式的三维位置，便于兼容旧接口。 |

评测日志默认为 `~/.ros/yolo_pose_evaluation.jsonl`，每行一个 JSON。`pose_estimate` 事件会记录估计位置、若可用则记录已转换到同一坐标系的 Gazebo 真值、`error_xyz` 和 `total_position_error`；`grasp` 事件记录 `success` 和 `failure_reason`。若 `gazebo_truth_frame` 没有到 `target_frame` 的 TF，日志仍会保留算法估计，但不产生误差字段。

## 关键参数

完整默认值见 [`config/pose_estimation.yaml`](config/pose_estimation.yaml)。

### YOLO 参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `image_topic` | `/bench_camera/image_raw` | RGB 输入。 |
| `model_path` | `yolov8s-worldv2.pt` | YOLO-World 权重路径或模型名。 |
| `target_classes` | 多个颜色/物体文本提示 | 要识别的开放词表类别。 |
| `confidence_threshold` | `0.35` | 最低置信度。 |
| `nms_iou_threshold` | `0.45` | NMS IoU 阈值。 |
| `device` | `auto` | `auto`、`cpu` 或 CUDA 设备编号。 |
| `inference_imgsz` | `640` | 推理尺寸。 |
| `publish_annotated_image` | `true` | 发布供 RViz 显示的标注图。 |

### YOLO+点云参数

| 参数 | 默认值 | 作用 |
| --- | --- | --- |
| `pointcloud_topic` | `/image_pkg/camera_points` | 本包生成的 RGB-D 点云。 |
| `target_frame` | `base_link` | 抓取使用的目标坐标系。 |
| `min_points` | `20` | 一个框内参与鲁棒质心的最小有效点数。 |
| `max_detection_age_sec` | `0.5` | 点云到达时可接受的检测最大年龄。 |
| `pose_source` | `yolo` | `yolo` 或 `aruco`，决定统一目标位姿来源。 |
| `aruco_pose_topic` | `/perception/aruco_0/pose` | ArUco 来源输入。 |
| `objects_topic` | `/perception/objects` | 标准三维对象输出。 |
| `selected_pose_topic` | `/perception/target_pose` | 统一目标位姿输出。 |
| `gazebo_truth_topic` | `/gazebo/model_states` | 评测真值输入；设为空字符串可关闭。 |
| `gazebo_truth_frame` | `odom` | Gazebo 真值所在坐标系，必须可 TF 到 `target_frame` 才能计算误差。 |
| `evaluation_log_path` | `~/.ros/yolo_pose_evaluation.jsonl` | JSONL 评测记录文件。 |

## 构建

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select image_pkg
source install/setup.bash
```

运行依赖包括 ROS 2 Humble、`cv_bridge`、`sensor_msgs_py`、`vision_msgs`、TF2、NumPy、OpenCV、PyTorch/Ultralytics YOLO-World。

## 启动与检查

先启动包含 RGB、点云和 TF 的仿真，例如：

```bash
ros2 launch lab_cobot_gazebo world.launch.py gui:=true
```

再启动默认视觉链路：

```bash
cd ~/lab_cobot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch image_pkg pose_estimation.launch.py
```

该命令默认启动 RViz：右侧主三维视图显示 `/image_pkg/camera_points` 点云，左侧保留实时 RGB `/bench_camera/image_raw` 与检测框图 `/yolo/annotated_image` 的预览。若只需启动算法节点，可传入 `rviz:=false`：

```bash
ros2 launch image_pkg pose_estimation.launch.py rviz:=false
```

常用检查命令：

```bash
ros2 topic echo /yolo/detections
ros2 topic echo /perception/objects
ros2 topic echo /perception/target_pose
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
tail -f ~/.ros/yolo_pose_evaluation.jsonl
```

切换到 ArUco 统一定位来源：

```bash
ros2 run image_pkg yolo_pointcloud_pose_node --ros-args \
  -p pose_source:=aruco
```

## 常见问题

- **没有三维目标输出**：确认点云是有组织的、带 `x/y/z` 字段，且与 RGB 完全同尺寸；检查 `/yolo/detections` 的时间戳是否在 `max_detection_age_sec` 内。
- **提示无法转到 `base_link`**：检查 `cloud.header.frame_id` 到 `base_link` 的 TF，必要时用 `tf2_echo` 检查。
- **只有估计值、没有误差字段**：`gazebo_truth_frame` 到 `base_link` 没有 TF，或 `gazebo_model_names` 没有匹配到模型；这不会影响抓取定位。
- **RViz 中没有点云**：确认 `/bench_camera/depth/image_raw` 与 `/bench_camera/camera_info` 正在发布，并检查 `camera_optical_frame` 的 TF；无图形环境时以 `rviz:=false` 启动算法节点。
- **YOLO 模型加载失败**：确认已安装包含 `YOLOWorld` 的 Ultralytics，并将 `model_path` 设置为可读取的权重；没有 CUDA 环境时使用 `device:=cpu`。
