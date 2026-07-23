# 六工况视觉识别测试结果

统一参数：同一 YOLO-World 权重、类别词、置信度和 NMS；每个工况采样 100 帧。

| 工况 | 帧数 | ArUco 识别率 | YOLO 识别率 |
|---|---:|---:|---:|
| a_station_normal_visible | 100 | 0.0% | 0.0% |
| a_station_probe | 1 | 0.0% | 0.0% |
| normal_visible | 100 | 0.0% | 0.0% |

## 说明

- ArUco：检测到 `aruco marker` 即记为成功。
- YOLO：检测到任一建图工位标签即记为成功；各标签明细在对应 `summary.json` 中。
- 每个工况最多保留 3 张 ArUco/YOLO 失败帧，位于同名工况目录。
- 六个工况目录均含 `summary.json` 后，才可作为完整六工况结论。
