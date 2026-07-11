"""Perception error sampling probe: mean/p95 statistics, never cherry-picked minima."""
# 对标赛题指标:目标空间定位误差(报告口径:均值/std/P95/最大,拒绝"最小误差"话术)
# 前置:先起栈但不跑任务(机器人静止在原点,odom≡world 成立——底盘插件零漂移已知):
#   ros2 launch lab_cobot_bringup lab_cobot.launch.py gui:=false use_rviz:=false launch_mission:=false
# 然后另一终端:
#   source install/setup.bash
#   python3 benchmarks/perception_error_probe.py [--samples 100] [--timeout 120]
# 注意:原点视角下 marker 可能不可见;可用 --require-view 提示先把底盘遥控到 A 工位
#   停靠位(ros2 topic pub /cmd_vel ...)或直接在 mission 停靠后 Ctrl-C mission 再采样。
import argparse
import math
import statistics
import sys
from datetime import datetime
from pathlib import Path

import rclpy
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

import tf2_ros


class ErrorProbe(Node):
    """Collect detection-vs-truth position error samples in the world frame."""

    def __init__(self):
        super().__init__("perception_error_probe")
        self._truth = None
        self._samples = []
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(
            ModelStates, "/gazebo/model_states", self._on_truth, 10)
        self.create_subscription(
            PoseStamped, "/perception/aruco_0/pose", self._on_detection, 10)

    def _on_truth(self, msg):
        if "aruco_sample" in msg.name:
            i = msg.name.index("aruco_sample")
            p = msg.pose[i].position
            self._truth = (p.x, p.y, p.z)

    def _on_detection(self, msg):
        # 检测为 base_link 系;经 TF 变换到 odom(本仿真 odom≡world,插件零漂移)
        if self._truth is None:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                "odom", msg.header.frame_id, rclpy.time.Time())
        except Exception:  # noqa: BLE001 TF 未就绪时跳过该样本
            return
        # 手工应用变换(平移+四元数旋转)
        q = tf.transform.rotation
        t = tf.transform.translation
        px, py, pz = msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
        # 四元数旋转 v' = q v q*
        x, y, z, w = q.x, q.y, q.z, q.w
        rx = (1 - 2 * (y * y + z * z)) * px + 2 * (x * y - w * z) * py \
            + 2 * (x * z + w * y) * pz
        ry = 2 * (x * y + w * z) * px + (1 - 2 * (x * x + z * z)) * py \
            + 2 * (y * z - w * x) * pz
        rz = 2 * (x * z - w * y) * px + 2 * (y * z + w * x) * py \
            + (1 - 2 * (x * x + y * y)) * pz
        wx, wy, wz = rx + t.x, ry + t.y, rz + t.z
        gx, gy, gz = self._truth
        err = math.sqrt((wx - gx) ** 2 + (wy - gy) ** 2 + (wz - gz) ** 2)
        self._samples.append({
            "err_m": err,
            "err_xyz": (wx - gx, wy - gy, wz - gz),
        })

    @property
    def samples(self):
        return self._samples


def main() -> int:
    parser = argparse.ArgumentParser(description="感知定位误差采样探针")
    parser.add_argument("--samples", type=int, default=100)
    parser.add_argument("--timeout", type=float, default=120.0)
    args = parser.parse_args()

    rclpy.init()
    node = ErrorProbe()
    start = node.get_clock().now()
    try:
        while rclpy.ok() and len(node.samples) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.2)
            elapsed = (node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > args.timeout:
                break
    except KeyboardInterrupt:
        pass

    n = len(node.samples)
    if n == 0:
        print("未采到样本:确认栈已起、marker 在相机视野内"
              "(可先让 mission 停靠到 A 工位后再采样)", file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    errs = sorted(s["err_m"] for s in node.samples)
    mean = statistics.fmean(errs)
    std = statistics.pstdev(errs)
    p95 = errs[max(0, int(n * 0.95) - 1)]
    ex = statistics.fmean(abs(s["err_xyz"][0]) for s in node.samples)
    ey = statistics.fmean(abs(s["err_xyz"][1]) for s in node.samples)
    ez = statistics.fmean(abs(s["err_xyz"][2]) for s in node.samples)

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = Path(__file__).parent / "results"
    out_dir.mkdir(exist_ok=True)
    report = out_dir / f"perception_error_{stamp}.md"
    body = "\n".join([
        f"# 感知定位误差采样报告 {stamp}",
        f"- 样本数: {n}",
        f"- **误差(3D 欧氏): 均值 {mean*1000:.1f} mm | std {std*1000:.1f} mm"
        f" | P95 {p95*1000:.1f} mm | 最大 {errs[-1]*1000:.1f} mm**",
        f"- 分轴平均绝对误差: |x| {ex*1000:.1f} / |y| {ey*1000:.1f}"
        f" / |z| {ez*1000:.1f} mm",
        "- 口径: 检测(base_link 系)经 TF 变换到 odom(≡world,底盘插件零漂移)"
        "与 Gazebo 真值逐样本对比;报告均值/P95,拒绝单帧最小值话术",
        f"- 赛题指标对照: 目标空间定位误差 <=1mm(实物级指标;"
        f"本仿真测量含视觉 z 系统偏差,详见报告 5.7 口径声明)",
    ])
    report.write_text(body, encoding="utf-8")
    print(body)
    print(f"\n报告: {report}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
