#!/usr/bin/env python3
"""ArUco 检测 + 6D 位姿 + TF/PoseStamped 发布节点。

改进自 eyrc ur5_control/aruco_detector:
- 订阅本项目 RGB-D 相机话题 /bench_camera/*
- 内参从 /bench_camera/camera_info 动态获取(不硬编码)
- 复用 lab_cobot_perception.pose_math 的针孔反投影
- headless 友好(无 cv2.imshow)
- 发布 TF: camera_optical_frame -> obj_<id>;并发布 PoseStamped /perception/aruco_<id>/pose

注:实际检测需运行时(Gazebo 相机图像 + 贴 ArUco 码的样件),本文件仅保证可编译/导入。
"""
import sys
import math

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped, PoseStamped

try:
    import cv2
    from cv_bridge import CvBridge
    _CV_AVAILABLE = True
except Exception:  # pragma: no cover - 运行时依赖
    _CV_AVAILABLE = False

import tf2_ros

from lab_cobot_perception.pose_math import pixel_to_camera


ARUCO_AREA_THRESHOLD = 1200  # 像素面积阈值,滤除过远/过小标记


def _make_aruco_detector():
    """兼容 OpenCV 新旧 ArUco API。返回 detect(image)->(corners, ids)。"""
    aruco = cv2.aruco
    dic = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    if hasattr(aruco, "ArucoDetector"):
        det = aruco.ArucoDetector(dic, aruco.DetectorParameters())
        return lambda img: det.detectMarkers(img)[:2]
    # 旧 API
    params = aruco.DetectorParameters_create()
    return lambda img: aruco.detectMarkers(img, dic, parameters=params)[:2]


def _marker_area(corner):
    c = corner.reshape(4, 2)
    w = np.linalg.norm(c[0] - c[1])
    h = np.linalg.norm(c[1] - c[2])
    return float(w * h)


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.declare_parameter("rgb_topic", "/bench_camera/image_raw")
        self.declare_parameter("depth_topic", "/bench_camera/depth/image_raw")
        self.declare_parameter("info_topic", "/bench_camera/camera_info")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "base_link")

        rgb = self.get_parameter("rgb_topic").value
        depth = self.get_parameter("depth_topic").value
        info = self.get_parameter("info_topic").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.target_frame = self.get_parameter("target_frame").value

        if not _CV_AVAILABLE:
            self.get_logger().error("cv2 / cv_bridge 不可用,节点无法检测(请检查依赖)")
            return

        self.bridge = CvBridge()
        self.detect = _make_aruco_detector()
        self.fx = self.fy = self.cx = self.cy = None
        self.rgb_img = None
        self.depth_img = None

        self.create_subscription(CameraInfo, info, self._info_cb, 10)
        self.create_subscription(Image, rgb, self._rgb_cb, 10)
        self.create_subscription(Image, depth, self._depth_cb, 10)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.pose_pubs = {}
        self.create_timer(0.2, self._process)  # 5 Hz
        self.get_logger().info("aruco_detector 启动")

    def _info_cb(self, msg: CameraInfo):
        k = msg.k
        self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]

    def _rgb_cb(self, msg: Image):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg: Image):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    def _process(self):
        if self.rgb_img is None or self.depth_img is None or self.fx is None:
            return
        gray = cv2.cvtColor(self.rgb_img, cv2.COLOR_BGR2GRAY)
        corners, ids = self.detect(gray)
        if ids is None:
            return
        for i, mid in enumerate(ids.flatten()):
            corner = corners[i][0]
            if _marker_area(corner) < ARUCO_AREA_THRESHOLD:
                continue
            cx_px = int(np.mean(corner[:, 0]))
            cy_px = int(np.mean(corner[:, 1]))
            depth = float(self.depth_img[cy_px, cx_px])
            if not math.isfinite(depth) or depth <= 0.0:
                continue  # 跳过 inf/nan/无效深度(Gazebo 深度图边缘常见)
            if depth > 50.0:  # 单位疑似 mm
                depth /= 1000.0
            x, y, z = pixel_to_camera(cx_px, cy_px, depth, self.fx, self.fy, self.cx, self.cy)
            self._publish(int(mid), x, y, z)

    def _publish(self, mid, x, y, z):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.optical_frame
        t.child_frame_id = f"obj_{mid}"
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        self.br.sendTransform(t)

        if mid not in self.pose_pubs:
            self.pose_pubs[mid] = self.create_publisher(
                PoseStamped, f"/perception/aruco_{mid}/pose", 10)
        ps = PoseStamped()
        ps.header = t.header
        ps.pose.position.x = x
        ps.pose.position.y = y
        ps.pose.position.z = z
        ps.pose.orientation.w = 1.0
        self.pose_pubs[mid].publish(ps)


def main():
    rclpy.init(args=sys.argv)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
