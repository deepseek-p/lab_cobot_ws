#!/usr/bin/env python3
"""
ArUco detection and object pose publishing node.

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
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Quaternion, TransformStamped, PoseStamped

try:
    import cv2
    _CV_AVAILABLE = True
except Exception:  # pragma: no cover - 运行时依赖
    cv2 = None
    _CV_AVAILABLE = False

import tf2_ros

from lab_cobot_perception.pose_math import offset_along_camera_ray, pixel_to_camera


ARUCO_AREA_THRESHOLD = 800  # 像素面积阈值,滤除过远/过小标记
DEFAULT_MARKER_SIZE_M = 0.07
DEFAULT_GAZEBO_MODEL_NAME = "aruco_sample"
DEFAULT_GAZEBO_REFERENCE_FRAME = "odom"
DEFAULT_MODEL_STATES_TOPIC = "/gazebo/model_states"


def _make_aruco_detector():
    """Return a detect(image) adapter for old and new OpenCV ArUco APIs."""
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


def _quat_normalize(q):
    x, y, z, w = [float(v) for v in q]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def _quat_multiply_raw(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _quat_multiply(a, b):
    return _quat_normalize(_quat_multiply_raw(a, b))


def _quat_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quat_rotate(q, point):
    px, py, pz = point
    rotated = _quat_multiply_raw(
        _quat_multiply_raw(_quat_normalize(q), (px, py, pz, 0.0)),
        _quat_conjugate(_quat_normalize(q)),
    )
    return (rotated[0], rotated[1], rotated[2])


def _quat_from_msg(q: Quaternion):
    return (q.x, q.y, q.z, q.w)


def _quat_to_msg(q):
    msg = Quaternion()
    msg.x, msg.y, msg.z, msg.w = _quat_normalize(q)
    return msg


def _quat_from_rotation_matrix(matrix):
    m = np.asarray(matrix, dtype=np.float64)
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return _quat_normalize((
            (m[2, 1] - m[1, 2]) / s,
            (m[0, 2] - m[2, 0]) / s,
            (m[1, 0] - m[0, 1]) / s,
            0.25 * s,
        ))
    if m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2.0
        return _quat_normalize((
            0.25 * s,
            (m[0, 1] + m[1, 0]) / s,
            (m[0, 2] + m[2, 0]) / s,
            (m[2, 1] - m[1, 2]) / s,
        ))
    if m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2.0
        return _quat_normalize((
            (m[0, 1] + m[1, 0]) / s,
            0.25 * s,
            (m[1, 2] + m[2, 1]) / s,
            (m[0, 2] - m[2, 0]) / s,
        ))
    s = math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2.0
    return _quat_normalize((
        (m[0, 2] + m[2, 0]) / s,
        (m[1, 2] + m[2, 1]) / s,
        0.25 * s,
        (m[1, 0] - m[0, 1]) / s,
    ))


def estimate_marker_pose_from_corners(
    corner,
    marker_size_m: float,
    camera_matrix,
    dist_coeffs,
):
    """Estimate marker center position and orientation in the optical frame."""
    half = float(marker_size_m) / 2.0
    object_points = np.array(
        [
            [-half, -half, 0.0],
            [half, -half, 0.0],
            [half, half, 0.0],
            [-half, half, 0.0],
        ],
        dtype=np.float32,
    )
    image_points = np.asarray(corner, dtype=np.float32).reshape(4, 2)
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        np.asarray(camera_matrix, dtype=np.float64),
        None if dist_coeffs is None else np.asarray(dist_coeffs, dtype=np.float64),
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("solvePnP failed for ArUco marker")
    rotation, _ = cv2.Rodrigues(rvec)
    return (
        tuple(float(v) for v in tvec.reshape(3)),
        _quat_from_rotation_matrix(rotation),
    )


def model_pose_to_object_transform(
    object_id: int,
    pose,
    frame_id: str = DEFAULT_GAZEBO_REFERENCE_FRAME,
) -> TransformStamped:
    """Build the object-center TF from a Gazebo model pose."""
    t = TransformStamped()
    t.header.frame_id = frame_id
    t.child_frame_id = f"obj_{int(object_id)}"
    t.transform.translation.x = pose.position.x
    t.transform.translation.y = pose.position.y
    t.transform.translation.z = pose.position.z
    t.transform.rotation = pose.orientation
    return t


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.declare_parameter("object_id", 0)
        self.declare_parameter("use_gazebo_model_pose", False)
        self.declare_parameter("gazebo_model_name", DEFAULT_GAZEBO_MODEL_NAME)
        self.declare_parameter("gazebo_reference_frame", DEFAULT_GAZEBO_REFERENCE_FRAME)
        self.declare_parameter("gazebo_model_states_topic", DEFAULT_MODEL_STATES_TOPIC)
        self.declare_parameter("rgb_topic", "/bench_camera/image_raw")
        self.declare_parameter("depth_topic", "/bench_camera/depth/image_raw")
        self.declare_parameter("info_topic", "/bench_camera/camera_info")
        self.declare_parameter("optical_frame", "camera_optical_frame")
        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("marker_size_m", DEFAULT_MARKER_SIZE_M)
        self.declare_parameter("marker_to_object_center_m", 0.035)

        self.object_id = int(self.get_parameter("object_id").value)
        self.use_gazebo_model_pose = bool(
            self.get_parameter("use_gazebo_model_pose").value
        )
        self.gazebo_model_name = str(self.get_parameter("gazebo_model_name").value)
        self.gazebo_reference_frame = str(
            self.get_parameter("gazebo_reference_frame").value
        )
        self.pose_pubs = {}
        self.br = tf2_ros.TransformBroadcaster(self)

        if self.use_gazebo_model_pose:
            topic = str(self.get_parameter("gazebo_model_states_topic").value)
            self.create_subscription(ModelStates, topic, self._model_states_cb, 10)
            self.get_logger().info(
                "aruco_detector using Gazebo model pose: %s -> obj_%d"
                % (self.gazebo_model_name, self.object_id)
            )
            return

        rgb = self.get_parameter("rgb_topic").value
        depth = self.get_parameter("depth_topic").value
        info = self.get_parameter("info_topic").value
        self.optical_frame = self.get_parameter("optical_frame").value
        self.target_frame = self.get_parameter("target_frame").value
        self.marker_size_m = float(self.get_parameter("marker_size_m").value)
        self.marker_to_object_center_m = float(
            self.get_parameter("marker_to_object_center_m").value
        )

        if not _CV_AVAILABLE:
            self.get_logger().error("cv2 / cv_bridge 不可用,节点无法检测(请检查依赖)")
            return

        try:
            from cv_bridge import CvBridge
        except Exception as ex:  # pragma: no cover - runtime dependency path
            self.get_logger().error(f"cv_bridge 不可用,节点无法检测: {ex}")
            return

        self.bridge = CvBridge()
        self.detect = _make_aruco_detector()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.fx = self.fy = self.cx = self.cy = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.rgb_img = None
        self.depth_img = None

        self.create_subscription(CameraInfo, info, self._info_cb, 10)
        self.create_subscription(Image, rgb, self._rgb_cb, 10)
        self.create_subscription(Image, depth, self._depth_cb, 10)
        self.create_timer(0.2, self._process)  # 5 Hz
        self.get_logger().info("aruco_detector 启动")

    def _info_cb(self, msg: CameraInfo):
        k = msg.k
        self.fx, self.fy, self.cx, self.cy = k[0], k[4], k[2], k[5]
        self.camera_matrix = np.asarray(k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = (
            np.asarray(msg.d, dtype=np.float64) if len(msg.d) > 0 else None
        )

    def _rgb_cb(self, msg: Image):
        self.rgb_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def _depth_cb(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if msg.encoding in ("16UC1", "mono16"):
            self.depth_img = img.astype(np.float32) / 1000.0
        elif msg.encoding == "32FC1":
            self.depth_img = img.astype(np.float32)
        else:
            self.get_logger().warning(
                f"未知深度编码 {msg.encoding}，按米处理",
                once=True,
            )
            self.depth_img = img.astype(np.float32)

    def _process(self):
        if (
            self.rgb_img is None
            or self.depth_img is None
            or self.camera_matrix is None
        ):
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
            try:
                _marker_position, marker_orientation = (
                    estimate_marker_pose_from_corners(
                        corner,
                        self.marker_size_m,
                        self.camera_matrix,
                        self.dist_coeffs,
                    )
                )
            except Exception as ex:  # noqa: BLE001
                self.get_logger().warn(f"ArUco 6D 位姿估计失败: {ex}")
                continue
            marker_point = pixel_to_camera(
                cx_px,
                cy_px,
                depth,
                self.fx,
                self.fy,
                self.cx,
                self.cy,
            )
            x, y, z = offset_along_camera_ray(
                marker_point,
                self.marker_to_object_center_m,
            )
            self._publish(int(mid), (x, y, z), marker_orientation)

    def _model_states_cb(self, msg: ModelStates):
        try:
            index = msg.name.index(self.gazebo_model_name)
        except ValueError:
            return

        transform = model_pose_to_object_transform(
            object_id=self.object_id,
            pose=msg.pose[index],
            frame_id=self.gazebo_reference_frame,
        )
        transform.header.stamp = self.get_clock().now().to_msg()
        self.br.sendTransform(transform)

        ps = PoseStamped()
        ps.header = transform.header
        ps.pose.position.x = transform.transform.translation.x
        ps.pose.position.y = transform.transform.translation.y
        ps.pose.position.z = transform.transform.translation.z
        ps.pose.orientation = transform.transform.rotation
        self._pose_pub(self.object_id).publish(ps)

    def _pose_pub(self, mid: int):
        if mid not in self.pose_pubs:
            self.pose_pubs[mid] = self.create_publisher(
                PoseStamped, f"/perception/aruco_{mid}/pose", 10)
        return self.pose_pubs[mid]

    def _pose_in_target_frame(self, position, orientation):
        if self.target_frame == self.optical_frame:
            return self.optical_frame, position, _quat_normalize(orientation)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.optical_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05),
            )
        except Exception as ex:  # noqa: BLE001
            self.get_logger().warn(
                "无法将 ArUco 位姿从 %s 变换到 %s: %s"
                % (self.optical_frame, self.target_frame, ex)
            )
            return None

        tf_q = _quat_from_msg(transform.transform.rotation)
        rotated = _quat_rotate(tf_q, position)
        translated = (
            rotated[0] + transform.transform.translation.x,
            rotated[1] + transform.transform.translation.y,
            rotated[2] + transform.transform.translation.z,
        )
        return (
            self.target_frame,
            translated,
            _quat_multiply(tf_q, orientation),
        )

    def _publish(self, mid, position, orientation):
        target_pose = self._pose_in_target_frame(position, orientation)
        if target_pose is None:
            return
        frame_id, target_position, target_orientation = target_pose

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = f"obj_{mid}"
        t.transform.translation.x = target_position[0]
        t.transform.translation.y = target_position[1]
        t.transform.translation.z = target_position[2]
        t.transform.rotation = _quat_to_msg(target_orientation)
        self.br.sendTransform(t)

        ps = PoseStamped()
        ps.header = t.header
        ps.pose.position.x = target_position[0]
        ps.pose.position.y = target_position[1]
        ps.pose.position.z = target_position[2]
        ps.pose.orientation = _quat_to_msg(target_orientation)
        self._pose_pub(mid).publish(ps)


def main():
    rclpy.init(args=sys.argv)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
