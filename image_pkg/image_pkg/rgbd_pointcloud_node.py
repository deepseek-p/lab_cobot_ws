#!/usr/bin/env python3
"""Publish a PointCloud2 directly from the RGB-D camera depth stream."""

import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class RgbdPointCloudNode(Node):
    """Create an organized RGB-XYZ cloud without strict timestamp matching."""

    def __init__(self):
        super().__init__("rgbd_pointcloud")
        self.declare_parameter("rgb_topic", "/bench_camera/image_raw")
        self.declare_parameter("depth_topic", "/bench_camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/bench_camera/camera_info")
        self.declare_parameter("pointcloud_topic", "/image_pkg/camera_points")
        self.declare_parameter("min_depth", 0.05)
        self.declare_parameter("max_depth", 3.0)
        self.bridge = CvBridge()
        self.camera_info = None
        self.rgb_image = None
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.publisher = self.create_publisher(
            PointCloud2, self.get_parameter("pointcloud_topic").value, 1
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter("camera_info_topic").value,
            self._camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("rgb_topic").value,
            self._rgb_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter("depth_topic").value,
            self._depth_callback,
            qos_profile_sensor_data,
        )

    def _camera_info_callback(self, message):
        if message.k[0] > 0.0 and message.k[4] > 0.0:
            self.camera_info = message

    def _rgb_callback(self, message):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(
                message, desired_encoding="bgr8"
            ).copy()
        except CvBridgeError as error:
            self.get_logger().warning(f"RGB frame skipped: {error}")

    def _depth_callback(self, message):
        if self.camera_info is None or self.rgb_image is None:
            return
        try:
            depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        except CvBridgeError as error:
            self.get_logger().warning(f"Depth frame skipped: {error}")
            return
        if depth.ndim != 2:
            self.get_logger().warning("Depth frame is not a single-channel image")
            return
        if self.rgb_image.shape[:2] != depth.shape:
            self.get_logger().warning(
                "RGB and depth image dimensions differ; point cloud skipped"
            )
            return
        depth = depth.astype(np.float32, copy=False)
        if message.encoding.upper() in {"16UC1", "MONO16"}:
            depth = depth * 0.001
        rows, columns = np.indices(depth.shape, dtype=np.float32)
        matrix = self.camera_info.k
        x = (columns - matrix[2]) * depth / matrix[0]
        y = (rows - matrix[5]) * depth / matrix[4]
        valid = np.isfinite(depth)
        valid &= depth >= self.min_depth
        valid &= depth <= self.max_depth
        bgr = self.rgb_image
        rgb_uint32 = (
            (bgr[:, :, 2].astype(np.uint32) << 16)
            | (bgr[:, :, 1].astype(np.uint32) << 8)
            | bgr[:, :, 0].astype(np.uint32)
        )
        packed_rgb = rgb_uint32.view(np.float32)
        points = np.dstack((x, y, depth, packed_rgb)).astype(
            np.float32, copy=False
        )
        points[~valid] = np.nan
        points[:, :, 3][~valid] = 0.0
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        cloud = point_cloud2.create_cloud(message.header, fields, points)
        # create_cloud reverses the dimensions of a 2-D structured
        # array. Restore the conventional height(rows) x width(columns).
        cloud.height, cloud.width = depth.shape
        cloud.row_step = cloud.point_step * cloud.width
        self.publisher.publish(cloud)


def main(args=None):
    rclpy.init(args=args)
    node = RgbdPointCloudNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
