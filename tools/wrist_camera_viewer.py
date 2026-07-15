#!/usr/bin/env python3
"""Lightweight wrist camera viewer with a resizable non-fullscreen window."""
# 轻量腕相机取景器:替代 rqt_image_view(WSLg 下会恢复全屏状态且难调整)。
# 窗口初始 640x480,可自由拖拽缩放;按 q 或 Esc 退出。
# 用法: python3 tools/wrist_camera_viewer.py [topic]  (默认 /wrist_camera/image_raw)
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

WINDOW = "wrist_camera (q to quit)"


class Viewer(Node):

    def __init__(self, topic: str):
        super().__init__("wrist_camera_viewer")
        self.frame = None
        self.create_subscription(Image, topic, self._cb, 1)

    def _cb(self, msg: Image):
        # 不经 cv_bridge:直接按 encoding reshape,规避本机 cv2 双版本纠缠。
        data = np.frombuffer(msg.data, dtype=np.uint8)
        img = data.reshape(msg.height, msg.width, -1)
        if msg.encoding == "rgb8":
            img = img[:, :, ::-1]  # 转 BGR 供 imshow
        self.frame = img


def main():
    import cv2  # 函数内 lazy import(项目依赖纪律)
    topic = sys.argv[1] if len(sys.argv) > 1 else "/wrist_camera/image_raw"
    rclpy.init()
    node = Viewer(topic)
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 640, 480)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.frame is not None:
                cv2.imshow(WINDOW, node.frame)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
