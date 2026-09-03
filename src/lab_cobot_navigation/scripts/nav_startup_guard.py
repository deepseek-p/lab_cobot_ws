#!/usr/bin/env python3
"""launch 入口:启动 Nav2 就绪守卫."""
import rclpy

from lab_cobot_navigation.nav_startup_guard import NavStartupGuard


def main(args=None):
    rclpy.init(args=args)
    node = NavStartupGuard()
    try:
        rclpy.spin(node)
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
