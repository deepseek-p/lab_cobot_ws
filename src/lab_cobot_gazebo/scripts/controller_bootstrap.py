#!/usr/bin/env python3
"""Serial, idempotent controller startup for slow Gazebo service responses."""

import rclpy
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from rclpy.node import Node


CONTROLLERS = (
    "joint_state_broadcaster",
    "joint_trajectory_controller",
    "gripper_position_controller",
    "wheel_velocity_controller",
)
SERVICE_TIMEOUT_SEC = 60.0


class ControllerBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("lab_cobot_controller_bootstrap")

    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, f"/controller_manager/{name}")
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            raise RuntimeError(f"service unavailable: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if future.result() is None:
            raise RuntimeError(f"service timed out: {name}")
        return future.result()

    def controller_state(self, controller_name):
        response = self.call(ListControllers, "list_controllers", ListControllers.Request())
        for controller in response.controller:
            if controller.name == controller_name:
                return controller.state
        return None

    def ensure_active(self, controller_name: str) -> None:
        state = self.controller_state(controller_name)
        if state is None:
            request = LoadController.Request()
            request.name = controller_name
            if not self.call(LoadController, "load_controller", request).ok:
                raise RuntimeError(f"failed to load {controller_name}")
            state = self.controller_state(controller_name)
        if state == "active":
            self.get_logger().info(f"{controller_name} already active")
            return
        if state == "unconfigured":
            request = ConfigureController.Request()
            request.name = controller_name
            if not self.call(ConfigureController, "configure_controller", request).ok:
                raise RuntimeError(f"failed to configure {controller_name}")
        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(SERVICE_TIMEOUT_SEC)
        if not self.call(SwitchController, "switch_controller", request).ok:
            raise RuntimeError(f"failed to activate {controller_name}")
        self.get_logger().info(f"activated {controller_name}")


def main() -> None:
    rclpy.init()
    node = ControllerBootstrap()
    try:
        for controller in CONTROLLERS:
            node.ensure_active(controller)
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
