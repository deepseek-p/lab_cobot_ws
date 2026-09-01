#!/usr/bin/env python3
"""Serial, idempotent controller startup for slow Gazebo service responses."""

import time

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
OVERALL_STARTUP_TIMEOUT_SEC = 240.0
RETRY_SLEEP_SEC = 1.0


class ControllerBootstrap(Node):
    def __init__(self) -> None:
        super().__init__("lab_cobot_controller_bootstrap")

    def call(self, srv_type, name, request, *, allow_timeout: bool = False):
        client = self.create_client(srv_type, f"/controller_manager/{name}")
        if not client.wait_for_service(timeout_sec=SERVICE_TIMEOUT_SEC):
            if allow_timeout:
                self.get_logger().warn(f"service unavailable: {name}")
                return None
            raise RuntimeError(f"service unavailable: {name}")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=SERVICE_TIMEOUT_SEC)
        if future.result() is None:
            if allow_timeout:
                self.get_logger().warn(f"service timed out: {name}")
                return None
            raise RuntimeError(f"service timed out: {name}")
        return future.result()

    def controller_state(self, controller_name):
        response = self.call(
            ListControllers,
            "list_controllers",
            ListControllers.Request(),
            allow_timeout=True,
        )
        if response is None:
            return None
        for controller in response.controller:
            if controller.name == controller_name:
                return controller.state
        return None

    def _sleep_before_retry(self) -> None:
        deadline = time.monotonic() + RETRY_SLEEP_SEC
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _load(self, controller_name: str) -> None:
        request = LoadController.Request()
        request.name = controller_name
        response = self.call(
            LoadController,
            "load_controller",
            request,
            allow_timeout=True,
        )
        if response is not None and not response.ok:
            self.get_logger().warn(
                f"load request returned not ok for {controller_name}; "
                "rechecking controller state"
            )

    def _configure(self, controller_name: str) -> None:
        request = ConfigureController.Request()
        request.name = controller_name
        response = self.call(
            ConfigureController,
            "configure_controller",
            request,
            allow_timeout=True,
        )
        if response is not None and not response.ok:
            self.get_logger().warn(
                f"configure request returned not ok for {controller_name}; "
                "rechecking controller state"
            )

    def _activate(self, controller_name: str) -> None:
        request = SwitchController.Request()
        request.activate_controllers = [controller_name]
        request.strictness = SwitchController.Request.STRICT
        request.activate_asap = True
        request.timeout.sec = int(SERVICE_TIMEOUT_SEC)
        response = self.call(
            SwitchController,
            "switch_controller",
            request,
            allow_timeout=True,
        )
        if response is not None and not response.ok:
            self.get_logger().warn(
                f"activate request returned not ok for {controller_name}; "
                "rechecking controller state"
            )

    def ensure_active(self, controller_name: str, deadline: float) -> None:
        last_state = None
        last_operation = "query"
        while rclpy.ok() and time.monotonic() < deadline:
            state = self.controller_state(controller_name)
            if state is not None:
                last_state = state
            if state == "active":
                self.get_logger().info(f"{controller_name} already active")
                return
            if state is None:
                last_operation = "load"
                self.get_logger().info(f"loading {controller_name}")
                self._load(controller_name)
                self._sleep_before_retry()
                continue
            if state == "unconfigured":
                last_operation = "configure"
                self.get_logger().info(f"configuring {controller_name}")
                self._configure(controller_name)
                self._sleep_before_retry()
                continue
            last_operation = "activate"
            self.get_logger().info(
                f"activating {controller_name} from state={state}"
            )
            self._activate(controller_name)
            self._sleep_before_retry()

        state = self.controller_state(controller_name)
        if state == "active":
            self.get_logger().info(f"{controller_name} already active")
            return
        raise RuntimeError(
            "controller bootstrap timed out: "
            f"controller={controller_name} operation={last_operation} "
            f"last_state={state or last_state or 'unknown'}"
        )

    def final_states(self):
        response = self.call(ListControllers, "list_controllers", ListControllers.Request())
        states = {}
        for controller in response.controller:
            if controller.name in CONTROLLERS:
                states[controller.name] = controller.state
        return states


def main() -> None:
    rclpy.init()
    node = ControllerBootstrap()
    try:
        deadline = time.monotonic() + OVERALL_STARTUP_TIMEOUT_SEC
        for controller in CONTROLLERS:
            node.ensure_active(controller, deadline)
        states = node.final_states()
        missing = [name for name in CONTROLLERS if states.get(name) != "active"]
        if missing:
            details = ", ".join(
                f"{name}={states.get(name, 'missing')}" for name in CONTROLLERS
            )
            raise RuntimeError(f"controller bootstrap incomplete: {details}")
        node.get_logger().info(
            "CONTROLLER_BOOTSTRAP_READY "
            + " ".join(f"{name}=active" for name in CONTROLLERS)
        )
    except Exception as exc:  # noqa: BLE001
        node.get_logger().error(str(exc))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
