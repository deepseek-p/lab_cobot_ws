#!/usr/bin/env python3
from dataclasses import dataclass
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64MultiArray
from rclpy.parameter import Parameter


@dataclass
class SimpleTwist:
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


def sanitize_twist(twist):
    values = (twist.vx, twist.vy, twist.wz)
    if not all(math.isfinite(value) for value in values):
        return SimpleTwist()
    return twist


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def limit_twist(twist, max_vx, max_vy, max_wz):
    return SimpleTwist(
        _clamp(twist.vx, max_vx),
        _clamp(twist.vy, max_vy),
        _clamp(twist.wz, max_wz),
    )


def _ramp_value(current, target, max_step):
    delta = target - current
    if delta > max_step:
        return current + max_step
    if delta < -max_step:
        return current - max_step
    return target


def ramp_twist(current, target, dt, max_accel_xy, max_accel_wz):
    linear_step = max_accel_xy * dt
    angular_step = max_accel_wz * dt
    return SimpleTwist(
        _ramp_value(current.vx, target.vx, linear_step),
        _ramp_value(current.vy, target.vy, linear_step),
        _ramp_value(current.wz, target.wz, angular_step),
    )


def apply_deadband(twist, linear_deadband, angular_deadband):
    return SimpleTwist(
        0.0 if abs(twist.vx) < linear_deadband else twist.vx,
        0.0 if abs(twist.vy) < linear_deadband else twist.vy,
        0.0 if abs(twist.wz) < angular_deadband else twist.wz,
    )


def zero_if_timed_out(target, elapsed, command_timeout):
    if elapsed > command_timeout:
        return SimpleTwist()
    return target


def reset_twists_on_clock_jump(target, current, raw_dt, elapsed):
    if raw_dt < 0.0 or elapsed < 0.0:
        return SimpleTwist(), SimpleTwist(), True
    return target, current, False


def validate_configuration(
    wheel_radius,
    wheel_separation_width,
    wheel_separation_length,
    max_vx,
    max_vy,
    max_wz,
    max_accel_xy,
    max_accel_wz,
    command_timeout,
    linear_deadband,
    angular_deadband,
):
    if wheel_radius <= 0.0:
        raise ValueError('wheel_radius must be greater than zero')
    if wheel_separation_width <= 0.0:
        raise ValueError('wheel_separation_width must be greater than zero')
    if wheel_separation_length <= 0.0:
        raise ValueError('wheel_separation_length must be greater than zero')

    nonnegative = {
        'max_vx': max_vx,
        'max_vy': max_vy,
        'max_wz': max_wz,
        'max_accel_xy': max_accel_xy,
        'max_accel_wz': max_accel_wz,
        'command_timeout': command_timeout,
        'linear_deadband': linear_deadband,
        'angular_deadband': angular_deadband,
    }
    for name, value in nonnegative.items():
        if value < 0.0:
            raise ValueError(f'{name} must be nonnegative')


def twist_msg_to_simple(msg):
    return SimpleTwist(msg.linear.x, msg.linear.y, msg.angular.z)


def twist_to_wheel_speeds(
    vx,
    vy,
    wz,
    wheel_radius=0.07,
    wheel_separation_width=0.24,
    wheel_separation_length=0.175,
):
    """Convert a planar twist into signed controller wheel velocities."""
    k_geom = wheel_separation_length + wheel_separation_width
    v_fl = (vx - vy - wz * k_geom) / wheel_radius
    v_fr = (vx + vy + wz * k_geom) / wheel_radius
    v_bl = (vx + vy - wz * k_geom) / wheel_radius
    v_br = (vx - vy + wz * k_geom) / wheel_radius
    return [-v_fl, -v_fr, -v_bl, -v_br]


class RoverTwistRelay(Node):
    def __init__(self):
        super().__init__('rover_twist_relay')

        # Declare rover selector
        self.rover = self.declare_parameter('rover', 'mecanum3').value

        # Declare default (fallback) geometry
        self.declare_parameter('wheel_radius', 0.07)
        self.declare_parameter('wheel_separation_width', 0.24)
        self.declare_parameter('wheel_separation_length', 0.175)

        # Command limits and smoothing. Keep these aligned with the Gazebo
        # kinematic drive node in gazebo_bringup.launch.py.
        self.declare_parameter('max_vx', 0.5)
        self.declare_parameter('max_vy', 0.3)
        self.declare_parameter('max_wz', 1.2)
        self.declare_parameter('max_accel_xy', 0.5)
        self.declare_parameter('max_accel_wz', 1.5)
        self.declare_parameter('command_timeout', 0.25)
        self.declare_parameter('linear_deadband', 0.001)
        self.declare_parameter('angular_deadband', 0.001)

        # Declare per-rover parameters (flat names)
        self.declare_parameter('mecanum3.wheel_radius', 0.07)
        self.declare_parameter('mecanum3.wheel_separation_width', 0.24)
        self.declare_parameter('mecanum3.wheel_separation_length', 0.175)

        self.declare_parameter('g120a.wheel_radius', 0.085)
        self.declare_parameter('g120a.wheel_separation_width', 0.30)
        self.declare_parameter('g120a.wheel_separation_length', 0.22)

        self.declare_parameter('g40a_lb.wheel_radius', 0.0762)
        self.declare_parameter('g40a_lb.wheel_separation_width', 0.2358)
        self.declare_parameter('g40a_lb.wheel_separation_length', 0.040)

        # Apply rover-specific geometry
        self.apply_rover_geometry()

        # Publisher
        self.pub_wheel = self.create_publisher(
            Float64MultiArray,
            '/wheel_velocity_controller/commands',
            10
        )

        self.max_vx = self.get_parameter('max_vx').value
        self.max_vy = self.get_parameter('max_vy').value
        self.max_wz = self.get_parameter('max_wz').value
        self.max_accel_xy = self.get_parameter('max_accel_xy').value
        self.max_accel_wz = self.get_parameter('max_accel_wz').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.linear_deadband = self.get_parameter('linear_deadband').value
        self.angular_deadband = self.get_parameter('angular_deadband').value

        validate_configuration(
            self.r,
            self.W,
            self.L,
            self.max_vx,
            self.max_vy,
            self.max_wz,
            self.max_accel_xy,
            self.max_accel_wz,
            self.command_timeout,
            self.linear_deadband,
            self.angular_deadband,
        )

        # Subscribers: /rover_twist remains the project-native topic, while
        # /cmd_vel lets Nav2 and standard teleop tools drive the rover.
        self.sub_twist = self.create_subscription(
            Twist,
            '/rover_twist',
            self.on_twist_received,
            10
        )
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.on_twist_received,
            10
        )

        self.target_twist = SimpleTwist()
        self.current_twist = SimpleTwist()
        self.last_command_time = self.get_clock().now()
        self.last_update_time = self.last_command_time

        # 100 Hz timer
        self.timer = self.create_timer(0.01, self.on_timer)

        self.get_logger().info(
            f"rover_twist_relay started (rover={self.rover})"
        )

    # ------------------------------------------------------------
    def apply_rover_geometry(self):
        prefix = f'{self.rover}.'
        params = self._parameters

        def get_param(name, default):
            key = prefix + name
            if key in params:
                return params[key].value
            return default

        wheel_radius = get_param(
            'wheel_radius',
            self.get_parameter('wheel_radius').value
        )
        width = get_param(
            'wheel_separation_width',
            self.get_parameter('wheel_separation_width').value
        )
        length = get_param(
            'wheel_separation_length',
            self.get_parameter('wheel_separation_length').value
        )

        self.set_parameters([
            Parameter('wheel_radius', Parameter.Type.DOUBLE, wheel_radius),
            Parameter('wheel_separation_width', Parameter.Type.DOUBLE, width),
            Parameter(
                'wheel_separation_length', Parameter.Type.DOUBLE, length
            ),
        ])

        self.r = wheel_radius
        self.W = width
        self.L = length
        self.k_geom = self.L + self.W

    # Convert Twist to wheel velocities

    def twist_to_wheels(self, twist):
        msg = Float64MultiArray()
        msg.data = twist_to_wheel_speeds(
            twist.vx,
            twist.vy,
            twist.wz,
            self.r,
            self.W,
            self.L,
        )
        return msg

    # ------------------------------------------------------------
    def on_twist_received(self, msg):
        twist = sanitize_twist(twist_msg_to_simple(msg))
        limited = limit_twist(
            twist,
            self.max_vx,
            self.max_vy,
            self.max_wz,
        )
        self.target_twist = apply_deadband(
            limited,
            self.linear_deadband,
            self.angular_deadband,
        )
        self.last_command_time = self.get_clock().now()

    # ------------------------------------------------------------
    def on_timer(self):
        now = self.get_clock().now()
        raw_dt = (now - self.last_update_time).nanoseconds * 1e-9
        dt = raw_dt
        self.last_update_time = now
        if dt <= 0.0 or dt > 0.2:
            dt = 0.01

        elapsed_since_command = (
            now - self.last_command_time
        ).nanoseconds * 1e-9
        self.target_twist, self.current_twist, clock_reset = (
            reset_twists_on_clock_jump(
                self.target_twist,
                self.current_twist,
                raw_dt,
                elapsed_since_command,
            )
        )
        if clock_reset:
            self.last_command_time = now
            self.last_update_time = now
            self.pub_wheel.publish(self.twist_to_wheels(self.current_twist))
            return

        if elapsed_since_command > self.command_timeout:
            self.target_twist = SimpleTwist()
            self.current_twist = SimpleTwist()
            self.pub_wheel.publish(self.twist_to_wheels(self.current_twist))
            return

        target_twist = zero_if_timed_out(
            self.target_twist,
            elapsed_since_command,
            self.command_timeout,
        )

        self.current_twist = ramp_twist(
            self.current_twist,
            target_twist,
            dt,
            self.max_accel_xy,
            self.max_accel_wz,
        )
        self.current_twist = apply_deadband(
            self.current_twist,
            self.linear_deadband,
            self.angular_deadband,
        )
        self.pub_wheel.publish(self.twist_to_wheels(self.current_twist))


def main():
    rclpy.init()
    node = RoverTwistRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
