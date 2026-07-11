// 麦克纳姆底盘运动学纯函数(无 Gazebo 依赖,可被 gtest 直接单测)。
// 轮序约定与 mecanum_wheel_visualizer.py / URDF 一致: [fl, fr, rl, rr]。
// 正解与 visualizer 的逆解矩阵严格互逆(FK·IK = I, 见 test_mecanum_kinematics.cpp)。
#ifndef LAB_COBOT_GAZEBO__MECANUM_KINEMATICS_HPP_
#define LAB_COBOT_GAZEBO__MECANUM_KINEMATICS_HPP_

#include <algorithm>
#include <array>
#include <cmath>

namespace lab_cobot_gazebo
{

inline double FiniteOrZero(const double value)
{
  return std::isfinite(value) ? value : 0.0;
}

inline double ClampAbs(const double value, const double limit)
{
  return std::clamp(value, -limit, limit);
}

// 麦轮正解: 轮角速度 [fl,fr,rl,rr] (rad/s) -> 底盘 twist (vx, vy, wz)。
// wheelbase_radius = lx + ly (纵横半轴距之和)。输出按限幅截断。
inline std::array<double, 3> TwistFromWheelSpeeds(
  const std::array<double, 4> & speeds,
  const double wheel_radius,
  const double wheelbase_radius,
  const double max_linear_speed,
  const double max_angular_speed)
{
  const double fl = FiniteOrZero(speeds[0]);
  const double fr = FiniteOrZero(speeds[1]);
  const double rl = FiniteOrZero(speeds[2]);
  const double rr = FiniteOrZero(speeds[3]);

  const double vx = wheel_radius * (fl + fr + rl + rr) / 4.0;
  const double vy = wheel_radius * (-fl + fr + rl - rr) / 4.0;
  const double wz = wheel_radius * (-fl + fr - rl + rr) /
    (4.0 * wheelbase_radius);

  return {
    ClampAbs(vx, max_linear_speed),
    ClampAbs(vy, max_linear_speed),
    ClampAbs(wz, max_angular_speed),
  };
}

// 麦轮逆解(与 mecanum_wheel_visualizer.py 的矩阵一致),主要供互逆性
// 测试与调试对拍使用;插件运行时输入是外部逆解产出的轮速命令。
inline std::array<double, 4> WheelSpeedsFromTwist(
  const double vx,
  const double vy,
  const double wz,
  const double wheel_radius,
  const double wheelbase_radius)
{
  const double lw = wheelbase_radius * wz;
  return {
    (vx - vy - lw) / wheel_radius,
    (vx + vy + lw) / wheel_radius,
    (vx + vy - lw) / wheel_radius,
    (vx - vy + lw) / wheel_radius,
  };
}

// 轮速命令看门狗:无命令或距上次命令超过 timeout 视为过期(应刹停)。
inline bool WheelCommandFresh(
  const double last_command_time,
  const double now,
  const double timeout)
{
  return last_command_time >= 0.0 && (now - last_command_time) <= timeout;
}

}  // namespace lab_cobot_gazebo

#endif  // LAB_COBOT_GAZEBO__MECANUM_KINEMATICS_HPP_
