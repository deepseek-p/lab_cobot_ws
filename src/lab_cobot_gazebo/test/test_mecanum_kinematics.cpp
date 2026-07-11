// 麦轮运动学纯函数单元测试(真行为测试,替代原源码文本断言)。
#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "lab_cobot_gazebo/mecanum_kinematics.hpp"

namespace
{

constexpr double kWheelRadius = 0.08;
constexpr double kWheelbaseRadius = 0.47;  // lx + ly, 与 URDF 一致
constexpr double kMaxLinear = 0.45;
constexpr double kMaxAngular = 0.9;

std::array<double, 3> Forward(const std::array<double, 4> & speeds)
{
  return lab_cobot_gazebo::TwistFromWheelSpeeds(
    speeds, kWheelRadius, kWheelbaseRadius, kMaxLinear, kMaxAngular);
}

TEST(MecanumKinematics, PureForwardWheelSpeedsYieldPureVx)
{
  // 四轮同向同速 -> 纯前进
  const auto twist = Forward({2.0, 2.0, 2.0, 2.0});
  EXPECT_NEAR(twist[0], kWheelRadius * 2.0, 1e-12);
  EXPECT_NEAR(twist[1], 0.0, 1e-12);
  EXPECT_NEAR(twist[2], 0.0, 1e-12);
}

TEST(MecanumKinematics, LateralWheelPatternYieldsPureVy)
{
  // [-,+,+,-] 组合 -> 纯左移(全向能力的运动学本体)
  const auto twist = Forward({-2.0, 2.0, 2.0, -2.0});
  EXPECT_NEAR(twist[0], 0.0, 1e-12);
  EXPECT_NEAR(twist[1], kWheelRadius * 2.0, 1e-12);
  EXPECT_NEAR(twist[2], 0.0, 1e-12);
}

TEST(MecanumKinematics, SpinWheelPatternYieldsPureWz)
{
  // [-,+,-,+] 组合 -> 纯原地旋转
  const auto twist = Forward({-2.0, 2.0, -2.0, 2.0});
  EXPECT_NEAR(twist[0], 0.0, 1e-12);
  EXPECT_NEAR(twist[1], 0.0, 1e-12);
  EXPECT_NEAR(twist[2], kWheelRadius * 2.0 / kWheelbaseRadius, 1e-12);
}

TEST(MecanumKinematics, ForwardIsExactInverseOfVisualizerIK)
{
  // 互逆性: 对一组一般 twist, IK(visualizer 同款矩阵) -> FK 恢复原值。
  // 这锁定了 cmd_vel -> 轮速命令 -> 车体 twist 链路的运动学一致性。
  const std::array<std::array<double, 3>, 4> cases = {{
    {0.30, 0.00, 0.00},
    {0.00, 0.20, 0.00},
    {0.00, 0.00, 0.60},
    {0.12, -0.08, 0.35},
  }};
  for (const auto & expected : cases) {
    const auto speeds = lab_cobot_gazebo::WheelSpeedsFromTwist(
      expected[0], expected[1], expected[2], kWheelRadius, kWheelbaseRadius);
    const auto twist = Forward(speeds);
    EXPECT_NEAR(twist[0], expected[0], 1e-12);
    EXPECT_NEAR(twist[1], expected[1], 1e-12);
    EXPECT_NEAR(twist[2], expected[2], 1e-12);
  }
}

TEST(MecanumKinematics, NonFiniteWheelSpeedsAreSanitizedToZeroContribution)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const auto twist = Forward({nan, inf, -inf, nan});
  EXPECT_NEAR(twist[0], 0.0, 1e-12);
  EXPECT_NEAR(twist[1], 0.0, 1e-12);
  EXPECT_NEAR(twist[2], 0.0, 1e-12);
}

TEST(MecanumKinematics, OutputIsClampedToConfiguredSpeedLimits)
{
  const auto twist = Forward({100.0, 100.0, 100.0, 100.0});
  EXPECT_NEAR(twist[0], kMaxLinear, 1e-12);
  const auto spin = Forward({-100.0, 100.0, -100.0, 100.0});
  EXPECT_NEAR(spin[2], kMaxAngular, 1e-12);
}

TEST(MecanumKinematics, WheelCommandWatchdog)
{
  using lab_cobot_gazebo::WheelCommandFresh;
  EXPECT_FALSE(WheelCommandFresh(-1.0, 10.0, 0.25));  // 从未收到命令
  EXPECT_TRUE(WheelCommandFresh(10.0, 10.2, 0.25));   // 命令新鲜
  EXPECT_FALSE(WheelCommandFresh(10.0, 10.3, 0.25));  // 超时应刹停
}

}  // namespace
