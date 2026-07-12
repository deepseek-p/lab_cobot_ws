#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <limits>

namespace lab_cobot_gazebo::planar_safety
{
constexpr double kContactTolerance = 1e-12;
struct Point
{
  double x;
  double y;
};

struct OrientedBox
{
  Point center;
  double yaw;
  double length;
  double width;
};

struct AxisAlignedBox
{
  double min_x;
  double max_x;
  double min_y;
  double max_y;
};

inline bool isValid(const OrientedBox & box)
{
  return std::isfinite(box.center.x) && std::isfinite(box.center.y) &&
         std::isfinite(box.yaw) && std::isfinite(box.length) &&
         std::isfinite(box.width) && box.length > 0.0 && box.width > 0.0;
}

inline bool isValid(const AxisAlignedBox & box)
{
  return std::isfinite(box.min_x) && std::isfinite(box.max_x) &&
         std::isfinite(box.min_y) && std::isfinite(box.max_y) &&
         box.min_x < box.max_x && box.min_y < box.max_y;
}

inline bool isValidMargin(double margin)
{
  return std::isfinite(margin) && margin >= 0.0;
}

inline AxisAlignedBox expanded(const AxisAlignedBox & box, double margin)
{
  return {box.min_x - margin, box.max_x + margin,
    box.min_y - margin, box.max_y + margin};
}

inline double projectionGap(
  const OrientedBox & obb, const AxisAlignedBox & aabb, const Point & axis)
{
  const double c = std::cos(obb.yaw);
  const double s = std::sin(obb.yaw);
  const Point length_axis{c, s};
  const Point width_axis{-s, c};
  const double obb_center = obb.center.x * axis.x + obb.center.y * axis.y;
  const double obb_radius =
    obb.length * 0.5 * std::abs(length_axis.x * axis.x + length_axis.y * axis.y) +
    obb.width * 0.5 * std::abs(width_axis.x * axis.x + width_axis.y * axis.y);
  const Point aabb_center{
    (aabb.min_x + aabb.max_x) * 0.5, (aabb.min_y + aabb.max_y) * 0.5};
  const double aabb_center_projection = aabb_center.x * axis.x + aabb_center.y * axis.y;
  const double aabb_radius =
    (aabb.max_x - aabb.min_x) * 0.5 * std::abs(axis.x) +
    (aabb.max_y - aabb.min_y) * 0.5 * std::abs(axis.y);
  return std::abs(obb_center - aabb_center_projection) - obb_radius - aabb_radius;
}

// Positive means separated, zero means touching, negative means overlap.
inline double signedSeparation(const OrientedBox & obb, const AxisAlignedBox & aabb)
{
  if (!isValid(obb) || !isValid(aabb)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double c = std::cos(obb.yaw);
  const double s = std::sin(obb.yaw);
  const std::array<Point, 4> axes{{{1.0, 0.0}, {0.0, 1.0}, {c, s}, {-s, c}}};
  double maximum_gap = -std::numeric_limits<double>::infinity();
  for (const auto & axis : axes) {
    maximum_gap = std::max(maximum_gap, projectionGap(obb, aabb, axis));
  }
  return maximum_gap;
}

inline bool intersects(const OrientedBox & obb, const AxisAlignedBox & aabb)
{
  const double separation = signedSeparation(obb, aabb);
  return std::isfinite(separation) && separation <= kContactTolerance;
}

inline bool intersects(
  const OrientedBox & obb, const AxisAlignedBox & aabb, double margin)
{
  return isValidMargin(margin) && intersects(obb, expanded(aabb, margin));
}

inline double safetyScore(
  const OrientedBox & chassis, std::initializer_list<AxisAlignedBox> obstacles,
  double margin)
{
  if (!isValid(chassis) || !isValidMargin(margin)) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  double score = std::numeric_limits<double>::infinity();
  for (const auto & obstacle : obstacles) {
    if (!isValid(obstacle)) {
      return std::numeric_limits<double>::quiet_NaN();
    }
    score = std::min(score, signedSeparation(chassis, expanded(obstacle, margin)));
  }
  return score;
}

inline bool isMotionAllowed(
  const OrientedBox & current, const OrientedBox & next,
  std::initializer_list<AxisAlignedBox> obstacles, double margin)
{
  const double current_score = safetyScore(current, obstacles, margin);
  const double next_score = safetyScore(next, obstacles, margin);
  if (!std::isfinite(current_score) || !std::isfinite(next_score)) {
    return false;
  }
  if (next_score > kContactTolerance) {
    return true;
  }
  return current_score <= kContactTolerance &&
         next_score > current_score + kContactTolerance;
}

inline bool isSweptMotionAllowed(
  const OrientedBox & current, const OrientedBox & next,
  std::initializer_list<AxisAlignedBox> obstacles, double margin,
  double maximum_corner_step = 0.005)
{
  if (!isValid(current) || !isValid(next) || !isValidMargin(margin) ||
    !std::isfinite(maximum_corner_step) || maximum_corner_step <= 0.0)
  {
    return false;
  }
  const double dx = next.center.x - current.center.x;
  const double dy = next.center.y - current.center.y;
  const double yaw_delta = std::atan2(
    std::sin(next.yaw - current.yaw), std::cos(next.yaw - current.yaw));
  const double corner_radius = std::hypot(current.length, current.width) * 0.5;
  const double path_bound = std::hypot(dx, dy) + corner_radius * std::abs(yaw_delta);
  const auto steps = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(path_bound / maximum_corner_step)));

  OrientedBox previous = current;
  for (std::size_t step = 1; step <= steps; ++step) {
    const double fraction = static_cast<double>(step) / static_cast<double>(steps);
    const OrientedBox sample{
      {current.center.x + dx * fraction, current.center.y + dy * fraction},
      current.yaw + yaw_delta * fraction, current.length, current.width};
    const double segment_translation = std::hypot(
      sample.center.x - previous.center.x, sample.center.y - previous.center.y);
    const double segment_yaw = std::abs(
      std::atan2(
        std::sin(sample.yaw - previous.yaw), std::cos(sample.yaw - previous.yaw)));
    const double segment_bound = segment_translation +
      2.0 * corner_radius * std::sin(segment_yaw * 0.5);
    const double previous_score = safetyScore(previous, obstacles, margin);
    if (previous_score > kContactTolerance) {
      const double conservative_margin = margin + segment_bound;
      if (safetyScore(previous, obstacles, conservative_margin) <= kContactTolerance &&
        safetyScore(sample, obstacles, conservative_margin) <= kContactTolerance)
      {
        return false;
      }
    }
    if (!isMotionAllowed(previous, sample, obstacles, margin)) {
      return false;
    }
    previous = sample;
  }
  return true;
}
}  // namespace lab_cobot_gazebo::planar_safety
