/**
 * @file planar_frame_rotation.hpp
 * @brief Convert planar linear velocities between the ROS base frame and the
 *        LeKiwi wheel-native frame.
 */

#pragma once

#include <Eigen/Core>

#include <cmath>

namespace robot_platform_pkg {

/**
 * heading_offset_rad is the yaw of the configured ROS base +X axis expressed
 * in the wheel-native frame. Angular velocity is invariant under this planar
 * coordinate rotation and is therefore intentionally not handled here.
 */
inline Eigen::Vector2d base_to_wheel_native(
  const Eigen::Vector2d & base_velocity, double heading_offset_rad)
{
  const double c = std::cos(heading_offset_rad);
  const double s = std::sin(heading_offset_rad);
  return Eigen::Vector2d{
    c * base_velocity.x() - s * base_velocity.y(),
    s * base_velocity.x() + c * base_velocity.y()};
}

inline Eigen::Vector2d wheel_native_to_base(
  const Eigen::Vector2d & native_velocity, double heading_offset_rad)
{
  const double c = std::cos(heading_offset_rad);
  const double s = std::sin(heading_offset_rad);
  return Eigen::Vector2d{
    c * native_velocity.x() + s * native_velocity.y(),
    -s * native_velocity.x() + c * native_velocity.y()};
}

}  // namespace robot_platform_pkg
