#include "robot_platform_pkg/kinematics/planar_frame_rotation.hpp"

#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

namespace
{

constexpr double kTolerance = 1e-12;
constexpr double kRightWheelHeading = -M_PI / 3.0;

TEST(PlanarFrameRotation, ZeroOffsetPreservesVelocity)
{
  const Eigen::Vector2d velocity{0.4, -0.2};
  const auto native =
    robot_platform_pkg::base_to_wheel_native(velocity, 0.0);
  EXPECT_NEAR(native.x(), velocity.x(), kTolerance);
  EXPECT_NEAR(native.y(), velocity.y(), kTolerance);
}

TEST(PlanarFrameRotation, RightWheelHeadingRotatesNewForwardMinusSixtyDegrees)
{
  const auto native = robot_platform_pkg::base_to_wheel_native(
    Eigen::Vector2d{1.0, 0.0}, kRightWheelHeading);
  EXPECT_NEAR(native.x(), 0.5, kTolerance);
  EXPECT_NEAR(native.y(), -std::sqrt(3.0) / 2.0, kTolerance);
}

TEST(PlanarFrameRotation, ForwardAndInverseAreExactInverses)
{
  const Eigen::Vector2d base_velocity{0.37, -0.19};
  const auto native = robot_platform_pkg::base_to_wheel_native(
    base_velocity, kRightWheelHeading);
  const auto recovered = robot_platform_pkg::wheel_native_to_base(
    native, kRightWheelHeading);
  EXPECT_NEAR(recovered.x(), base_velocity.x(), kTolerance);
  EXPECT_NEAR(recovered.y(), base_velocity.y(), kTolerance);
}

}  // namespace
