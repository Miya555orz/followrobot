#include <cmath>

#include <gtest/gtest.h>

#include "servo_control_pkg/gimbal_visual_servo_core.hpp"

namespace {

servo_control_pkg::GimbalCameraModel camera()
{
  return {700.0, 700.0, 512.0, 340.0, 1024.0, 680.0};
}

servo_control_pkg::GimbalVisualObservation observation(double now)
{
  servo_control_pkg::GimbalVisualObservation value;
  value.pixel_x = 700.0;
  value.pixel_y = 258.4;  // desired_v_ratio=0.38
  value.covariance_x = 1.0;
  value.covariance_y = 1.0;
  value.confidence = 0.9;
  value.capture_time = now - 0.03;
  value.estimate_time = now - 0.01;
  value.receive_time = now - 0.005;
  value.valid = true;
  return value;
}

servo_control_pkg::GimbalPlatformSample platform(double now)
{
  servo_control_pkg::GimbalPlatformSample value;
  value.source_time = now - 0.01;
  value.receive_time = now - 0.005;
  value.received = true;
  value.gimbal_connected = true;
  return value;
}

servo_control_pkg::GimbalVisualServoConfig config()
{
  servo_control_pkg::GimbalVisualServoConfig value;
  value.max_yaw_acceleration = 100.0;
  value.max_pitch_acceleration = 100.0;
  return value;
}

}  // namespace

TEST(GimbalVisualServoCore, UsesCalibratedDirectionForRightSideTarget)
{
  constexpr double now = 10.0;
  servo_control_pkg::GimbalVisualServoCore core(config());
  const auto result = core.update(
      observation(now), camera(), platform(now), now, 0.02);
  EXPECT_EQ(result.hold_reason, servo_control_pkg::GimbalHoldReason::NONE);
  EXPECT_GT(result.yaw_error, 0.0);
  EXPECT_GT(result.yaw_rate, 0.0);
  EXPECT_NEAR(result.pitch_rate, 0.0, 1e-6);
}

TEST(GimbalVisualServoCore, SupportsNonCentralCompositionPoint)
{
  constexpr double now = 10.0;
  auto cfg = config();
  cfg.desired_u_ratio = 0.25;
  cfg.desired_v_ratio = 0.30;
  auto obs = observation(now);
  obs.pixel_x = 0.25 * camera().width;
  obs.pixel_y = 0.30 * camera().height;
  servo_control_pkg::GimbalVisualServoCore core(cfg);
  const auto result = core.update(obs, camera(), platform(now), now, 0.02);
  EXPECT_NEAR(result.yaw_error, 0.0, 1e-9);
  EXPECT_NEAR(result.pitch_error, 0.0, 1e-9);
}

TEST(GimbalVisualServoCore, RejectsOldMeasuredObservation)
{
  constexpr double now = 10.0;
  auto obs = observation(now);
  obs.capture_time = now - 0.22;
  obs.predicted = false;
  servo_control_pkg::GimbalVisualServoCore core(config());
  const auto result = core.update(obs, camera(), platform(now), now, 0.02);
  EXPECT_EQ(
      result.hold_reason,
      servo_control_pkg::GimbalHoldReason::PREDICTION_NOT_AUTHORIZED);
  EXPECT_TRUE(result.hold_yaw);
}

TEST(GimbalVisualServoCore, AllowsFiniteExplicitPredictionThenExpires)
{
  constexpr double now = 10.0;
  auto obs = observation(now);
  obs.capture_time = now - 0.22;
  obs.predicted = true;
  servo_control_pkg::GimbalVisualServoCore core(config());
  auto result = core.update(obs, camera(), platform(now), now, 0.02);
  EXPECT_EQ(result.hold_reason, servo_control_pkg::GimbalHoldReason::NONE);
  EXPECT_GT(result.permission_scale, 0.0);
  EXPECT_LE(result.prediction_horizon, config().max_prediction_seconds);

  obs.capture_time = now - 0.31;
  result = core.update(obs, camera(), platform(now), now, 0.02);
  EXPECT_EQ(
      result.hold_reason,
      servo_control_pkg::GimbalHoldReason::OBSERVATION_STALE);
}

TEST(GimbalVisualServoCore, HoldsForDisconnectedGimbal)
{
  constexpr double now = 10.0;
  auto state = platform(now);
  state.gimbal_connected = false;
  servo_control_pkg::GimbalVisualServoCore core(config());
  const auto result = core.update(
      observation(now), camera(), state, now, 0.02);
  EXPECT_EQ(
      result.hold_reason,
      servo_control_pkg::GimbalHoldReason::GIMBAL_DISCONNECTED);
}

TEST(GimbalVisualServoCore, SoftLimitOnlyBlocksMotionFurtherIntoLimit)
{
  constexpr double now = 10.0;
  auto state = platform(now);
  state.yaw = config().yaw_soft_limit_rad;
  servo_control_pkg::GimbalVisualServoCore core(config());
  auto result = core.update(observation(now), camera(), state, now, 0.02);
  EXPECT_DOUBLE_EQ(result.yaw_rate, 0.0);

  auto obs = observation(now);
  obs.pixel_x = 200.0;
  result = core.update(obs, camera(), state, now, 0.02);
  EXPECT_LT(result.yaw_rate, 0.0);
}

TEST(GimbalVisualServoCore, LimitsAngularAcceleration)
{
  constexpr double now = 10.0;
  auto cfg = config();
  cfg.max_yaw_acceleration = 0.5;
  servo_control_pkg::GimbalVisualServoCore core(cfg);
  const auto result = core.update(
      observation(now), camera(), platform(now), now, 0.02);
  EXPECT_LE(std::abs(result.yaw_rate), 0.0100001);
}
