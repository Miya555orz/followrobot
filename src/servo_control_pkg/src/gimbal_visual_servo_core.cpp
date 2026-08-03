#include "servo_control_pkg/gimbal_visual_servo_core.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace servo_control_pkg {

namespace {

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

bool GimbalCameraModel::valid() const
{
  return finite_positive(fx) && finite_positive(fy) &&
      finite_positive(width) && finite_positive(height) &&
      std::isfinite(cx) && std::isfinite(cy);
}

GimbalVisualServoCore::GimbalVisualServoCore(GimbalVisualServoConfig config)
  : config_(std::move(config))
{
  config_.desired_u_ratio = std::clamp(config_.desired_u_ratio, 0.0, 1.0);
  config_.desired_v_ratio = std::clamp(config_.desired_v_ratio, 0.0, 1.0);
  config_.yaw_kp = std::max(0.0, config_.yaw_kp);
  config_.pitch_kp = std::max(0.0, config_.pitch_kp);
  config_.yaw_direction = config_.yaw_direction < 0.0 ? -1.0 : 1.0;
  config_.pitch_direction = config_.pitch_direction < 0.0 ? -1.0 : 1.0;
  config_.gain_small_scale = std::max(0.0, config_.gain_small_scale);
  config_.gain_large_scale = std::max(
      config_.gain_small_scale, config_.gain_large_scale);
  config_.gain_small_error_rad = std::max(0.0, config_.gain_small_error_rad);
  config_.gain_large_error_rad = std::max(
      config_.gain_small_error_rad + 1e-6, config_.gain_large_error_rad);
  config_.max_yaw_rate = std::abs(config_.max_yaw_rate);
  config_.max_pitch_rate = std::abs(config_.max_pitch_rate);
  config_.max_yaw_acceleration = std::abs(config_.max_yaw_acceleration);
  config_.max_pitch_acceleration = std::abs(config_.max_pitch_acceleration);
  config_.min_effective_rate = std::max(0.0, config_.min_effective_rate);
  config_.rs2_latency_seconds = std::max(0.0, config_.rs2_latency_seconds);
  config_.max_prediction_seconds = std::max(0.0, config_.max_prediction_seconds);
  config_.covariance_reference_px = std::max(
      1.0, config_.covariance_reference_px);
  config_.min_confidence = std::clamp(config_.min_confidence, 0.0, 0.99);
  config_.normal_age_seconds = std::max(0.0, config_.normal_age_seconds);
  config_.degraded_age_seconds = std::max(
      config_.normal_age_seconds, config_.degraded_age_seconds);
  config_.prediction_age_seconds = std::max(
      config_.degraded_age_seconds, config_.prediction_age_seconds);
  config_.platform_timeout_seconds = std::max(
      0.01, config_.platform_timeout_seconds);
  config_.static_enter_yaw_rad = std::max(0.0, config_.static_enter_yaw_rad);
  config_.static_enter_pitch_rad = std::max(0.0, config_.static_enter_pitch_rad);
  config_.static_exit_yaw_rad = std::max(
      config_.static_enter_yaw_rad, config_.static_exit_yaw_rad);
  config_.static_exit_pitch_rad = std::max(
      config_.static_enter_pitch_rad, config_.static_exit_pitch_rad);
  config_.static_enter_cycles = std::max(1, config_.static_enter_cycles);
  config_.yaw_soft_limit_rad = std::abs(config_.yaw_soft_limit_rad);
  config_.yaw_limit_margin_rad = std::clamp(
      std::abs(config_.yaw_limit_margin_rad), 0.0,
      config_.yaw_soft_limit_rad);
  if (config_.pitch_min_rad > config_.pitch_max_rad) {
    std::swap(config_.pitch_min_rad, config_.pitch_max_rad);
  }
  config_.pitch_limit_margin_rad = std::clamp(
      std::abs(config_.pitch_limit_margin_rad), 0.0,
      0.5 * (config_.pitch_max_rad - config_.pitch_min_rad));
}

void GimbalVisualServoCore::reset()
{
  previous_yaw_rate_ = 0.0;
  previous_pitch_rate_ = 0.0;
  static_hold_ = false;
  static_counter_ = 0;
}

GimbalVisualServoResult GimbalVisualServoCore::hold(
    GimbalHoldReason reason, double age)
{
  previous_yaw_rate_ = 0.0;
  previous_pitch_rate_ = 0.0;
  static_counter_ = 0;

  GimbalVisualServoResult result;
  result.observation_age = std::max(0.0, age);
  result.hold_reason = reason;
  return result;
}

double GimbalVisualServoCore::smoothstep(
    double edge0, double edge1, double value)
{
  if (!(edge1 > edge0)) {
    return value >= edge1 ? 1.0 : 0.0;
  }
  const double x = std::clamp((value - edge0) / (edge1 - edge0), 0.0, 1.0);
  return x * x * (3.0 - 2.0 * x);
}

double GimbalVisualServoCore::clamp_finite(
    double value, double low, double high)
{
  return std::isfinite(value) ? std::clamp(value, low, high) : 0.0;
}

double GimbalVisualServoCore::scheduled_gain(
    double error, double base_gain) const
{
  const double blend = smoothstep(
      config_.gain_small_error_rad,
      config_.gain_large_error_rad,
      std::abs(error));
  const double scale = config_.gain_small_scale +
      (config_.gain_large_scale - config_.gain_small_scale) * blend;
  return base_gain * scale;
}

GimbalVisualServoResult GimbalVisualServoCore::update(
    const GimbalVisualObservation& observation,
    const GimbalCameraModel& camera,
    const GimbalPlatformSample& platform,
    double control_time,
    double dt_seconds)
{
  if (!camera.valid()) {
    return hold(GimbalHoldReason::NO_CAMERA_INFO);
  }

  const bool observation_finite =
      std::isfinite(observation.pixel_x) &&
      std::isfinite(observation.pixel_y) &&
      std::isfinite(observation.pixel_velocity_x) &&
      std::isfinite(observation.pixel_velocity_y) &&
      std::isfinite(observation.confidence) &&
      finite_positive(observation.capture_time);
  if (!observation.valid || !observation_finite) {
    return hold(GimbalHoldReason::INVALID_OBSERVATION);
  }
  if (observation.confidence < config_.min_confidence) {
    return hold(GimbalHoldReason::LOW_CONFIDENCE);
  }

  const double observation_age = std::max(
      0.0, control_time - observation.capture_time);
  if (observation_age > config_.prediction_age_seconds) {
    return hold(GimbalHoldReason::OBSERVATION_STALE, observation_age);
  }

  double permission_scale = 1.0;
  if (observation_age > config_.degraded_age_seconds) {
    if (!observation.predicted) {
      return hold(
          GimbalHoldReason::PREDICTION_NOT_AUTHORIZED,
          observation_age);
    }
    permission_scale = 0.6 * std::clamp(
        (config_.prediction_age_seconds - observation_age) /
        std::max(1e-6,
            config_.prediction_age_seconds - config_.degraded_age_seconds),
        0.0, 1.0);
  } else if (observation_age > config_.normal_age_seconds) {
    const double blend = std::clamp(
        (observation_age - config_.normal_age_seconds) /
        std::max(1e-6,
            config_.degraded_age_seconds - config_.normal_age_seconds),
        0.0, 1.0);
    permission_scale = 1.0 - 0.4 * smoothstep(0.0, 1.0, blend);
  }

  if (!platform.received) {
    return hold(GimbalHoldReason::PLATFORM_STALE, observation_age);
  }
  const double receive_age = std::max(0.0, control_time - platform.receive_time);
  const double source_age = finite_positive(platform.source_time)
      ? std::max(0.0, control_time - platform.source_time)
      : receive_age;
  if (std::max(receive_age, source_age) > config_.platform_timeout_seconds) {
    return hold(GimbalHoldReason::PLATFORM_STALE, observation_age);
  }
  if (platform.emergency_stop) {
    return hold(GimbalHoldReason::EMERGENCY_STOP, observation_age);
  }
  if (!platform.gimbal_connected) {
    return hold(GimbalHoldReason::GIMBAL_DISCONNECTED, observation_age);
  }

  const double confidence_factor = std::clamp(
      (observation.confidence - config_.min_confidence) /
      std::max(1e-6, 1.0 - config_.min_confidence),
      0.0, 1.0);
  const double covariance_px = std::sqrt(std::max(
      0.0, std::max(observation.covariance_x, observation.covariance_y)));
  const double covariance_factor = 1.0 /
      (1.0 + covariance_px / std::max(1.0, config_.covariance_reference_px));
  const double prediction_quality = permission_scale *
      (0.45 + 0.55 * confidence_factor) * covariance_factor;
  const double state_age = finite_positive(observation.estimate_time)
      ? std::max(0.0, control_time - observation.estimate_time)
      : observation_age;
  const double prediction_horizon = std::clamp(
      (state_age + config_.rs2_latency_seconds) * prediction_quality,
      0.0, config_.max_prediction_seconds);

  const double predicted_u = std::clamp(
      observation.pixel_x + observation.pixel_velocity_x * prediction_horizon,
      -0.25 * camera.width, 1.25 * camera.width);
  const double predicted_v = std::clamp(
      observation.pixel_y + observation.pixel_velocity_y * prediction_horizon,
      -0.25 * camera.height, 1.25 * camera.height);
  const double desired_u = config_.desired_u_ratio * camera.width;
  const double desired_v = config_.desired_v_ratio * camera.height;

  const double x = (predicted_u - camera.cx) / camera.fx;
  const double y = (predicted_v - camera.cy) / camera.fy;
  const double desired_x = (desired_u - camera.cx) / camera.fx;
  const double desired_y = (desired_v - camera.cy) / camera.fy;
  const double yaw_error = std::atan(x) - std::atan(desired_x);
  const double pitch_error =
      std::atan2(y, std::sqrt(1.0 + x * x)) -
      std::atan2(desired_y, std::sqrt(1.0 + desired_x * desired_x));

  if (static_hold_) {
    if (std::abs(yaw_error) > config_.static_exit_yaw_rad ||
        std::abs(pitch_error) > config_.static_exit_pitch_rad) {
      static_hold_ = false;
      static_counter_ = 0;
    }
  } else if (std::abs(yaw_error) < config_.static_enter_yaw_rad &&
             std::abs(pitch_error) < config_.static_enter_pitch_rad) {
    ++static_counter_;
    if (static_counter_ >= std::max(1, config_.static_enter_cycles)) {
      static_hold_ = true;
    }
  } else {
    static_counter_ = 0;
  }

  if (static_hold_) {
    auto result = hold(GimbalHoldReason::STATIC_HOLD, observation_age);
    result.yaw_error = yaw_error;
    result.pitch_error = pitch_error;
    result.predicted_pixel_x = predicted_u;
    result.predicted_pixel_y = predicted_v;
    result.prediction_horizon = prediction_horizon;
    result.permission_scale = permission_scale;
    static_hold_ = true;
    return result;
  }

  const double yaw_limit = config_.max_yaw_rate * permission_scale;
  const double pitch_limit = config_.max_pitch_rate * permission_scale;
  double yaw_rate = config_.yaw_direction *
      scheduled_gain(yaw_error, config_.yaw_kp) * yaw_error;
  double pitch_rate = config_.pitch_direction *
      scheduled_gain(pitch_error, config_.pitch_kp) * pitch_error;
  yaw_rate = clamp_finite(yaw_rate, -yaw_limit, yaw_limit);
  pitch_rate = clamp_finite(pitch_rate, -pitch_limit, pitch_limit);

  const double dt = std::clamp(dt_seconds, 1e-4, 0.10);
  yaw_rate = std::clamp(
      yaw_rate,
      previous_yaw_rate_ - config_.max_yaw_acceleration * dt,
      previous_yaw_rate_ + config_.max_yaw_acceleration * dt);
  pitch_rate = std::clamp(
      pitch_rate,
      previous_pitch_rate_ - config_.max_pitch_acceleration * dt,
      previous_pitch_rate_ + config_.max_pitch_acceleration * dt);

  const double yaw_guard = std::max(
      0.0, config_.yaw_soft_limit_rad - config_.yaw_limit_margin_rad);
  if ((platform.yaw >= yaw_guard && yaw_rate > 0.0) ||
      (platform.yaw <= -yaw_guard && yaw_rate < 0.0)) {
    yaw_rate = 0.0;
  }
  if ((platform.pitch >= config_.pitch_max_rad - config_.pitch_limit_margin_rad &&
       pitch_rate > 0.0) ||
      (platform.pitch <= config_.pitch_min_rad + config_.pitch_limit_margin_rad &&
       pitch_rate < 0.0)) {
    pitch_rate = 0.0;
  }

  if (std::abs(yaw_error) < config_.static_enter_yaw_rad &&
      std::abs(yaw_rate) < config_.min_effective_rate) {
    yaw_rate = 0.0;
  }
  if (std::abs(pitch_error) < config_.static_enter_pitch_rad &&
      std::abs(pitch_rate) < config_.min_effective_rate) {
    pitch_rate = 0.0;
  }

  previous_yaw_rate_ = yaw_rate;
  previous_pitch_rate_ = pitch_rate;

  GimbalVisualServoResult result;
  result.yaw_rate = yaw_rate;
  result.pitch_rate = pitch_rate;
  result.yaw_error = yaw_error;
  result.pitch_error = pitch_error;
  result.predicted_pixel_x = predicted_u;
  result.predicted_pixel_y = predicted_v;
  result.observation_age = observation_age;
  result.prediction_horizon = prediction_horizon;
  result.permission_scale = permission_scale;
  result.hold_yaw = std::abs(yaw_rate) < 1e-9;
  result.hold_pitch = std::abs(pitch_rate) < 1e-9;
  result.hold_reason = GimbalHoldReason::NONE;
  return result;
}

const char* GimbalVisualServoCore::hold_reason_name(GimbalHoldReason reason)
{
  switch (reason) {
    case GimbalHoldReason::NONE: return "TRACKING";
    case GimbalHoldReason::NO_CAMERA_INFO: return "NO_CAMERA_INFO";
    case GimbalHoldReason::INVALID_OBSERVATION: return "INVALID_OBSERVATION";
    case GimbalHoldReason::LOW_CONFIDENCE: return "LOW_CONFIDENCE";
    case GimbalHoldReason::OBSERVATION_STALE: return "OBSERVATION_STALE";
    case GimbalHoldReason::PREDICTION_NOT_AUTHORIZED: return "PREDICTION_NOT_AUTHORIZED";
    case GimbalHoldReason::PLATFORM_STALE: return "PLATFORM_STALE";
    case GimbalHoldReason::GIMBAL_DISCONNECTED: return "GIMBAL_DISCONNECTED";
    case GimbalHoldReason::EMERGENCY_STOP: return "EMERGENCY_STOP";
    case GimbalHoldReason::STATIC_HOLD: return "STATIC_HOLD";
  }
  return "UNKNOWN";
}

}  // namespace servo_control_pkg
