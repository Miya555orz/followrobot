#pragma once

#include <cstdint>

namespace servo_control_pkg {

enum class GimbalHoldReason : std::uint8_t {
  NONE = 0,
  NO_CAMERA_INFO,
  INVALID_OBSERVATION,
  LOW_CONFIDENCE,
  OBSERVATION_STALE,
  PREDICTION_NOT_AUTHORIZED,
  PLATFORM_STALE,
  GIMBAL_DISCONNECTED,
  EMERGENCY_STOP,
  STATIC_HOLD,
};

struct GimbalCameraModel {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double width = 0.0;
  double height = 0.0;

  bool valid() const;
};

struct GimbalVisualObservation {
  double pixel_x = 0.0;
  double pixel_y = 0.0;
  double pixel_velocity_x = 0.0;
  double pixel_velocity_y = 0.0;
  double covariance_x = 0.0;
  double covariance_y = 0.0;
  double confidence = 0.0;
  double capture_time = 0.0;
  double estimate_time = 0.0;
  double receive_time = 0.0;
  bool valid = false;
  bool predicted = false;
};

struct GimbalPlatformSample {
  double yaw = 0.0;
  double pitch = 0.0;
  double source_time = 0.0;
  double receive_time = 0.0;
  bool received = false;
  bool gimbal_connected = false;
  bool emergency_stop = false;
};

struct GimbalVisualServoConfig {
  double desired_u_ratio = 0.50;
  double desired_v_ratio = 0.38;

  double yaw_kp = 2.2;
  double pitch_kp = 1.5;
  double yaw_direction = 1.0;
  double pitch_direction = -1.0;

  double gain_small_scale = 0.45;
  double gain_large_scale = 1.25;
  double gain_small_error_rad = 0.015;
  double gain_large_error_rad = 0.18;

  double max_yaw_rate = 0.35;
  double max_pitch_rate = 0.22;
  double max_yaw_acceleration = 1.8;
  double max_pitch_acceleration = 1.2;
  double min_effective_rate = 0.006;

  double rs2_latency_seconds = 0.05;
  double max_prediction_seconds = 0.15;
  double covariance_reference_px = 20.0;
  double min_confidence = 0.15;

  double normal_age_seconds = 0.08;
  double degraded_age_seconds = 0.18;
  double prediction_age_seconds = 0.30;
  double platform_timeout_seconds = 0.25;

  double static_enter_yaw_rad = 0.006;
  double static_enter_pitch_rad = 0.007;
  double static_exit_yaw_rad = 0.012;
  double static_exit_pitch_rad = 0.014;
  int static_enter_cycles = 5;

  double yaw_soft_limit_rad = 1.40;
  double yaw_limit_margin_rad = 0.08;
  double pitch_min_rad = -0.75;
  double pitch_max_rad = 0.55;
  double pitch_limit_margin_rad = 0.06;
};

struct GimbalVisualServoResult {
  double yaw_rate = 0.0;
  double pitch_rate = 0.0;
  double yaw_error = 0.0;
  double pitch_error = 0.0;
  double predicted_pixel_x = 0.0;
  double predicted_pixel_y = 0.0;
  double observation_age = 0.0;
  double prediction_horizon = 0.0;
  double permission_scale = 0.0;
  bool hold_yaw = true;
  bool hold_pitch = true;
  GimbalHoldReason hold_reason = GimbalHoldReason::INVALID_OBSERVATION;
};

class GimbalVisualServoCore {
public:
  explicit GimbalVisualServoCore(GimbalVisualServoConfig config = {});

  GimbalVisualServoResult update(
      const GimbalVisualObservation& observation,
      const GimbalCameraModel& camera,
      const GimbalPlatformSample& platform,
      double control_time,
      double dt_seconds);

  void reset();
  const GimbalVisualServoConfig& config() const { return config_; }

  static const char* hold_reason_name(GimbalHoldReason reason);

private:
  GimbalVisualServoResult hold(GimbalHoldReason reason, double age = 0.0);
  double scheduled_gain(double error, double base_gain) const;
  static double smoothstep(double edge0, double edge1, double value);
  static double clamp_finite(double value, double low, double high);

  GimbalVisualServoConfig config_;
  double previous_yaw_rate_ = 0.0;
  double previous_pitch_rate_ = 0.0;
  bool static_hold_ = false;
  int static_counter_ = 0;
};

}  // namespace servo_control_pkg
