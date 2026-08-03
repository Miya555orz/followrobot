#include "servo_control_pkg/gimbal_visual_servo_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <utility>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_servo_msgs/msg/aim_target2_d.hpp>
#include <vision_servo_msgs/msg/gimbal_cmd.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>

namespace servo_control_pkg {

class GimbalVisualServoNode : public rclcpp::Node {
public:
  explicit GimbalVisualServoNode(
      const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
    : Node("gimbal_visual_servo", options),
      core_(load_config())
  {
    control_rate_hz_ = declare_parameter<double>("control_rate_hz", 50.0);
    diagnostics_rate_hz_ = declare_parameter<double>("diagnostics_rate_hz", 5.0);
    control_rate_hz_ = std::clamp(control_rate_hz_, 10.0, 200.0);
    diagnostics_rate_hz_ = std::clamp(diagnostics_rate_hz_, 0.5, 20.0);

    const auto latest_sensor_qos = rclcpp::SensorDataQoS().keep_last(1);
    const auto latest_control_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    const auto platform_qos =
        rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();

    aim_sub_ = create_subscription<vision_servo_msgs::msg::AimTarget2D>(
        "/perception/aim_target_2d", latest_sensor_qos,
        std::bind(
            &GimbalVisualServoNode::aim_callback, this,
            std::placeholders::_1));
    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        "/sony/camera_info", latest_sensor_qos,
        std::bind(
            &GimbalVisualServoNode::camera_info_callback, this,
            std::placeholders::_1));
    platform_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
        "/platform/state", platform_qos,
        std::bind(
            &GimbalVisualServoNode::platform_callback, this,
            std::placeholders::_1));

    command_pub_ = create_publisher<vision_servo_msgs::msg::GimbalCmd>(
        "/auto/cmd_gimbal", latest_control_qos);
    diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", rclcpp::QoS(10).reliable());

    last_control_steady_ = std::chrono::steady_clock::now();
    control_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / control_rate_hz_)),
        std::bind(&GimbalVisualServoNode::control_tick, this));
    diagnostics_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / diagnostics_rate_hz_)),
        std::bind(&GimbalVisualServoNode::publish_diagnostics, this));

    RCLCPP_INFO(
        get_logger(),
        "二维云台快环已启动: control=%.1fHz desired=(%.2f, %.2f) "
        "yaw_sign=%.0f pitch_sign=%.0f",
        control_rate_hz_, core_.config().desired_u_ratio,
        core_.config().desired_v_ratio, core_.config().yaw_direction,
        core_.config().pitch_direction);
  }

private:
  GimbalVisualServoConfig load_config()
  {
    GimbalVisualServoConfig config;
    config.desired_u_ratio = declare_parameter<double>("desired_u_ratio", 0.50);
    config.desired_v_ratio = declare_parameter<double>("desired_v_ratio", 0.38);
    config.yaw_kp = declare_parameter<double>("yaw_kp", 2.2);
    config.pitch_kp = declare_parameter<double>("pitch_kp", 1.5);
    config.yaw_direction = declare_parameter<double>("yaw_direction", 1.0);
    config.pitch_direction = declare_parameter<double>("pitch_direction", -1.0);
    config.gain_small_scale = declare_parameter<double>("gain_small_scale", 0.45);
    config.gain_large_scale = declare_parameter<double>("gain_large_scale", 1.25);
    config.gain_small_error_rad =
        declare_parameter<double>("gain_small_error_rad", 0.015);
    config.gain_large_error_rad =
        declare_parameter<double>("gain_large_error_rad", 0.18);
    config.max_yaw_rate = declare_parameter<double>("max_yaw_rate", 0.35);
    config.max_pitch_rate = declare_parameter<double>("max_pitch_rate", 0.22);
    config.max_yaw_acceleration =
        declare_parameter<double>("max_yaw_acceleration", 1.8);
    config.max_pitch_acceleration =
        declare_parameter<double>("max_pitch_acceleration", 1.2);
    config.min_effective_rate =
        declare_parameter<double>("min_effective_rate", 0.006);
    config.rs2_latency_seconds =
        declare_parameter<double>("rs2_latency_seconds", 0.05);
    config.max_prediction_seconds =
        declare_parameter<double>("max_prediction_seconds", 0.15);
    config.covariance_reference_px =
        declare_parameter<double>("covariance_reference_px", 20.0);
    config.min_confidence = declare_parameter<double>("min_confidence", 0.15);
    config.normal_age_seconds =
        declare_parameter<double>("normal_age_seconds", 0.08);
    config.degraded_age_seconds =
        declare_parameter<double>("degraded_age_seconds", 0.18);
    config.prediction_age_seconds =
        declare_parameter<double>("prediction_age_seconds", 0.30);
    config.platform_timeout_seconds =
        declare_parameter<double>("platform_timeout_seconds", 0.25);
    config.static_enter_yaw_rad =
        declare_parameter<double>("static_enter_yaw_rad", 0.006);
    config.static_enter_pitch_rad =
        declare_parameter<double>("static_enter_pitch_rad", 0.007);
    config.static_exit_yaw_rad =
        declare_parameter<double>("static_exit_yaw_rad", 0.012);
    config.static_exit_pitch_rad =
        declare_parameter<double>("static_exit_pitch_rad", 0.014);
    config.static_enter_cycles =
        declare_parameter<int>("static_enter_cycles", 5);
    config.yaw_soft_limit_rad =
        declare_parameter<double>("yaw_soft_limit_rad", 1.40);
    config.yaw_limit_margin_rad =
        declare_parameter<double>("yaw_limit_margin_rad", 0.08);
    config.pitch_min_rad = declare_parameter<double>("pitch_min_rad", -0.75);
    config.pitch_max_rad = declare_parameter<double>("pitch_max_rad", 0.55);
    config.pitch_limit_margin_rad =
        declare_parameter<double>("pitch_limit_margin_rad", 0.06);
    return config;
  }

  static double stamp_seconds(const builtin_interfaces::msg::Time& stamp)
  {
    return static_cast<double>(stamp.sec) +
        static_cast<double>(stamp.nanosec) * 1e-9;
  }

  void aim_callback(
      const vision_servo_msgs::msg::AimTarget2D::ConstSharedPtr msg)
  {
    GimbalVisualObservation observation;
    observation.pixel_x = msg->pixel_x;
    observation.pixel_y = msg->pixel_y;
    observation.pixel_velocity_x = msg->pixel_velocity_x;
    observation.pixel_velocity_y = msg->pixel_velocity_y;
    observation.covariance_x = msg->covariance_x;
    observation.covariance_y = msg->covariance_y;
    observation.confidence = msg->confidence;
    observation.capture_time = stamp_seconds(msg->header.stamp);
    observation.estimate_time = stamp_seconds(msg->estimate_stamp);
    observation.receive_time = now().seconds();
    observation.valid = msg->valid;
    observation.predicted = msg->predicted ||
        msg->source == vision_servo_msgs::msg::AimTarget2D::PREDICTED ||
        msg->source == vision_servo_msgs::msg::AimTarget2D::PREDICTED_FACE ||
        msg->source == vision_servo_msgs::msg::AimTarget2D::LOST_PREDICTION;

    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_observation_ = observation;
    latest_aim_frame_ = msg->header.frame_id;
    latest_source_stamp_ = msg->header.stamp;
  }

  void camera_info_callback(
      const sensor_msgs::msg::CameraInfo::ConstSharedPtr msg)
  {
    GimbalCameraModel camera;
    camera.fx = msg->k[0];
    camera.fy = msg->k[4];
    camera.cx = msg->k[2];
    camera.cy = msg->k[5];
    camera.width = static_cast<double>(msg->width);
    camera.height = static_cast<double>(msg->height);

    std::lock_guard<std::mutex> lock(data_mutex_);
    camera_ = camera;
  }

  void platform_callback(
      const vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg)
  {
    GimbalPlatformSample platform;
    platform.yaw = msg->gimbal_yaw;
    platform.pitch = msg->gimbal_pitch;
    platform.source_time = stamp_seconds(msg->header.stamp);
    platform.receive_time = now().seconds();
    platform.received = true;
    platform.gimbal_connected = msg->gimbal_connected;
    platform.emergency_stop = msg->emergency_stop;

    std::lock_guard<std::mutex> lock(data_mutex_);
    platform_ = platform;
  }

  void control_tick()
  {
    const auto steady_now = std::chrono::steady_clock::now();
    const double dt = std::chrono::duration<double>(
        steady_now - last_control_steady_).count();
    last_control_steady_ = steady_now;
    const auto control_stamp = now();

    GimbalVisualObservation observation;
    GimbalCameraModel camera;
    GimbalPlatformSample platform;
    std::string frame_id;
    builtin_interfaces::msg::Time source_stamp;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      observation = latest_observation_;
      camera = camera_;
      platform = platform_;
      frame_id = latest_aim_frame_;
      source_stamp = latest_source_stamp_;
    }

    const auto result = core_.update(
        observation, camera, platform, control_stamp.seconds(), dt);

    vision_servo_msgs::msg::GimbalCmd command;
    command.header.stamp = control_stamp;
    command.header.frame_id = frame_id.empty()
        ? "sony_camera_optical_frame" : frame_id;
    command.source_stamp = source_stamp;
    command.control_stamp = control_stamp;
    command.yaw_rate = static_cast<float>(result.yaw_rate);
    command.pitch_rate = static_cast<float>(result.pitch_rate);
    command.hold_yaw = result.hold_yaw;
    command.hold_pitch = result.hold_pitch;
    command_pub_->publish(command);

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      last_result_ = result;
      last_control_stamp_ = control_stamp.seconds();
      last_aim_to_control_ms_ = observation.receive_time > 0.0
          ? (control_stamp.seconds() - observation.receive_time) * 1000.0
          : -1.0;
      last_capture_to_control_ms_ = observation.capture_time > 0.0
          ? (control_stamp.seconds() - observation.capture_time) * 1000.0
          : -1.0;
    }

    if (result.hold_reason != GimbalHoldReason::NONE &&
        result.hold_reason != GimbalHoldReason::STATIC_HOLD) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "二维云台快环处于 HOLD: %s (age=%.1fms)",
          GimbalVisualServoCore::hold_reason_name(result.hold_reason),
          result.observation_age * 1000.0);
    }
  }

  static void add_key(
      diagnostic_msgs::msg::DiagnosticStatus& status,
      const std::string& key,
      double value)
  {
    diagnostic_msgs::msg::KeyValue item;
    item.key = key;
    item.value = std::to_string(value);
    status.values.push_back(std::move(item));
  }

  void publish_diagnostics()
  {
    GimbalVisualServoResult result;
    double aim_to_control_ms = -1.0;
    double capture_to_control_ms = -1.0;
    double control_stamp = 0.0;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      result = last_result_;
      aim_to_control_ms = last_aim_to_control_ms_;
      capture_to_control_ms = last_capture_to_control_ms_;
      control_stamp = last_control_stamp_;
    }

    diagnostic_msgs::msg::DiagnosticArray array;
    array.header.stamp = now();
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = "gimbal_visual_servo: fast_2d_loop";
    status.hardware_id = "sony_rs2_visual_servo";
    status.message = GimbalVisualServoCore::hold_reason_name(result.hold_reason);
    if (result.hold_reason == GimbalHoldReason::NONE ||
        result.hold_reason == GimbalHoldReason::STATIC_HOLD) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    } else if (result.hold_reason == GimbalHoldReason::EMERGENCY_STOP ||
               result.hold_reason == GimbalHoldReason::GIMBAL_DISCONNECTED ||
               result.hold_reason == GimbalHoldReason::PLATFORM_STALE) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    }

    add_key(status, "capture_to_control_ms", capture_to_control_ms);
    add_key(status, "aim_receive_to_control_ms", aim_to_control_ms);
    add_key(status, "observation_age_ms", result.observation_age * 1000.0);
    add_key(status, "prediction_horizon_ms", result.prediction_horizon * 1000.0);
    add_key(status, "permission_scale", result.permission_scale);
    add_key(status, "yaw_error_rad", result.yaw_error);
    add_key(status, "pitch_error_rad", result.pitch_error);
    add_key(status, "yaw_rate_cmd", result.yaw_rate);
    add_key(status, "pitch_rate_cmd", result.pitch_rate);
    add_key(status, "predicted_pixel_x", result.predicted_pixel_x);
    add_key(status, "predicted_pixel_y", result.predicted_pixel_y);
    add_key(status, "last_control_stamp", control_stamp);
    array.status.push_back(std::move(status));
    diagnostics_pub_->publish(array);
  }

  GimbalVisualServoCore core_;
  double control_rate_hz_ = 50.0;
  double diagnostics_rate_hz_ = 5.0;

  std::mutex data_mutex_;
  GimbalVisualObservation latest_observation_;
  GimbalCameraModel camera_;
  GimbalPlatformSample platform_;
  GimbalVisualServoResult last_result_;
  std::string latest_aim_frame_;
  builtin_interfaces::msg::Time latest_source_stamp_;
  double last_control_stamp_ = 0.0;
  double last_aim_to_control_ms_ = -1.0;
  double last_capture_to_control_ms_ = -1.0;
  std::chrono::steady_clock::time_point last_control_steady_;

  rclcpp::Subscription<vision_servo_msgs::msg::AimTarget2D>::SharedPtr aim_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalCmd>::SharedPtr command_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
};

}  // namespace servo_control_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_control_pkg::GimbalVisualServoNode>());
  rclcpp::shutdown();
  return 0;
}
