/**
 * @file pbvs_controller.cpp
 * @brief Point-position PBVS for person following.
 *
 * A person detector does not provide a rigid-body orientation.  The controller
 * therefore treats the fused target as a 3D point and controls three observable
 * quantities independently:
 *   yaw bearing   = atan2(X, Z)
 *   pitch bearing = atan2(Y, hypot(X, Z))
 *   depth error   = Z - desired_depth
 *
 * Target position is expressed in the Sony optical frame (X right, Y down,
 * Z forward).  ControlAllocator maps the resulting camera twist to the gimbal
 * and planar chassis.
 */

#include "servo_control_pkg/pbvs_controller.hpp"

#include <pluginlib/class_list_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>

namespace servo_control_pkg {

namespace {

double apply_deadband(double value, double deadband)
{
  if (std::abs(value) <= deadband) {
    return 0.0;
  }
  return value - std::copysign(deadband, value);
}

}  // namespace

PBVSController::PBVSController(const rclcpp::NodeOptions& options)
  : ServoControllerBase("pbvs_controller", options),
    translational_gain_(0.6),
    rotational_gain_(0.6),
    lateral_gain_(0.25),
    yaw_deadband_rad_(0.034906585),
    pitch_deadband_rad_(0.034906585),
    depth_deadband_m_(0.20),
    enable_lateral_translation_(false),
    point_goal_set_(false)
{
  declare_parameter("translational_gain", translational_gain_);
  declare_parameter("rotational_gain", rotational_gain_);
  declare_parameter("lateral_gain", lateral_gain_);
  declare_parameter("yaw_deadband_rad", yaw_deadband_rad_);
  declare_parameter("pitch_deadband_rad", pitch_deadband_rad_);
  declare_parameter("depth_deadband_m", depth_deadband_m_);
  declare_parameter("enable_lateral_translation", enable_lateral_translation_);

  translational_gain_ = get_parameter("translational_gain").as_double();
  rotational_gain_ = get_parameter("rotational_gain").as_double();
  lateral_gain_ = get_parameter("lateral_gain").as_double();
  yaw_deadband_rad_ = get_parameter("yaw_deadband_rad").as_double();
  pitch_deadband_rad_ = get_parameter("pitch_deadband_rad").as_double();
  depth_deadband_m_ = get_parameter("depth_deadband_m").as_double();
  enable_lateral_translation_ =
    get_parameter("enable_lateral_translation").as_bool();
}

void PBVSController::configureFromNode(const rclcpp::Node& node)
{
  ServoControllerBase::configureFromNode(node);

  const auto get_double = [&node](const std::string& name, double fallback) {
      double value = fallback;
      if (node.has_parameter(name)) {
        node.get_parameter(name, value);
      }
      return value;
    };
  const auto get_bool = [&node](const std::string& name, bool fallback) {
      bool value = fallback;
      if (node.has_parameter(name)) {
        node.get_parameter(name, value);
      }
      return value;
    };

  translational_gain_ =
    std::max(0.0, get_double("translational_gain", translational_gain_));
  rotational_gain_ =
    std::max(0.0, get_double("rotational_gain", rotational_gain_));
  lateral_gain_ = std::max(0.0, get_double("lateral_gain", lateral_gain_));
  yaw_deadband_rad_ =
    std::max(0.0, get_double("yaw_deadband_rad", yaw_deadband_rad_));
  pitch_deadband_rad_ =
    std::max(0.0, get_double("pitch_deadband_rad", pitch_deadband_rad_));
  depth_deadband_m_ =
    std::max(0.0, get_double("depth_deadband_m", depth_deadband_m_));
  enable_lateral_translation_ =
    get_bool("enable_lateral_translation", enable_lateral_translation_);
}

bool PBVSController::initialize(
    double fx, double fy, double cx, double cy, int width, int height)
{
  return ServoControllerBase::initialize(fx, fy, cx, cy, width, height);
}

bool PBVSController::setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance)
{
  const double x = target.position[0];
  const double y = target.position[1];
  const double z = target.position[2];
  if (!std::isfinite(desired_depth) || desired_depth <= 0.0) {
    RCLCPP_WARN(get_logger(), "PBVS期望距离无效: %.3f", desired_depth);
    return false;
  }
  if (!std::isfinite(x) || !std::isfinite(y) ||
      !std::isfinite(z) || z <= 0.0) {
    RCLCPP_WARN(get_logger(), "PBVS目标三维点无效，拒绝设置目标");
    return false;
  }
  if (!ServoControllerBase::setGoalFromTarget(
      target, desired_depth, feature_tolerance)) {
    return false;
  }

  goal_.desired_depth = desired_depth;
  last_yaw_error_ = std::atan2(x, z);
  last_pitch_error_ = std::atan2(y, std::hypot(x, z));
  last_depth_error_ = z - desired_depth;
  feature_error_.setZero();
  feature_error_(0) = last_yaw_error_;
  feature_error_(1) = last_pitch_error_;
  feature_error_(2) = last_depth_error_;
  feature_error_(3) = x;
  feature_error_(4) = y;
  point_goal_set_ = true;
  return true;
}

std::optional<Eigen::Matrix<double, 6, 1>> PBVSController::computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt)
{
  (void)dt;
  if (!initialized_ || !goal_configured_ || !point_goal_set_) {
    last_camera_velocity_.setZero();
    return std::nullopt;
  }

  const double x = target.position[0];
  const double y = target.position[1];
  const double z = target.position[2];
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || z <= 0.0) {
    last_camera_velocity_.setZero();
    return std::nullopt;
  }

  current_features_ = extractFeatures(target);
  current_depth_ = z;

  last_yaw_error_ = std::atan2(x, z);
  last_pitch_error_ = std::atan2(y, std::hypot(x, z));
  last_depth_error_ = z - goal_.desired_depth;

  // PBVS-specific diagnostic vector.  Do not manufacture a target orientation:
  // a person detector observes a point/bearing and a range, not a rigid pose.
  feature_error_.setZero();
  feature_error_(0) = last_yaw_error_;
  feature_error_(1) = last_pitch_error_;
  feature_error_(2) = last_depth_error_;
  feature_error_(3) = x;
  feature_error_(4) = y;

  Eigen::Matrix<double, 6, 1> velocity =
    Eigen::Matrix<double, 6, 1>::Zero();

  const double yaw_control =
    apply_deadband(last_yaw_error_, yaw_deadband_rad_);
  const double pitch_control =
    apply_deadband(last_pitch_error_, pitch_deadband_rad_);
  const double depth_control =
    apply_deadband(last_depth_error_, depth_deadband_m_);

  // Camera optical twist.  At the neutral mount:
  //   +wy maps to negative gimbal yaw, -wx maps to positive gimbal pitch.
  velocity(3) = -rotational_gain_ * pitch_control;
  velocity(4) = rotational_gain_ * yaw_control;
  velocity(2) = translational_gain_ * depth_control;
  if (enable_lateral_translation_) {
    velocity(0) = lateral_gain_ * x;
  }

  for (int axis = 0; axis < 3; ++axis) {
    velocity(axis) =
      std::clamp(velocity(axis), -max_linear_vel_, max_linear_vel_);
  }
  for (int axis = 3; axis < 6; ++axis) {
    velocity(axis) =
      std::clamp(velocity(axis), -max_angular_vel_, max_angular_vel_);
  }

  if (isConverged()) {
    velocity.setZero();
  }

  ++iteration_count_;
  last_camera_velocity_ = velocity;
  return velocity;
}

bool PBVSController::isConverged() const
{
  return initialized_ && goal_configured_ && point_goal_set_ &&
         std::abs(last_yaw_error_) <= yaw_deadband_rad_ &&
         std::abs(last_pitch_error_) <= pitch_deadband_rad_ &&
         std::abs(last_depth_error_) <= depth_deadband_m_;
}

}  // namespace servo_control_pkg

PLUGINLIB_EXPORT_CLASS(
  servo_control_pkg::PBVSController,
  servo_control_pkg::ServoControllerBase)
