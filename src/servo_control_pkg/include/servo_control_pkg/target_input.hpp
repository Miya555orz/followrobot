#pragma once

#include <cmath>
#include <optional>

#include <vision_servo_msgs/msg/target.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

namespace servo_control_pkg {

inline bool is_actionable_target(const vision_servo_msgs::msg::Target& target)
{
  using Target = vision_servo_msgs::msg::Target;
  return target.visible &&
         (target.tracking_state == Target::TRACKING_STATE_CONFIRMED ||
          target.tracking_state == Target::TRACKING_STATE_UNTRACKED);
}

inline bool has_finite_metric_position(const vision_servo_msgs::msg::Target& target)
{
  return std::isfinite(target.position[0]) &&
         std::isfinite(target.position[1]) &&
         std::isfinite(target.position[2]) &&
         target.position[2] > 0.0F;
}

inline bool is_pbvs_angular_target(
    const vision_servo_msgs::msg::Target& target,
    float min_degraded_confidence,
    float max_prediction_age)
{
  using Target = vision_servo_msgs::msg::Target;
  if (!has_finite_metric_position(target) ||
      !std::isfinite(target.depth_confidence) ||
      !std::isfinite(target.fusion_age)) {
    return false;
  }

  if (target.fusion_state == Target::FUSION_STATE_VALID ||
      target.fusion_state == Target::FUSION_STATE_DEGRADED) {
    return target.visible &&
           target.tracking_state == Target::TRACKING_STATE_CONFIRMED &&
           target.depth_confidence >= min_degraded_confidence;
  }

  return target.fusion_state == Target::FUSION_STATE_PREDICTED &&
         (target.tracking_state == Target::TRACKING_STATE_CONFIRMED ||
          target.tracking_state == Target::TRACKING_STATE_LOST) &&
         target.fusion_age >= 0.0F &&
         target.fusion_age <= max_prediction_age;
}

inline bool is_pbvs_translation_target(
    const vision_servo_msgs::msg::Target& target,
    float min_depth_confidence,
    float max_measurement_age)
{
  using Target = vision_servo_msgs::msg::Target;
  return target.visible &&
         target.tracking_state == Target::TRACKING_STATE_CONFIRMED &&
         target.fusion_state == Target::FUSION_STATE_VALID &&
         has_finite_metric_position(target) &&
         std::isfinite(target.depth_confidence) &&
         target.depth_confidence >= min_depth_confidence &&
         std::isfinite(target.fusion_age) &&
         target.fusion_age >= 0.0F &&
         target.fusion_age <= max_measurement_age;
}

inline std::optional<vision_servo_msgs::msg::Target> select_actionable_target(
    const vision_servo_msgs::msg::TargetArray& targets)
{
  if (targets.tracking_id >= 0) {
    for (const auto& target : targets.targets) {
      if (target.id == targets.tracking_id && is_actionable_target(target)) {
        return target;
      }
    }
    return std::nullopt;
  }

  for (const auto& target : targets.targets) {
    if (is_actionable_target(target)) {
      return target;
    }
  }
  return std::nullopt;
}

inline std::optional<vision_servo_msgs::msg::Target> select_pbvs_target(
    const vision_servo_msgs::msg::TargetArray& targets)
{
  // A locked target may be temporarily invisible while depth fusion publishes
  // a short PREDICTED state. Return the exact locked ID and let the PBVS
  // quality gate decide whether angular-only prediction remains safe.
  if (targets.tracking_id >= 0) {
    for (const auto& target : targets.targets) {
      if (target.id == targets.tracking_id) {
        return target;
      }
    }
    return std::nullopt;
  }
  return select_actionable_target(targets);
}

}  // namespace servo_control_pkg
