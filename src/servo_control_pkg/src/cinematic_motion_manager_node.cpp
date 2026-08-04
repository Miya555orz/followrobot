/**
 * @file cinematic_motion_manager_node.cpp
 * @brief Cancelable STATIC/DOLLY/TRUCK/ORBIT task and reference generator.
 *
 * This node never publishes actuator commands. It publishes a bounded dynamic
 * reference consumed by servo_manager, preserving command_mux as the only
 * final command owner.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <vision_servo_msgs/action/cinematic_move.hpp>
#include <vision_servo_msgs/msg/cinematic_reference.hpp>
#include <vision_servo_msgs/msg/platform_state.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace servo_control_pkg {

namespace {

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

bool finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

}  // namespace

class CinematicMotionManagerNode : public rclcpp::Node
{
public:
  using CinematicMove = vision_servo_msgs::action::CinematicMove;
  using GoalHandle = rclcpp_action::ServerGoalHandle<CinematicMove>;
  using Target = vision_servo_msgs::msg::Target;
  using TargetArray = vision_servo_msgs::msg::TargetArray;
  using Reference = vision_servo_msgs::msg::CinematicReference;

  CinematicMotionManagerNode()
  : Node("cinematic_motion_manager")
  {
    declare_parameter("control_rate_hz", 20.0);
    declare_parameter("target_timeout_sec", 0.35);
    declare_parameter("target_lost_abort_sec", 1.0);
    declare_parameter("depth_abort_sec", 0.60);
    declare_parameter("alignment_timeout_sec", 3.0);
    declare_parameter("alignment_bearing_tolerance_deg", 5.0);
    declare_parameter("min_valid_depth_confidence", 0.65);
    declare_parameter("min_degraded_depth_confidence", 0.40);
    declare_parameter("default_target_distance_m", 2.0);
    declare_parameter("default_max_speed_mps", 0.10);
    declare_parameter("max_linear_speed_mps", 0.20);
    declare_parameter("max_angular_speed_rad_s", 0.25);
    declare_parameter("linear_acceleration_mps2", 0.20);
    declare_parameter("distance_tolerance_m", 0.10);
    declare_parameter("orbit_radius_tolerance_m", 0.15);
    declare_parameter("orbit_angle_tolerance_deg", 3.0);
    declare_parameter("orbit_radial_gain", 0.45);
    declare_parameter("orbit_facing_gain", 0.50);
    declare_parameter("camera_mount_yaw_offset_rad", 0.0);
    declare_parameter("target_world_filter_alpha", 0.08);
    declare_parameter("completion_hold_sec", 0.40);
    declare_parameter("max_task_duration_sec", 120.0);
    declare_parameter("max_displacement_m", 2.0);
    declare_parameter("max_orbit_angle_deg", 180.0);
    declare_parameter("min_target_distance_m", 0.60);
    declare_parameter("max_target_distance_m", 5.0);

    control_rate_hz_ = std::clamp(get_parameter("control_rate_hz").as_double(), 5.0, 50.0);
    target_timeout_sec_ = std::max(0.05, get_parameter("target_timeout_sec").as_double());
    target_lost_abort_sec_ = std::max(
      target_timeout_sec_, get_parameter("target_lost_abort_sec").as_double());
    depth_abort_sec_ = std::max(0.10, get_parameter("depth_abort_sec").as_double());
    alignment_timeout_sec_ = std::max(
      0.5, get_parameter("alignment_timeout_sec").as_double());
    alignment_bearing_tolerance_rad_ = std::max(
      0.5, get_parameter("alignment_bearing_tolerance_deg").as_double()) *
      M_PI / 180.0;
    min_valid_depth_confidence_ = std::clamp(
      get_parameter("min_valid_depth_confidence").as_double(), 0.0, 1.0);
    min_degraded_depth_confidence_ = std::clamp(
      get_parameter("min_degraded_depth_confidence").as_double(), 0.0, 1.0);
    default_target_distance_m_ = std::max(
      0.30, get_parameter("default_target_distance_m").as_double());
    default_max_speed_mps_ = std::max(
      0.01, get_parameter("default_max_speed_mps").as_double());
    max_linear_speed_mps_ = std::max(
      0.01, get_parameter("max_linear_speed_mps").as_double());
    max_angular_speed_rad_s_ = std::max(
      0.01, get_parameter("max_angular_speed_rad_s").as_double());
    linear_acceleration_mps2_ = std::max(
      0.01, get_parameter("linear_acceleration_mps2").as_double());
    distance_tolerance_m_ = std::max(
      0.02, get_parameter("distance_tolerance_m").as_double());
    orbit_radius_tolerance_m_ = std::max(
      0.03, get_parameter("orbit_radius_tolerance_m").as_double());
    orbit_angle_tolerance_rad_ = std::max(
      0.5, get_parameter("orbit_angle_tolerance_deg").as_double()) * M_PI / 180.0;
    orbit_radial_gain_ = std::max(0.0, get_parameter("orbit_radial_gain").as_double());
    orbit_facing_gain_ = std::max(0.0, get_parameter("orbit_facing_gain").as_double());
    camera_mount_yaw_offset_rad_ = get_parameter("camera_mount_yaw_offset_rad").as_double();
    target_world_filter_alpha_ = std::clamp(
      get_parameter("target_world_filter_alpha").as_double(), 0.0, 1.0);
    completion_hold_sec_ = std::max(0.0, get_parameter("completion_hold_sec").as_double());
    max_task_duration_sec_ = std::max(
      1.0, get_parameter("max_task_duration_sec").as_double());
    max_displacement_m_ = std::max(
      0.1, get_parameter("max_displacement_m").as_double());
    max_orbit_angle_deg_ = std::clamp(
      get_parameter("max_orbit_angle_deg").as_double(), 1.0, 360.0);
    min_target_distance_m_ = std::max(
      0.1, get_parameter("min_target_distance_m").as_double());
    max_target_distance_m_ = std::max(
      min_target_distance_m_, get_parameter("max_target_distance_m").as_double());

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    reference_pub_ = create_publisher<Reference>("/cinematic/reference", command_qos);
    tracks_sub_ = create_subscription<TargetArray>(
      "/perception/tracks", command_qos,
      [this](TargetArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_tracks_ = msg;
        tracks_receive_time_ = now();
      });
    targets_3d_sub_ = create_subscription<TargetArray>(
      "/perception/targets_3d", command_qos,
      [this](TargetArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        latest_targets_3d_ = msg;
        targets_3d_receive_time_ = now();
      });
    platform_sub_ = create_subscription<vision_servo_msgs::msg::PlatformState>(
      "/platform/state", rclcpp::QoS(rclcpp::KeepLast(1)).reliable(),
      [this](vision_servo_msgs::msg::PlatformState::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        platform_state_ = *msg;
        platform_receive_time_ = now();
      });

    action_server_ = rclcpp_action::create_server<CinematicMove>(
      this, "/cinematic/execute",
      std::bind(
        &CinematicMotionManagerNode::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(
        &CinematicMotionManagerNode::handle_cancel, this,
        std::placeholders::_1),
      std::bind(
        &CinematicMotionManagerNode::handle_accepted, this,
        std::placeholders::_1));

    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/cinematic/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_goal_) {
          response->success = true;
          response->message = "没有活动运镜任务";
          return;
        }
        publish_reference(stopped_reference(now()));
        finish_goal(CinematicMove::Result::RESULT_CANCELED, "运镜任务被停止服务终止");
        response->success = true;
        response->message = "已停止运镜任务";
      });

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz_)),
      std::bind(&CinematicMotionManagerNode::control_tick, this));

    RCLCPP_INFO(
      get_logger(),
      "运镜任务管理器已启动 (%.1fHz): STATIC/DOLLY/TRUCK/ORBIT",
      control_rate_hz_);
  }

private:
  enum class Phase : uint8_t
  {
    Idle = CinematicMove::Feedback::STATE_IDLE,
    Acquire = CinematicMove::Feedback::STATE_ACQUIRE,
    Align = CinematicMove::Feedback::STATE_ALIGN,
    Execute = CinematicMove::Feedback::STATE_EXECUTE,
    Hold = CinematicMove::Feedback::STATE_HOLD,
    Recover = CinematicMove::Feedback::STATE_RECOVER,
  };

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const CinematicMove::Goal> goal)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_goal_) {
      RCLCPP_WARN(get_logger(), "已有运镜任务，拒绝新任务");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->mode > CinematicMove::Goal::ORBIT_ARC ||
        goal->direction < CinematicMove::Goal::DIRECTION_RIGHT ||
        goal->direction > CinematicMove::Goal::DIRECTION_LEFT ||
        !std::isfinite(goal->max_speed) || goal->max_speed < 0.0F ||
        !std::isfinite(goal->duration_sec) || goal->duration_sec < 0.0F ||
        goal->duration_sec > max_task_duration_sec_) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->mode != CinematicMove::Goal::STATIC_TRACK &&
        goal->duration_sec <= 0.0F) {
      RCLCPP_WARN(get_logger(), "动态运镜必须设置正的 duration_sec");
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->mode == CinematicMove::Goal::DOLLY_IN_OUT &&
        (!finite_positive(goal->target_distance_m) ||
         goal->target_distance_m < min_target_distance_m_ ||
         goal->target_distance_m > max_target_distance_m_)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->mode == CinematicMove::Goal::TRUCK_LEFT_RIGHT &&
        (!finite_positive(std::abs(goal->displacement_m)) ||
         std::abs(goal->displacement_m) > max_displacement_m_)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    if (goal->mode == CinematicMove::Goal::ORBIT_ARC &&
        (!finite_positive(std::abs(goal->orbit_angle_deg)) ||
         std::abs(goal->orbit_angle_deg) > max_orbit_angle_deg_ ||
         !finite_positive(goal->orbit_radius_m) ||
         goal->orbit_radius_m < min_target_distance_m_ ||
         goal->orbit_radius_m > max_target_distance_m_)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle>)
  {
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    active_goal_ = goal_handle;
    goal_ = *goal_handle->get_goal();
    phase_ = Phase::Acquire;
    locked_target_id_ = goal_.tracking_id;
    goal_start_time_ = now();
    last_tick_time_ = goal_start_time_;
    target_lost_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    depth_lost_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    completion_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    alignment_start_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    have_start_pose_ = false;
    have_orbit_target_ = false;
    have_initial_distance_ = false;
    current_profile_speed_ = 0.0;
    traveled_distance_m_ = 0.0;
    completed_angle_rad_ = 0.0;
    previous_orbit_bearing_rad_ = 0.0;
    RCLCPP_INFO(
      get_logger(), "接受运镜任务: mode=%u target=%d",
      goal_.mode, goal_.tracking_id);
  }

  std::optional<Target> select_target(
    const TargetArray::ConstSharedPtr & array, int32_t target_id) const
  {
    if (!array) {
      return std::nullopt;
    }
    int32_t selected_id = target_id;
    if (selected_id < 0 && array->tracking_id >= 0) {
      selected_id = array->tracking_id;
    }
    for (const auto & target : array->targets) {
      if ((selected_id < 0 || target.id == selected_id) &&
          target.tracking_state == Target::TRACKING_STATE_CONFIRMED &&
          target.visible) {
        return target;
      }
    }
    return std::nullopt;
  }

  bool fresh(const rclcpp::Time & stamp, const rclcpp::Time & tick, double timeout) const
  {
    return stamp.nanoseconds() > 0 &&
      (tick - stamp).seconds() >= -0.05 &&
      (tick - stamp).seconds() <= timeout;
  }

  bool valid_metric_target(const Target & target) const
  {
    const bool confidence_allowed =
      (target.fusion_state == Target::FUSION_STATE_VALID &&
       target.depth_confidence >= min_valid_depth_confidence_) ||
      (target.fusion_state == Target::FUSION_STATE_DEGRADED &&
       target.depth_confidence >= min_degraded_depth_confidence_);
    return target.visible && confidence_allowed &&
      std::isfinite(target.position[0]) &&
      std::isfinite(target.position[2]) && target.position[2] > 0.0F;
  }

  double requested_speed() const
  {
    const double requested = goal_.max_speed > 0.0F ?
      static_cast<double>(goal_.max_speed) : default_max_speed_mps_;
    return std::clamp(requested, 0.01, max_linear_speed_mps_);
  }

  double direction_sign() const
  {
    if (goal_.direction == CinematicMove::Goal::DIRECTION_RIGHT) {
      return -1.0;
    }
    if (goal_.direction == CinematicMove::Goal::DIRECTION_LEFT) {
      return 1.0;
    }
    if (goal_.mode == CinematicMove::Goal::TRUCK_LEFT_RIGHT) {
      return goal_.displacement_m < 0.0F ? -1.0 : 1.0;
    }
    return goal_.orbit_angle_deg < 0.0F ? -1.0 : 1.0;
  }

  // At the usual initial geometry (robot behind and facing the target), a
  // physical left strafe increases world +Y while the conventional CCW
  // tangent of the target-to-robot radius points toward -Y. Orbit therefore
  // needs the opposite sign from base_link lateral truck motion.
  double orbit_direction_sign() const
  {
    return -direction_sign();
  }

  double profile_speed(double remaining, double dt)
  {
    const double braking_speed = std::sqrt(
      std::max(0.0, 2.0 * linear_acceleration_mps2_ * remaining));
    const double target_speed = std::min(requested_speed(), braking_speed);
    const double max_delta = linear_acceleration_mps2_ * std::clamp(dt, 0.001, 0.10);
    current_profile_speed_ = std::clamp(
      target_speed,
      std::max(0.0, current_profile_speed_ - max_delta),
      current_profile_speed_ + max_delta);
    return current_profile_speed_;
  }

  void initialize_start_geometry(
    const vision_servo_msgs::msg::PlatformState & platform,
    const Target & metric_target)
  {
    start_x_ = platform.chassis_pose[0];
    start_y_ = platform.chassis_pose[1];
    start_yaw_ = platform.chassis_pose[2];
    have_start_pose_ = true;
    initial_distance_m_ = metric_target.position[2];
    have_initial_distance_ = true;

    const double camera_yaw = platform.gimbal_yaw + camera_mount_yaw_offset_rad_;
    const double forward_base = metric_target.position[2];
    const double left_base = -metric_target.position[0];
    const double bx = std::cos(camera_yaw) * forward_base -
      std::sin(camera_yaw) * left_base;
    const double by = std::sin(camera_yaw) * forward_base +
      std::cos(camera_yaw) * left_base;
    target_world_x_ = start_x_ + std::cos(start_yaw_) * bx - std::sin(start_yaw_) * by;
    target_world_y_ = start_y_ + std::sin(start_yaw_) * bx + std::cos(start_yaw_) * by;
    previous_orbit_bearing_rad_ = std::atan2(
      start_y_ - target_world_y_, start_x_ - target_world_x_);
    have_orbit_target_ = true;
  }

  void update_world_target(
    const vision_servo_msgs::msg::PlatformState & platform,
    const Target & target)
  {
    if (!have_orbit_target_) {
      initialize_start_geometry(platform, target);
      return;
    }
    const double yaw = platform.chassis_pose[2];
    const double camera_yaw = platform.gimbal_yaw + camera_mount_yaw_offset_rad_;
    const double forward_base = target.position[2];
    const double left_base = -target.position[0];
    const double bx = std::cos(camera_yaw) * forward_base -
      std::sin(camera_yaw) * left_base;
    const double by = std::sin(camera_yaw) * forward_base +
      std::cos(camera_yaw) * left_base;
    const double measured_x = platform.chassis_pose[0] +
      std::cos(yaw) * bx - std::sin(yaw) * by;
    const double measured_y = platform.chassis_pose[1] +
      std::sin(yaw) * bx + std::cos(yaw) * by;
    target_world_x_ += target_world_filter_alpha_ * (measured_x - target_world_x_);
    target_world_y_ += target_world_filter_alpha_ * (measured_y - target_world_y_);
  }

  void publish_reference(const Reference & reference)
  {
    reference_pub_->publish(reference);
  }

  Reference stopped_reference(const rclcpp::Time & tick) const
  {
    Reference reference;
    reference.header.stamp = tick;
    reference.header.frame_id = "base_link";
    reference.mode = goal_.mode;
    reference.tracking_id = locked_target_id_;
    double hold_depth = have_initial_distance_ ? initial_distance_m_ : 0.0;
    if (goal_.mode == CinematicMove::Goal::DOLLY_IN_OUT &&
        goal_.target_distance_m > 0.0F) {
      hold_depth = goal_.target_distance_m;
    } else if (goal_.mode == CinematicMove::Goal::ORBIT_ARC &&
               goal_.orbit_radius_m > 0.0F) {
      hold_depth = goal_.orbit_radius_m;
    }
    reference.desired_depth = static_cast<float>(hold_depth);
    reference.max_linear_speed = static_cast<float>(requested_speed());
    reference.allow_translation = false;
    reference.valid = true;
    return reference;
  }

  void finish_goal(uint8_t result_code, const std::string & message, bool canceled = false)
  {
    auto result = std::make_shared<CinematicMove::Result>();
    result->success = result_code == CinematicMove::Result::RESULT_SUCCESS;
    result->result_code = result_code;
    result->message = message;
    result->traveled_distance_m = static_cast<float>(traveled_distance_m_);
    result->completed_angle_deg = static_cast<float>(completed_angle_rad_ * 180.0 / M_PI);
    if (active_goal_) {
      if (canceled) {
        active_goal_->canceled(result);
      } else if (result->success) {
        active_goal_->succeed(result);
      } else {
        active_goal_->abort(result);
      }
    }
    active_goal_.reset();
    phase_ = Phase::Idle;
    locked_target_id_ = -1;
    current_profile_speed_ = 0.0;
  }

  void publish_feedback(double progress, double current_distance)
  {
    if (!active_goal_) {
      return;
    }
    auto feedback = std::make_shared<CinematicMove::Feedback>();
    feedback->state = static_cast<uint8_t>(phase_);
    feedback->progress = static_cast<float>(std::clamp(progress, 0.0, 1.0));
    feedback->current_distance_m = static_cast<float>(current_distance);
    feedback->traveled_distance_m = static_cast<float>(traveled_distance_m_);
    feedback->completed_angle_deg = static_cast<float>(completed_angle_rad_ * 180.0 / M_PI);
    feedback->active_tracking_id = locked_target_id_;
    active_goal_->publish_feedback(feedback);
  }

  void control_tick()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!active_goal_) {
      return;
    }
    const auto tick = now();
    const double dt = std::clamp((tick - last_tick_time_).seconds(), 0.001, 0.10);
    last_tick_time_ = tick;

    if (active_goal_->is_canceling()) {
      publish_reference(stopped_reference(tick));
      finish_goal(CinematicMove::Result::RESULT_CANCELED, "运镜任务已取消", true);
      return;
    }
    if (goal_.duration_sec > 0.0F &&
        (tick - goal_start_time_).seconds() > goal_.duration_sec) {
      publish_reference(stopped_reference(tick));
      if (goal_.mode == CinematicMove::Goal::STATIC_TRACK) {
        finish_goal(CinematicMove::Result::RESULT_SUCCESS, "定时静态跟踪完成");
      } else {
        finish_goal(CinematicMove::Result::RESULT_TIMEOUT, "运镜任务超时");
      }
      return;
    }

    const bool tracks_fresh = fresh(tracks_receive_time_, tick, target_timeout_sec_);
    auto visual_target = tracks_fresh ? select_target(latest_tracks_, locked_target_id_) :
      std::optional<Target>{};
    if (!visual_target) {
      const bool three_d_fresh = fresh(targets_3d_receive_time_, tick, target_timeout_sec_);
      visual_target = three_d_fresh ? select_target(latest_targets_3d_, locked_target_id_) :
        std::optional<Target>{};
    }

    if (!visual_target) {
      if (target_lost_since_.nanoseconds() == 0) {
        target_lost_since_ = tick;
      }
      phase_ = Phase::Recover;
      publish_reference(stopped_reference(tick));
      publish_feedback(0.0, 0.0);
      if ((tick - target_lost_since_).seconds() > target_lost_abort_sec_) {
        finish_goal(CinematicMove::Result::RESULT_TARGET_LOST, "目标丢失");
      }
      return;
    }
    target_lost_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    if (locked_target_id_ < 0) {
      locked_target_id_ = visual_target->id;
    }

    const bool metric_fresh = fresh(targets_3d_receive_time_, tick, target_timeout_sec_);
    auto metric_target = metric_fresh ? select_target(latest_targets_3d_, locked_target_id_) :
      std::optional<Target>{};
    const bool metric_valid = metric_target && valid_metric_target(*metric_target);
    const bool motion_mode = goal_.mode != CinematicMove::Goal::STATIC_TRACK;
    if (motion_mode && !metric_valid) {
      if (depth_lost_since_.nanoseconds() == 0) {
        depth_lost_since_ = tick;
      }
      phase_ = Phase::Recover;
      publish_reference(stopped_reference(tick));
      publish_feedback(0.0, 0.0);
      if ((tick - depth_lost_since_).seconds() > depth_abort_sec_) {
        finish_goal(CinematicMove::Result::RESULT_DEPTH_INVALID, "三维深度不可用");
      }
      return;
    }
    depth_lost_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());

    const bool platform_fresh = fresh(platform_receive_time_, tick, target_timeout_sec_);
    if (motion_mode &&
        (!platform_fresh || !platform_state_.chassis_connected ||
         platform_state_.emergency_stop)) {
      publish_reference(stopped_reference(tick));
      finish_goal(CinematicMove::Result::RESULT_SAFETY_STOP, "底盘状态不允许运镜");
      return;
    }

    if (phase_ == Phase::Acquire || phase_ == Phase::Recover) {
      phase_ = Phase::Align;
      alignment_start_time_ = tick;
    }
    if (phase_ == Phase::Align) {
      if (motion_mode) {
        const double bearing = std::atan2(
          static_cast<double>(metric_target->position[0]),
          static_cast<double>(metric_target->position[2]));
        if (std::abs(bearing) > alignment_bearing_tolerance_rad_) {
          publish_reference(stopped_reference(tick));
          publish_feedback(0.0, metric_target->position[2]);
          if ((tick - alignment_start_time_).seconds() > alignment_timeout_sec_) {
            finish_goal(CinematicMove::Result::RESULT_TIMEOUT, "目标未能在运镜前居中");
          }
          return;
        }
      }
      if (metric_valid && !have_start_pose_) {
        initialize_start_geometry(platform_state_, *metric_target);
      }
      phase_ = Phase::Execute;
    }

    Reference reference;
    reference.header.stamp = tick;
    reference.header.frame_id = "base_link";
    reference.mode = goal_.mode;
    reference.tracking_id = locked_target_id_;
    reference.valid = true;
    reference.allow_translation = motion_mode;
    reference.max_linear_speed = static_cast<float>(requested_speed());
    const double current_distance = metric_valid ? metric_target->position[2] : 0.0;
    double progress = 0.0;
    bool complete = false;

    if (goal_.mode == CinematicMove::Goal::STATIC_TRACK) {
      reference.allow_translation = false;
      reference.desired_depth = static_cast<float>(
        metric_valid ? current_distance : 0.0);
      progress = 1.0;
    } else if (goal_.mode == CinematicMove::Goal::DOLLY_IN_OUT) {
      reference.desired_depth = goal_.target_distance_m;
      if (!have_initial_distance_) {
        initial_distance_m_ = current_distance;
        have_initial_distance_ = true;
      }
      const double total = std::max(
        distance_tolerance_m_, std::abs(initial_distance_m_ - goal_.target_distance_m));
      const double remaining = std::abs(current_distance - goal_.target_distance_m);
      progress = 1.0 - std::clamp(remaining / total, 0.0, 1.0);
      complete = remaining <= distance_tolerance_m_;
    } else if (goal_.mode == CinematicMove::Goal::TRUCK_LEFT_RIGHT) {
      reference.desired_depth = static_cast<float>(
        goal_.target_distance_m > 0.0F ? goal_.target_distance_m : initial_distance_m_);
      const double dx = platform_state_.chassis_pose[0] - start_x_;
      const double dy = platform_state_.chassis_pose[1] - start_y_;
      const double left_world_x = -std::sin(start_yaw_);
      const double left_world_y = std::cos(start_yaw_);
      traveled_distance_m_ = direction_sign() *
        (dx * left_world_x + dy * left_world_y);
      const double total = std::abs(goal_.displacement_m);
      const double remaining = std::max(0.0, total - traveled_distance_m_);
      reference.chassis_feedforward.linear.y =
        direction_sign() * profile_speed(remaining, dt);
      progress = std::clamp(traveled_distance_m_ / total, 0.0, 1.0);
      complete = remaining <= distance_tolerance_m_;
    } else {
      reference.desired_depth = goal_.orbit_radius_m;
      update_world_target(platform_state_, *metric_target);
      const double rx = platform_state_.chassis_pose[0] - target_world_x_;
      const double ry = platform_state_.chassis_pose[1] - target_world_y_;
      const double radius = std::max(0.05, std::hypot(rx, ry));
      const double bearing = std::atan2(ry, rx);
      const double signed_delta = orbit_direction_sign() *
        wrap_angle(bearing - previous_orbit_bearing_rad_);
      if (signed_delta > -0.20) {
        completed_angle_rad_ += std::max(0.0, signed_delta);
      }
      previous_orbit_bearing_rad_ = bearing;

      const double target_angle = std::abs(goal_.orbit_angle_deg) * M_PI / 180.0;
      const double remaining_angle = std::max(0.0, target_angle - completed_angle_rad_);
      const double remaining_arc = remaining_angle * goal_.orbit_radius_m;
      const double tangential_speed = profile_speed(remaining_arc, dt);
      const double ex = rx / radius;
      const double ey = ry / radius;
      const double radial_speed = std::clamp(
        -orbit_radial_gain_ * (radius - goal_.orbit_radius_m),
        -max_linear_speed_mps_, max_linear_speed_mps_);
      const double world_vx = radial_speed * ex +
        orbit_direction_sign() * tangential_speed * (-ey);
      const double world_vy = radial_speed * ey +
        orbit_direction_sign() * tangential_speed * ex;
      const double yaw = platform_state_.chassis_pose[2];
      reference.chassis_feedforward.linear.x =
        std::cos(yaw) * world_vx + std::sin(yaw) * world_vy;
      reference.chassis_feedforward.linear.y =
        -std::sin(yaw) * world_vx + std::cos(yaw) * world_vy;
      const double target_bearing_base = std::atan2(
        -metric_target->position[0], metric_target->position[2]);
      reference.chassis_feedforward.angular.z = std::clamp(
        orbit_facing_gain_ * target_bearing_base,
        -max_angular_speed_rad_s_, max_angular_speed_rad_s_);
      progress = std::clamp(completed_angle_rad_ / target_angle, 0.0, 1.0);
      complete = remaining_angle <= orbit_angle_tolerance_rad_ &&
        std::abs(radius - goal_.orbit_radius_m) <= orbit_radius_tolerance_m_;
    }

    reference.progress = static_cast<float>(progress);
    if (complete) {
      reference.allow_translation = false;
      reference.chassis_feedforward = geometry_msgs::msg::Twist();
      phase_ = Phase::Hold;
      if (completion_since_.nanoseconds() == 0) {
        completion_since_ = tick;
      }
      if ((tick - completion_since_).seconds() >= completion_hold_sec_) {
        publish_reference(reference);
        finish_goal(CinematicMove::Result::RESULT_SUCCESS, "运镜任务完成");
        return;
      }
    } else {
      completion_since_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
      phase_ = Phase::Execute;
    }
    publish_reference(reference);
    publish_feedback(progress, current_distance);
  }

  double control_rate_hz_{20.0};
  double target_timeout_sec_{0.35};
  double target_lost_abort_sec_{1.0};
  double depth_abort_sec_{0.60};
  double alignment_timeout_sec_{3.0};
  double alignment_bearing_tolerance_rad_{5.0 * M_PI / 180.0};
  double min_valid_depth_confidence_{0.65};
  double min_degraded_depth_confidence_{0.40};
  double default_target_distance_m_{2.0};
  double default_max_speed_mps_{0.10};
  double max_linear_speed_mps_{0.20};
  double max_angular_speed_rad_s_{0.25};
  double linear_acceleration_mps2_{0.20};
  double distance_tolerance_m_{0.10};
  double orbit_radius_tolerance_m_{0.15};
  double orbit_angle_tolerance_rad_{3.0 * M_PI / 180.0};
  double orbit_radial_gain_{0.45};
  double orbit_facing_gain_{0.50};
  double camera_mount_yaw_offset_rad_{0.0};
  double target_world_filter_alpha_{0.08};
  double completion_hold_sec_{0.40};
  double max_task_duration_sec_{120.0};
  double max_displacement_m_{2.0};
  double max_orbit_angle_deg_{180.0};
  double min_target_distance_m_{0.60};
  double max_target_distance_m_{5.0};

  std::mutex mutex_;
  std::shared_ptr<GoalHandle> active_goal_;
  CinematicMove::Goal goal_;
  Phase phase_{Phase::Idle};
  int32_t locked_target_id_{-1};
  TargetArray::ConstSharedPtr latest_tracks_;
  TargetArray::ConstSharedPtr latest_targets_3d_;
  vision_servo_msgs::msg::PlatformState platform_state_;
  rclcpp::Time tracks_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time targets_3d_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time platform_receive_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tick_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time target_lost_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time depth_lost_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time completion_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Time alignment_start_time_{0, 0, RCL_ROS_TIME};

  bool have_start_pose_{false};
  bool have_orbit_target_{false};
  bool have_initial_distance_{false};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double initial_distance_m_{0.0};
  double target_world_x_{0.0};
  double target_world_y_{0.0};
  double previous_orbit_bearing_rad_{0.0};
  double current_profile_speed_{0.0};
  double traveled_distance_m_{0.0};
  double completed_angle_rad_{0.0};

  rclcpp::Publisher<Reference>::SharedPtr reference_pub_;
  rclcpp::Subscription<TargetArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<TargetArray>::SharedPtr targets_3d_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::PlatformState>::SharedPtr platform_sub_;
  rclcpp_action::Server<CinematicMove>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace servo_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<servo_control_pkg::CinematicMotionManagerNode>());
  rclcpp::shutdown();
  return 0;
}
