#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_servo_msgs/action/manual_jog.hpp>
#include <vision_servo_msgs/msg/gimbal_nudge.hpp>
#include <vision_servo_msgs/msg/gimbal_status.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <sstream>
#include <string>

namespace teleop_control_pkg
{

class ManualJogManagerNode : public rclcpp::Node
{
public:
  using ManualJog = vision_servo_msgs::action::ManualJog;
  using GoalHandle = rclcpp_action::ServerGoalHandle<ManualJog>;

  ManualJogManagerNode()
  : Node("manual_jog_manager")
  {
    declare_parameter("control_rate_hz", 30.0);
    declare_parameter("ownership_settle_sec", 0.25);
    declare_parameter("manual_arm_sec", 0.12);
    declare_parameter("stop_settle_sec", 0.20);
    declare_parameter("sensor_timeout_sec", 0.40);
    declare_parameter("position_tolerance_m", 0.015);
    declare_parameter("angle_tolerance_rad", 0.0174532925);
    declare_parameter("max_translation_m", 0.50);
    declare_parameter("max_chassis_rotation_rad", 0.7853981634);
    declare_parameter("max_gimbal_rotation_rad", 0.1745329252);
    declare_parameter("max_linear_speed", 0.12);
    declare_parameter("max_chassis_yaw_rate", 0.35);
    declare_parameter("max_gimbal_rate", 0.30);
    declare_parameter("min_command_speed", 0.02);
    declare_parameter("frame_id", "base_link");
    declare_parameter("action_name", "/manual_jog/execute");
    declare_parameter("cinematic_exit_service", "/cinematic/exit");

    control_rate_hz_ = std::clamp(get_parameter("control_rate_hz").as_double(), 10.0, 100.0);
    ownership_settle_sec_ = std::clamp(
      get_parameter("ownership_settle_sec").as_double(), 0.05, 1.0);
    manual_arm_sec_ = std::clamp(get_parameter("manual_arm_sec").as_double(), 0.05, 0.5);
    stop_settle_sec_ = std::clamp(get_parameter("stop_settle_sec").as_double(), 0.05, 1.0);
    sensor_timeout_sec_ = std::max(0.10, get_parameter("sensor_timeout_sec").as_double());
    position_tolerance_m_ = std::max(0.003, get_parameter("position_tolerance_m").as_double());
    angle_tolerance_rad_ = std::max(0.003, get_parameter("angle_tolerance_rad").as_double());
    max_translation_m_ = std::max(0.01, get_parameter("max_translation_m").as_double());
    max_chassis_rotation_rad_ = std::max(
      0.01, get_parameter("max_chassis_rotation_rad").as_double());
    max_gimbal_rotation_rad_ = std::max(
      0.01, get_parameter("max_gimbal_rotation_rad").as_double());
    max_linear_speed_ = std::max(0.01, get_parameter("max_linear_speed").as_double());
    max_chassis_yaw_rate_ = std::max(
      0.01, get_parameter("max_chassis_yaw_rate").as_double());
    max_gimbal_rate_ = std::max(0.01, get_parameter("max_gimbal_rate").as_double());
    min_command_speed_ = std::max(0.005, get_parameter("min_command_speed").as_double());
    frame_id_ = get_parameter("frame_id").as_string();

    const auto command_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
    mode_pub_ = create_publisher<std_msgs::msg::String>("/teleop/mode", command_qos);
    heartbeat_pub_ = create_publisher<std_msgs::msg::Empty>("/teleop/heartbeat", command_qos);
    deadman_pub_ = create_publisher<std_msgs::msg::Bool>("/teleop/deadman", command_qos);
    velocity_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      "/teleop/cmd_vel", command_qos);
    gimbal_nudge_pub_ = create_publisher<vision_servo_msgs::msg::GimbalNudge>(
      "/teleop/gimbal_nudge", command_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/manual_jog/status",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
    active_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/manual_jog/active",
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        latest_odom_ = *msg;
        last_odom_time_ = now();
        have_odom_ = true;
      });
    gimbal_sub_ = create_subscription<vision_servo_msgs::msg::GimbalStatus>(
      "/gimbal/status", rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      [this](vision_servo_msgs::msg::GimbalStatus::ConstSharedPtr msg) {
        latest_gimbal_ = *msg;
        last_gimbal_time_ = now();
        have_gimbal_ = true;
      });
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/safety/estop_state", command_qos,
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {estop_active_ = msg->data;});

    cinematic_exit_client_ = create_client<std_srvs::srv::Trigger>(
      get_parameter("cinematic_exit_service").as_string());
    stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/manual_jog/stop",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        if (!active_goal_) {
          response->success = true;
          response->message = "MANUAL_JOG未执行任务";
          return;
        }
        // This service is an external safety stop, not an action cancel request.
        // Abort the goal after settling so rclcpp_action is not asked to move a
        // non-CANCELING goal directly into the CANCELED terminal state.
        begin_settling(ManualJog::Result::RESULT_CANCELED, "手动微调已停止", false);
        response->success = true;
        response->message = "正在平滑停止并返回STANDBY";
      });

    action_server_ = rclcpp_action::create_server<ManualJog>(
      this, get_parameter("action_name").as_string(),
      std::bind(
        &ManualJogManagerNode::handle_goal, this,
        std::placeholders::_1, std::placeholders::_2),
      std::bind(&ManualJogManagerNode::handle_cancel, this, std::placeholders::_1),
      std::bind(&ManualJogManagerNode::handle_accepted, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / control_rate_hz_)),
      std::bind(&ManualJogManagerNode::control_tick, this));
    publish_status("STANDBY", "IDLE");
    publish_active(false);
    RCLCPP_INFO(get_logger(), "MANUAL_JOG管理器已启动 (%.1fHz)", control_rate_hz_);
  }

  ~ManualJogManagerNode() override
  {
    publish_zero();
    publish_deadman(false);
    publish_mode("stop");
  }

private:
  enum class Phase : uint8_t {Idle, Entering, Arming, Executing, Settling};

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ManualJog::Goal> goal)
  {
    if (active_goal_ || phase_ != Phase::Idle || !valid_goal(*goal)) {
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
  {
    return goal_handle == active_goal_ ?
      rclcpp_action::CancelResponse::ACCEPT : rclcpp_action::CancelResponse::REJECT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    active_goal_ = goal_handle;
    goal_ = *goal_handle->get_goal();
    phase_ = Phase::Entering;
    phase_started_ = now();
    goal_started_ = phase_started_;
    completed_ = 0.0;
    gimbal_nudge_sent_ = false;
    pending_result_code_ = ManualJog::Result::RESULT_SUCCESS;
    pending_result_message_.clear();
    pending_canceled_ = false;
    publish_active(true);

    // Revoke autonomous ownership first. If cinematic support is absent, the
    // stop/manual handover still safely suppresses autonomous commands.
    publish_mode("stop");
    publish_zero();
    publish_deadman(false);
    if (cinematic_exit_client_->service_is_ready()) {
      cinematic_exit_client_->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>());
    }
    publish_status("MANUAL_JOG", "ENTERING");
  }

  bool valid_goal(const ManualJog::Goal & goal) const
  {
    if (!std::isfinite(goal.displacement) || !std::isfinite(goal.max_speed) ||
      !std::isfinite(goal.timeout_sec) || std::abs(goal.displacement) < 1e-5F ||
      goal.max_speed <= 0.0F || goal.timeout_sec <= 0.0F ||
      goal.axis > ManualJog::Goal::GIMBAL_PITCH)
    {
      return false;
    }
    if (is_translation(goal.axis) && std::abs(goal.displacement) > max_translation_m_) {
      return false;
    }
    if (goal.axis == ManualJog::Goal::CHASSIS_YAW &&
      std::abs(goal.displacement) > max_chassis_rotation_rad_)
    {
      return false;
    }
    return !is_gimbal(goal.axis) ||
           std::abs(goal.displacement) <= max_gimbal_rotation_rad_;
  }

  static bool is_gimbal(uint8_t axis)
  {
    return axis == ManualJog::Goal::GIMBAL_YAW || axis == ManualJog::Goal::GIMBAL_PITCH;
  }

  static bool is_translation(uint8_t axis)
  {
    return axis == ManualJog::Goal::CHASSIS_X || axis == ManualJog::Goal::CHASSIS_Y;
  }

  bool sensor_ready(const rclcpp::Time & tick) const
  {
    if (is_gimbal(goal_.axis)) {
      return have_gimbal_ && latest_gimbal_.connected &&
             (tick - last_gimbal_time_).seconds() <= sensor_timeout_sec_;
    }
    return have_odom_ && (tick - last_odom_time_).seconds() <= sensor_timeout_sec_;
  }

  void control_tick()
  {
    const auto tick = now();
    if (!active_goal_) {return;}
    if (estop_active_) {
      begin_settling(ManualJog::Result::RESULT_ESTOP, "急停已激活", false);
    }
    if (active_goal_->is_canceling() && phase_ != Phase::Settling) {
      begin_settling(ManualJog::Result::RESULT_CANCELED, "手动微调已取消", true);
    }
    if (phase_ != Phase::Settling && (tick - goal_started_).seconds() > goal_.timeout_sec) {
      begin_settling(ManualJog::Result::RESULT_TIMEOUT, "手动微调超时", false);
    }

    if (phase_ == Phase::Entering) {
      publish_zero();
      publish_deadman(false);
      if ((tick - phase_started_).seconds() >= ownership_settle_sec_) {
        if (!sensor_ready(tick)) {
          begin_settling(
            is_gimbal(goal_.axis) ? ManualJog::Result::RESULT_GIMBAL_UNAVAILABLE :
            ManualJog::Result::RESULT_ODOMETRY_UNAVAILABLE,
            is_gimbal(goal_.axis) ? "云台状态不可用" : "里程计不可用", false);
          return;
        }
        publish_mode("manual");
        publish_deadman(true);
        publish_heartbeat();
        publish_zero();
        phase_ = Phase::Arming;
        phase_started_ = tick;
        publish_status("MANUAL_JOG", "ARMING");
      }
      return;
    }

    if (phase_ == Phase::Arming) {
      publish_heartbeat();
      publish_deadman(true);
      publish_zero();
      if ((tick - phase_started_).seconds() >= manual_arm_sec_) {
        capture_start_state();
        phase_ = Phase::Executing;
        phase_started_ = tick;
        publish_status("MANUAL_JOG", "EXECUTING");
      }
      return;
    }

    if (phase_ == Phase::Executing) {
      if (!sensor_ready(tick)) {
        begin_settling(
          is_gimbal(goal_.axis) ? ManualJog::Result::RESULT_GIMBAL_UNAVAILABLE :
          ManualJog::Result::RESULT_ODOMETRY_UNAVAILABLE,
          is_gimbal(goal_.axis) ? "云台反馈中断" : "里程计反馈中断", false);
        return;
      }
      publish_heartbeat();
      publish_deadman(true);
      completed_ = measured_displacement();
      const double remaining = static_cast<double>(goal_.displacement) - completed_;
      publish_feedback(remaining);
      const double tolerance = is_translation(goal_.axis) ?
        position_tolerance_m_ : angle_tolerance_rad_;
      if (std::abs(remaining) <= tolerance || passed_target(goal_.displacement, completed_)) {
        begin_settling(ManualJog::Result::RESULT_SUCCESS, "手动微调完成", false);
        return;
      }
      if (is_gimbal(goal_.axis)) {
        publish_gimbal_nudge_once();
      } else {
        publish_chassis_command(remaining);
      }
      return;
    }

    if (phase_ == Phase::Settling) {
      publish_zero();
      publish_heartbeat();
      publish_deadman(true);
      if ((tick - phase_started_).seconds() >= stop_settle_sec_) {complete_goal();}
    }
  }

  static bool passed_target(double target, double completed)
  {
    return target > 0.0 ? completed >= target : completed <= target;
  }

  void capture_start_state()
  {
    if (is_gimbal(goal_.axis)) {
      start_gimbal_yaw_ = latest_gimbal_.yaw;
      start_gimbal_pitch_ = latest_gimbal_.pitch;
    } else {
      start_x_ = latest_odom_.pose.pose.position.x;
      start_y_ = latest_odom_.pose.pose.position.y;
      start_yaw_ = odom_yaw(latest_odom_);
    }
  }

  double measured_displacement() const
  {
    if (goal_.axis == ManualJog::Goal::GIMBAL_YAW) {
      return normalize_angle(latest_gimbal_.yaw - start_gimbal_yaw_);
    }
    if (goal_.axis == ManualJog::Goal::GIMBAL_PITCH) {
      return normalize_angle(latest_gimbal_.pitch - start_gimbal_pitch_);
    }
    if (goal_.axis == ManualJog::Goal::CHASSIS_YAW) {
      return normalize_angle(odom_yaw(latest_odom_) - start_yaw_);
    }
    const double dx = latest_odom_.pose.pose.position.x - start_x_;
    const double dy = latest_odom_.pose.pose.position.y - start_y_;
    if (goal_.axis == ManualJog::Goal::CHASSIS_X) {
      return std::cos(start_yaw_) * dx + std::sin(start_yaw_) * dy;
    }
    return -std::sin(start_yaw_) * dx + std::cos(start_yaw_) * dy;
  }

  void publish_chassis_command(double remaining)
  {
    const double speed_limit = goal_.axis == ManualJog::Goal::CHASSIS_YAW ?
      std::min<double>(goal_.max_speed, max_chassis_yaw_rate_) :
      std::min<double>(goal_.max_speed, max_linear_speed_);
    const double magnitude = std::min(
      speed_limit, std::max(min_command_speed_, std::abs(remaining) * 1.5));
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = frame_id_;
    const double signed_speed = std::copysign(magnitude, remaining);
    if (goal_.axis == ManualJog::Goal::CHASSIS_X) {
      command.twist.linear.x = signed_speed;
    } else if (goal_.axis == ManualJog::Goal::CHASSIS_Y) {
      command.twist.linear.y = signed_speed;
    } else {
      command.twist.angular.z = signed_speed;
    }
    velocity_pub_->publish(command);
  }

  void publish_gimbal_nudge_once()
  {
    if (gimbal_nudge_sent_) {
      publish_zero();
      return;
    }
    vision_servo_msgs::msg::GimbalNudge command;
    command.header.stamp = now();
    command.header.frame_id = "gimbal_link";
    command.command_id = ++next_command_id_;
    if (goal_.axis == ManualJog::Goal::GIMBAL_YAW) {
      command.yaw_delta = goal_.displacement;
    } else {
      command.pitch_delta = goal_.displacement;
    }
    const double rate = std::min<double>(goal_.max_speed, max_gimbal_rate_);
    command.duration = static_cast<float>(std::clamp(
      std::abs(goal_.displacement) / rate, 0.10, 1.0));
    gimbal_nudge_pub_->publish(command);
    gimbal_nudge_sent_ = true;
  }

  void begin_settling(uint8_t result_code, const std::string & message, bool canceled)
  {
    if (!active_goal_ || phase_ == Phase::Settling) {return;}
    pending_result_code_ = result_code;
    pending_result_message_ = message;
    pending_canceled_ = canceled;
    phase_ = Phase::Settling;
    phase_started_ = now();
    publish_zero();
    publish_status("MANUAL_JOG", "SETTLING");
  }

  void complete_goal()
  {
    publish_zero();
    publish_deadman(false);
    publish_mode("stop");
    auto result = std::make_shared<ManualJog::Result>();
    result->success = pending_result_code_ == ManualJog::Result::RESULT_SUCCESS;
    result->result_code = pending_result_code_;
    result->message = pending_result_message_;
    result->completed_displacement = static_cast<float>(completed_);
    if (pending_canceled_) {
      active_goal_->canceled(result);
    } else if (result->success) {
      active_goal_->succeed(result);
    } else {
      active_goal_->abort(result);
    }
    active_goal_.reset();
    phase_ = Phase::Idle;
    publish_active(false);
    publish_status("STANDBY", "IDLE");
  }

  void publish_feedback(double remaining)
  {
    auto feedback = std::make_shared<ManualJog::Feedback>();
    feedback->state = ManualJog::Feedback::STATE_EXECUTING;
    feedback->completed_displacement = static_cast<float>(completed_);
    feedback->remaining_displacement = static_cast<float>(remaining);
    feedback->progress = static_cast<float>(std::clamp(
      std::abs(completed_) / std::abs(static_cast<double>(goal_.displacement)), 0.0, 1.0));
    active_goal_->publish_feedback(feedback);
  }

  void publish_mode(const std::string & value)
  {
    std_msgs::msg::String message;
    message.data = value;
    mode_pub_->publish(message);
  }

  void publish_heartbeat() {heartbeat_pub_->publish(std_msgs::msg::Empty());}

  void publish_deadman(bool enabled)
  {
    std_msgs::msg::Bool message;
    message.data = enabled;
    deadman_pub_->publish(message);
  }

  void publish_zero()
  {
    geometry_msgs::msg::TwistStamped command;
    command.header.stamp = now();
    command.header.frame_id = frame_id_;
    velocity_pub_->publish(command);
  }

  void publish_status(const std::string & mode, const std::string & state)
  {
    std_msgs::msg::String message;
    std::ostringstream stream;
    stream << "{\"mode\":\"" << mode << "\",\"state\":\"" << state
           << "\",\"axis\":" << (active_goal_ ? static_cast<int>(goal_.axis) : -1)
           << ",\"completed\":" << completed_ << "}";
    message.data = stream.str();
    status_pub_->publish(message);
  }

  void publish_active(bool active)
  {
    std_msgs::msg::Bool message;
    message.data = active;
    active_pub_->publish(message);
  }

  static double normalize_angle(double angle)
  {
    constexpr double pi = 3.14159265358979323846;
    while (angle > pi) {angle -= 2.0 * pi;}
    while (angle < -pi) {angle += 2.0 * pi;}
    return angle;
  }

  static double odom_yaw(const nav_msgs::msg::Odometry & odom)
  {
    const auto & q = odom.pose.pose.orientation;
    return std::atan2(
      2.0 * (q.w * q.z + q.x * q.y),
      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
  }

  double control_rate_hz_{30.0};
  double ownership_settle_sec_{0.25};
  double manual_arm_sec_{0.12};
  double stop_settle_sec_{0.20};
  double sensor_timeout_sec_{0.40};
  double position_tolerance_m_{0.015};
  double angle_tolerance_rad_{0.0174532925};
  double max_translation_m_{0.50};
  double max_chassis_rotation_rad_{0.7853981634};
  double max_gimbal_rotation_rad_{0.1745329252};
  double max_linear_speed_{0.12};
  double max_chassis_yaw_rate_{0.35};
  double max_gimbal_rate_{0.30};
  double min_command_speed_{0.02};
  std::string frame_id_{"base_link"};
  Phase phase_{Phase::Idle};
  ManualJog::Goal goal_;
  std::shared_ptr<GoalHandle> active_goal_;
  rclcpp::Time phase_started_{0, 0, RCL_ROS_TIME};
  rclcpp::Time goal_started_{0, 0, RCL_ROS_TIME};
  nav_msgs::msg::Odometry latest_odom_;
  vision_servo_msgs::msg::GimbalStatus latest_gimbal_;
  rclcpp::Time last_odom_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_gimbal_time_{0, 0, RCL_ROS_TIME};
  bool have_odom_{false};
  bool have_gimbal_{false};
  bool estop_active_{false};
  bool gimbal_nudge_sent_{false};
  uint64_t next_command_id_{0};
  double start_x_{0.0};
  double start_y_{0.0};
  double start_yaw_{0.0};
  double start_gimbal_yaw_{0.0};
  double start_gimbal_pitch_{0.0};
  double completed_{0.0};
  uint8_t pending_result_code_{ManualJog::Result::RESULT_SUCCESS};
  std::string pending_result_message_;
  bool pending_canceled_{false};
  rclcpp_action::Server<ManualJog>::SharedPtr action_server_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cinematic_exit_client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr heartbeat_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr deadman_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::GimbalNudge>::SharedPtr gimbal_nudge_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr active_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::GimbalStatus>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace teleop_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<teleop_control_pkg::ManualJogManagerNode>());
  rclcpp::shutdown();
  return 0;
}
