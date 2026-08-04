#include "external_control_pkg/msg/voice_command.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <vision_servo_msgs/action/manual_jog.hpp>

#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace external_control_pkg
{

class VoiceManualJogActionNode : public rclcpp::Node
{
public:
  using ManualJog = vision_servo_msgs::action::ManualJog;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ManualJog>;
  using VoiceCommand = external_control_pkg::msg::VoiceCommand;

  VoiceManualJogActionNode()
  : Node("voice_manual_jog_action_node")
  {
    declare_parameter("gimbal_voice_topic", "/voice/gimbal_command");
    declare_parameter("chassis_voice_topic", "/voice/chassis_command");
    declare_parameter("action_name", "/manual_jog/execute");
    declare_parameter("min_confidence", 0.50);
    declare_parameter("default_chassis_step_m", 0.10);
    declare_parameter("default_chassis_turn_rad", 0.0872664626);
    declare_parameter("default_gimbal_step_rad", 0.0872664626);
    declare_parameter("linear_speed_mps", 0.08);
    declare_parameter("chassis_yaw_rate", 0.25);
    declare_parameter("gimbal_rate", 0.22);
    declare_parameter("timeout_sec", 8.0);
    declare_parameter("forward_x_sign", 1.0);
    declare_parameter("left_y_sign", 1.0);
    declare_parameter("left_turn_sign", 1.0);
    declare_parameter("right_gimbal_yaw_sign", 1.0);
    declare_parameter("up_gimbal_pitch_sign", 1.0);

    min_confidence_ = std::clamp(
      get_parameter("min_confidence").as_double(), 0.0, 1.0);
    action_client_ = rclcpp_action::create_client<ManualJog>(
      this, get_parameter("action_name").as_string());
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).reliable();
    gimbal_sub_ = create_subscription<VoiceCommand>(
      get_parameter("gimbal_voice_topic").as_string(), qos,
      std::bind(&VoiceManualJogActionNode::voice_callback, this, std::placeholders::_1));
    chassis_sub_ = create_subscription<VoiceCommand>(
      get_parameter("chassis_voice_topic").as_string(), qos,
      std::bind(&VoiceManualJogActionNode::voice_callback, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "语音手动微调Action桥接已启动");
  }

private:
  void voice_callback(const VoiceCommand::ConstSharedPtr command)
  {
    if (has_intent(*command, {"chassis_stop", "gimbal_stop", "stop_gimbal"})) {
      pending_goal_.reset();
      cancel_active();
      return;
    }

    auto goal = classify(*command);
    if (!goal) {return;}
    if (active_goal_ || goal_request_in_flight_) {
      pending_goal_ = *goal;
      cancel_active();
      return;
    }
    send_goal(*goal);
  }

  std::optional<ManualJog::Goal> classify(const VoiceCommand & command) const
  {
    ManualJog::Goal goal;
    goal.timeout_sec = static_cast<float>(get_parameter("timeout_sec").as_double());
    const double requested_distance = distance_metres(command);
    const double chassis_step = requested_distance > 0.0 ? requested_distance :
      get_parameter("default_chassis_step_m").as_double();
    const double turn_step = get_parameter("default_chassis_turn_rad").as_double();
    const double gimbal_step = get_parameter("default_gimbal_step_rad").as_double();

    if (has_intent(command, {"chassis_move_forward"})) {
      goal.axis = ManualJog::Goal::CHASSIS_X;
      goal.displacement = static_cast<float>(sign_param("forward_x_sign") * chassis_step);
      goal.max_speed = static_cast<float>(get_parameter("linear_speed_mps").as_double());
    } else if (has_intent(command, {"chassis_move_backward"})) {
      goal.axis = ManualJog::Goal::CHASSIS_X;
      goal.displacement = static_cast<float>(-sign_param("forward_x_sign") * chassis_step);
      goal.max_speed = static_cast<float>(get_parameter("linear_speed_mps").as_double());
    } else if (has_intent(command, {"chassis_move_left"})) {
      goal.axis = ManualJog::Goal::CHASSIS_Y;
      goal.displacement = static_cast<float>(sign_param("left_y_sign") * chassis_step);
      goal.max_speed = static_cast<float>(get_parameter("linear_speed_mps").as_double());
    } else if (has_intent(command, {"chassis_move_right"})) {
      goal.axis = ManualJog::Goal::CHASSIS_Y;
      goal.displacement = static_cast<float>(-sign_param("left_y_sign") * chassis_step);
      goal.max_speed = static_cast<float>(get_parameter("linear_speed_mps").as_double());
    } else if (has_intent(command, {"chassis_turn_left"})) {
      goal.axis = ManualJog::Goal::CHASSIS_YAW;
      goal.displacement = static_cast<float>(sign_param("left_turn_sign") * turn_step);
      goal.max_speed = static_cast<float>(get_parameter("chassis_yaw_rate").as_double());
    } else if (has_intent(command, {"chassis_turn_right"})) {
      goal.axis = ManualJog::Goal::CHASSIS_YAW;
      goal.displacement = static_cast<float>(-sign_param("left_turn_sign") * turn_step);
      goal.max_speed = static_cast<float>(get_parameter("chassis_yaw_rate").as_double());
    } else if (has_intent(command, {
        "gimbal_nudge_right", "gimbal_right", "turn_gimbal_right"}))
    {
      goal.axis = ManualJog::Goal::GIMBAL_YAW;
      goal.displacement = static_cast<float>(sign_param("right_gimbal_yaw_sign") * gimbal_step);
      goal.max_speed = static_cast<float>(get_parameter("gimbal_rate").as_double());
    } else if (has_intent(command, {
        "gimbal_nudge_left", "gimbal_left", "turn_gimbal_left"}))
    {
      goal.axis = ManualJog::Goal::GIMBAL_YAW;
      goal.displacement = static_cast<float>(-sign_param("right_gimbal_yaw_sign") * gimbal_step);
      goal.max_speed = static_cast<float>(get_parameter("gimbal_rate").as_double());
    } else if (has_intent(command, {
        "gimbal_nudge_up", "gimbal_up", "tilt_gimbal_up"}))
    {
      goal.axis = ManualJog::Goal::GIMBAL_PITCH;
      goal.displacement = static_cast<float>(sign_param("up_gimbal_pitch_sign") * gimbal_step);
      goal.max_speed = static_cast<float>(get_parameter("gimbal_rate").as_double());
    } else if (has_intent(command, {
        "gimbal_nudge_down", "gimbal_down", "tilt_gimbal_down"}))
    {
      goal.axis = ManualJog::Goal::GIMBAL_PITCH;
      goal.displacement = static_cast<float>(-sign_param("up_gimbal_pitch_sign") * gimbal_step);
      goal.max_speed = static_cast<float>(get_parameter("gimbal_rate").as_double());
    } else {
      return std::nullopt;
    }
    return goal;
  }

  bool has_intent(
    const VoiceCommand & command, const std::vector<std::string> & candidates) const
  {
    for (std::size_t i = 0; i < command.intents.size(); ++i) {
      if (std::find(candidates.begin(), candidates.end(), command.intents[i]) ==
        candidates.end())
      {
        continue;
      }
      const float confidence = i < command.confidences.size() ? command.confidences[i] : 0.0F;
      if (confidence >= min_confidence_) {return true;}
    }
    return false;
  }

  static double distance_metres(const VoiceCommand & command)
  {
    if (!std::isfinite(command.distance) || command.distance <= 0.0F) {return -1.0;}
    return command.unit == "cm" ? command.distance * 0.01 : command.distance;
  }

  double sign_param(const std::string & name) const
  {
    return get_parameter(name).as_double() >= 0.0 ? 1.0 : -1.0;
  }

  void send_goal(const ManualJog::Goal & goal)
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "MANUAL_JOG Action未就绪，拒绝语音微调");
      return;
    }
    rclcpp_action::Client<ManualJog>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        goal_request_in_flight_ = false;
        if (!handle) {
          RCLCPP_WARN(get_logger(), "MANUAL_JOG拒绝了语音微调");
          send_pending();
          return;
        }
        active_goal_ = handle;
        if (cancel_when_accepted_) {
          cancel_when_accepted_ = false;
          action_client_->async_cancel_goal(active_goal_);
        }
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        RCLCPP_INFO(
          get_logger(), "语音手动微调结束: action=%d result=%u",
          static_cast<int>(result.code), result.result ? result.result->result_code : 255U);
        active_goal_.reset();
        send_pending();
      };
    goal_request_in_flight_ = true;
    action_client_->async_send_goal(goal, options);
  }

  void cancel_active()
  {
    if (active_goal_) {
      action_client_->async_cancel_goal(active_goal_);
    } else if (goal_request_in_flight_) {
      cancel_when_accepted_ = true;
    }
  }

  void send_pending()
  {
    if (!pending_goal_) {return;}
    const auto goal = *pending_goal_;
    pending_goal_.reset();
    send_goal(goal);
  }

  double min_confidence_{0.50};
  std::optional<ManualJog::Goal> pending_goal_;
  GoalHandle::SharedPtr active_goal_;
  bool goal_request_in_flight_{false};
  bool cancel_when_accepted_{false};
  rclcpp_action::Client<ManualJog>::SharedPtr action_client_;
  rclcpp::Subscription<VoiceCommand>::SharedPtr gimbal_sub_;
  rclcpp::Subscription<VoiceCommand>::SharedPtr chassis_sub_;
};

}  // namespace external_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::VoiceManualJogActionNode>());
  rclcpp::shutdown();
  return 0;
}
