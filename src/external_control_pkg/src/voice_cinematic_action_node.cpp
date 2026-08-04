/**
 * @file voice_cinematic_action_node.cpp
 * @brief Bridge validated autonomy voice intents to CinematicMove actions.
 *
 * Raw ASR text is deliberately ignored. Only dispatcher-approved intents are
 * accepted, and this node never publishes actuator commands directly.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <external_control_pkg/msg/voice_command.hpp>
#include <vision_servo_msgs/action/cinematic_move.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace external_control_pkg {

class VoiceCinematicActionNode : public rclcpp::Node
{
public:
  using Action = vision_servo_msgs::action::CinematicMove;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;
  using VoiceCommand = external_control_pkg::msg::VoiceCommand;

  VoiceCinematicActionNode()
  : Node("voice_cinematic_action_node")
  {
    declare_parameter("voice_command_topic", "/voice/autonomy_command");
    declare_parameter("action_name", "/cinematic/execute");
    declare_parameter("enter_service", "/cinematic/enter");
    declare_parameter("stop_service", "/cinematic/stop");
    declare_parameter("exit_service", "/cinematic/exit");
    declare_parameter("min_confidence", 0.60);
    declare_parameter("default_dolly_distance_m", 1.5);
    declare_parameter("default_orbit_radius_m", 2.0);
    declare_parameter("default_orbit_angle_deg", 30.0);
    declare_parameter("default_speed_mps", 0.08);
    declare_parameter("default_timeout_sec", 20.0);
    declare_parameter("default_orbit_direction", 1);

    min_confidence_ = std::clamp(
      get_parameter("min_confidence").as_double(), 0.0, 1.0);
    service_client_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    action_client_ = rclcpp_action::create_client<Action>(
      this, get_parameter("action_name").as_string());
    enter_client_ = create_client<std_srvs::srv::Trigger>(
      get_parameter("enter_service").as_string(),
      rmw_qos_profile_services_default, service_client_group_);
    stop_client_ = create_client<std_srvs::srv::Trigger>(
      get_parameter("stop_service").as_string(),
      rmw_qos_profile_services_default, service_client_group_);
    exit_client_ = create_client<std_srvs::srv::Trigger>(
      get_parameter("exit_service").as_string(),
      rmw_qos_profile_services_default, service_client_group_);
    command_sub_ = create_subscription<VoiceCommand>(
      get_parameter("voice_command_topic").as_string(),
      rclcpp::QoS(rclcpp::KeepLast(5)).reliable(),
      std::bind(&VoiceCinematicActionNode::command_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(), "语音运镜桥接已启动: %s -> %s",
      get_parameter("voice_command_topic").as_string().c_str(),
      get_parameter("action_name").as_string().c_str());
  }

private:
  void command_callback(const VoiceCommand::ConstSharedPtr msg)
  {
    for (std::size_t i = 0; i < msg->intents.size(); ++i) {
      const double confidence = i < msg->confidences.size() ? msg->confidences[i] : 0.0;
      if (!std::isfinite(confidence) || confidence < min_confidence_) {
        continue;
      }
      const auto & intent = msg->intents[i];
      if (intent == "enter_cinematic") {
        call_mode_service(enter_client_, "进入运镜模式");
      } else if (intent == "exit_cinematic") {
        pending_goal_.reset();
        call_mode_service(exit_client_, "退出运镜模式");
      } else if (intent == "stop_cinematic" || intent == "stop_current_action") {
        pending_goal_.reset();
        call_mode_service(stop_client_, "停止当前运镜动作");
      } else if (intent == "start_following") {
        // Follow is a peer top-level mode.  Explicitly leave cinematic mode;
        // the existing follow command consumer then owns the transition from
        // STANDBY to FOLLOW.
        pending_goal_.reset();
        call_mode_service(exit_client_, "开始跟随前退出运镜模式");
      } else if (intent == "start_dolly") {
        Action::Goal goal;
        goal.mode = Action::Goal::DOLLY_IN_OUT;
        goal.tracking_id = -1;
        const float requested_distance = distance_metres(*msg);
        goal.target_distance_m = requested_distance > 0.0F ? requested_distance :
          static_cast<float>(get_parameter("default_dolly_distance_m").as_double());
        goal.max_speed = speed_from_command(*msg);
        goal.duration_sec = static_cast<float>(
          get_parameter("default_timeout_sec").as_double());
        queue_or_send(goal);
      } else if (intent == "start_orbit") {
        Action::Goal goal;
        goal.mode = Action::Goal::ORBIT_ARC;
        goal.tracking_id = -1;
        const float requested_distance = distance_metres(*msg);
        goal.orbit_radius_m = requested_distance > 0.0F ? requested_distance :
          static_cast<float>(get_parameter("default_orbit_radius_m").as_double());
        goal.orbit_angle_deg = static_cast<float>(
          get_parameter("default_orbit_angle_deg").as_double());
        goal.max_speed = speed_from_command(*msg);
        goal.duration_sec = static_cast<float>(
          get_parameter("default_timeout_sec").as_double());
        goal.direction = static_cast<int8_t>(std::clamp<int64_t>(
          get_parameter("default_orbit_direction").as_int(), -1, 1));
        queue_or_send(goal);
      } else if (intent == "query_camera_motion_status") {
        RCLCPP_INFO(
          get_logger(), "当前运镜状态: %s",
          active_goal_ ? "ACTIVE" : (pending_goal_ ? "PENDING" : "IDLE"));
      }
    }
  }

  float speed_from_command(const VoiceCommand & msg) const
  {
    double speed = get_parameter("default_speed_mps").as_double();
    if (msg.speed == "slow") {
      speed *= 0.65;
    } else if (msg.speed == "fast") {
      speed *= 1.25;
    }
    return static_cast<float>(std::clamp(speed, 0.01, 0.20));
  }

  static float distance_metres(const VoiceCommand & msg)
  {
    if (!std::isfinite(msg.distance) || msg.distance <= 0.0F) {
      return -1.0F;
    }
    if (msg.unit == "cm") {
      return msg.distance * 0.01F;
    }
    return msg.distance;
  }

  void queue_or_send(const Action::Goal & goal)
  {
    if (active_goal_ || goal_request_in_flight_) {
      pending_goal_ = goal;
      cancel_active("切换运镜任务", false);
      return;
    }
    send_goal(goal);
  }

  void send_goal(const Action::Goal & goal)
  {
    if (!action_client_->action_server_is_ready()) {
      RCLCPP_WARN(get_logger(), "运镜Action服务未就绪，拒绝本次语音任务");
      return;
    }
    rclcpp_action::Client<Action>::SendGoalOptions options;
    options.goal_response_callback = [this](GoalHandle::SharedPtr handle) {
        goal_request_in_flight_ = false;
        if (!handle) {
          RCLCPP_WARN(get_logger(), "运镜Action拒绝了语音任务");
          cancel_when_accepted_ = false;
          send_pending_if_any();
          return;
        }
        active_goal_ = handle;
        if (cancel_when_accepted_) {
          cancel_when_accepted_ = false;
          action_client_->async_cancel_goal(active_goal_);
          return;
        }
        RCLCPP_INFO(get_logger(), "语音运镜任务已接受");
      };
    options.result_callback = [this](const GoalHandle::WrappedResult & result) {
        RCLCPP_INFO(
          get_logger(), "语音运镜任务结束: action_code=%d result=%u message=%s",
          static_cast<int>(result.code),
          result.result ? result.result->result_code : 255U,
          result.result ? result.result->message.c_str() : "no result");
        active_goal_.reset();
        send_pending_if_any();
      };
    goal_request_in_flight_ = true;
    action_client_->async_send_goal(goal, options);
  }

  void cancel_active(const char * reason, bool clear_pending)
  {
    if (clear_pending) {
      pending_goal_.reset();
    }
    if (!active_goal_) {
      if (goal_request_in_flight_) {
        cancel_when_accepted_ = true;
        RCLCPP_INFO(get_logger(), "%s: 等待Action接受后立即取消", reason);
      } else {
        RCLCPP_INFO(get_logger(), "%s: 当前没有活动运镜任务", reason);
      }
      return;
    }
    RCLCPP_INFO(get_logger(), "%s", reason);
    action_client_->async_cancel_goal(active_goal_);
  }

  void send_pending_if_any()
  {
    if (!pending_goal_) {
      return;
    }
    const auto next = *pending_goal_;
    pending_goal_.reset();
    send_goal(next);
  }

  void call_mode_service(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr & client,
    const char * description)
  {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s失败: 服务未就绪", description);
      return;
    }
    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(
      request,
      [this, description](
        rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        try {
          const auto response = future.get();
          if (response->success) {
            RCLCPP_INFO(
              get_logger(), "%s成功: %s", description,
              response->message.c_str());
          } else {
            RCLCPP_WARN(
              get_logger(), "%s被拒绝: %s", description,
              response->message.c_str());
          }
        } catch (const std::exception & error) {
          RCLCPP_ERROR(
            get_logger(), "%s异常: %s", description, error.what());
        }
      });
  }

  double min_confidence_{0.60};
  std::optional<Action::Goal> pending_goal_;
  GoalHandle::SharedPtr active_goal_;
  bool goal_request_in_flight_{false};
  bool cancel_when_accepted_{false};
  rclcpp::Subscription<VoiceCommand>::SharedPtr command_sub_;
  rclcpp_action::Client<Action>::SharedPtr action_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr enter_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr exit_client_;
  rclcpp::CallbackGroup::SharedPtr service_client_group_;
};

}  // namespace external_control_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::VoiceCinematicActionNode>());
  rclcpp::shutdown();
  return 0;
}
