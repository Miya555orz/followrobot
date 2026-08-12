#include "external_control_pkg/msg/voice_command.hpp"
#include "external_control_pkg/msg/voice_dispatch_status.hpp"
#include "external_control_pkg/voice_intent_contract.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_servo_msgs/msg/camera_recording_state.hpp>
#include <vision_servo_msgs/msg/system_state.hpp>
#include <vision_servo_msgs/srv/set_camera_recording.hpp>
#include <vision_servo_msgs/srv/set_follow_distance.hpp>
#include <vision_servo_msgs/srv/set_system_mode.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <sstream>
#include <string>

namespace external_control_pkg {

class VoiceCommandDispatcherNode : public rclcpp::Node {
public:
  using Command = msg::VoiceCommand;
  using Status = msg::VoiceDispatchStatus;
  using SystemState = vision_servo_msgs::msg::SystemState;
  using RecordingState = vision_servo_msgs::msg::CameraRecordingState;
  using SetMode = vision_servo_msgs::srv::SetSystemMode;
  using SetRecording = vision_servo_msgs::srv::SetCameraRecording;
  using SetFollowDistance = vision_servo_msgs::srv::SetFollowDistance;

  VoiceCommandDispatcherNode() : Node("voice_command_dispatcher_node") {
    declare_parameter("input_topic", "/external/voice_command");
    declare_parameter("gimbal_topic", "/voice/gimbal_command");
    declare_parameter("chassis_topic", "/voice/chassis_command");
    declare_parameter("autonomy_topic", "/voice/autonomy_command");
    declare_parameter("status_topic", "/voice/dispatch_status");
    declare_parameter("min_confidence", 0.6);
    declare_parameter("duplicate_window_sec", 0.5);
    declare_parameter("max_command_age_sec", 2.0);

    input_topic_ = get_parameter("input_topic").as_string();
    min_confidence_ = get_parameter("min_confidence").as_double();
    duplicate_window_sec_ = std::max(0.0, get_parameter("duplicate_window_sec").as_double());
    max_command_age_sec_ = std::max(0.0, get_parameter("max_command_age_sec").as_double());
    const auto qos = rclcpp::QoS(10).reliable();
    publishers_[VoiceTarget::Gimbal] = create_publisher<Command>(
        get_parameter("gimbal_topic").as_string(), qos);
    publishers_[VoiceTarget::Chassis] = create_publisher<Command>(
        get_parameter("chassis_topic").as_string(), qos);
    publishers_[VoiceTarget::Autonomy] = create_publisher<Command>(
        get_parameter("autonomy_topic").as_string(), qos);
    status_pub_ = create_publisher<Status>(get_parameter("status_topic").as_string(), qos);
    estop_pub_ = create_publisher<std_msgs::msg::Bool>("/teleop/estop", qos);
    mux_mode_pub_ = create_publisher<std_msgs::msg::String>("/teleop/mode", qos);

    input_sub_ = create_subscription<Command>(
        input_topic_, qos,
        std::bind(&VoiceCommandDispatcherNode::command_callback, this,
                  std::placeholders::_1));
    system_state_sub_ = create_subscription<SystemState>(
        "/system/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](const SystemState::ConstSharedPtr state) {
          const bool leaving_cinematic = have_system_state_ &&
              system_state_.mode == SystemState::MODE_CINEMATIC &&
              state->mode != SystemState::MODE_CINEMATIC;
          system_state_ = *state;
          have_system_state_ = true;
          if (leaving_cinematic && cinematic_started_recording_) {
            request_recording(false, "cinematic_voice_release");
            cinematic_started_recording_ = false;
          }
        });
    recording_state_sub_ = create_subscription<RecordingState>(
        "/sony/recording_status", rclcpp::QoS(1).reliable().transient_local(),
        [this](const RecordingState::ConstSharedPtr state) {
          recording_state_ = state->state;
          if (state->state == RecordingState::RECORDING && pending_cinematic_) {
            publishers_.at(VoiceTarget::Autonomy)->publish(*pending_cinematic_);
            pending_cinematic_.reset();
            publish_status("voice_orchestrator", "autonomy", "cinematic_pending",
                           1.0F, true, "recording_confirmed_command_released");
          }
        });
    set_mode_client_ = create_client<SetMode>("/system/set_mode");
    set_recording_client_ = create_client<SetRecording>("/sony/set_recording");
    take_photo_client_ = create_client<std_srvs::srv::Trigger>("/sony/take_photo");
    follow_distance_client_ = create_client<SetFollowDistance>("/follow/set_distance");
    pending_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {
      if (pending_cinematic_ && now() > pending_deadline_) {
        pending_cinematic_.reset();
        publish_status("voice_orchestrator", "autonomy", "cinematic_pending",
                       0.0F, false, "recording_confirmation_timeout");
      }
    });
    RCLCPP_INFO(get_logger(), "Jetson voice intent gate started; external classifier is candidate-only");
  }

private:
  static bool is_cinematic_action(const std::string& intent) {
    return intent == "start_orbit" || intent == "start_dolly" ||
           intent == "start_truck" || intent == "start_static_track";
  }

  bool stale(const Command& command) const {
    if (max_command_age_sec_ <= 0.0 ||
        (command.header.stamp.sec == 0 && command.header.stamp.nanosec == 0)) return false;
    const double age = (now() - rclcpp::Time(command.header.stamp)).seconds();
    return age > max_command_age_sec_ || age < -0.5;
  }

  bool duplicate(const Command& command) {
    if (duplicate_window_sec_ <= 0.0) return false;
    std::ostringstream key;
    key << command.header.frame_id << '|' << command.raw_text;
    for (const auto& intent : command.intents) key << '|' << intent;
    const auto current = now();
    const bool result = key.str() == last_key_ &&
        (current - last_command_time_).seconds() <= duplicate_window_sec_;
    last_key_ = key.str();
    last_command_time_ = current;
    return result;
  }

  Command single_intent(const Command& source, size_t index) const {
    Command result = source;
    result.intents = {source.intents[index]};
    result.confidences = {index < source.confidences.size() ? source.confidences[index] : 0.0F};
    return result;
  }

  void command_callback(const Command::ConstSharedPtr command) {
    if (command->intents.empty()) {
      publish_status(command->header.frame_id, "unknown", "", 0.0F, false, "empty_intents");
      return;
    }
    if (stale(*command)) {
      for (size_t i = 0; i < command->intents.size(); ++i)
        publish_for(*command, i, VoiceTarget::Unknown, false, "stale_command");
      return;
    }
    if (duplicate(*command)) {
      for (size_t i = 0; i < command->intents.size(); ++i)
        publish_for(*command, i, VoiceTarget::Unknown, false, "duplicate_suppressed");
      return;
    }
    for (size_t i = 0; i < command->intents.size(); ++i) dispatch(*command, i);
  }

  void dispatch(const Command& command, size_t index) {
    const auto& intent = command.intents[index];
    const float confidence = index < command.confidences.size() ? command.confidences[index] : 0.0F;
    const auto target = intentTarget(intent);
    if (isAmbiguousStopIntent(intent)) {
      publish_for(command, index, VoiceTarget::Unknown, false, "ambiguous_stop_requires_target");
      return;
    }
    if (target == VoiceTarget::Unknown) {
      publish_for(command, index, target, false, "unknown_intent");
      return;
    }
    if (confidence < min_confidence_) {
      publish_for(command, index, target, false, "confidence_below_threshold");
      return;
    }
    if (intent == "emergency_stop") {
      std_msgs::msg::Bool stop; stop.data = true; estop_pub_->publish(stop);
      publish_for(command, index, target, true, "emergency_stop_latched");
      return;
    }
    if (!have_system_state_) {
      publish_for(command, index, target, false, "system_state_unavailable");
      return;
    }
    if (intent == "stop_all") {
      std_msgs::msg::String stop; stop.data = "stop"; mux_mode_pub_->publish(stop);
      request_mode(SystemState::MODE_STANDBY, "voice_stop_all");
      publish_for(command, index, target, true, "standby_requested");
      return;
    }
    if (intent == "status_query" || intent == "query_camera_status" ||
        intent == "query_chassis_status" || intent == "query_gimbal_status" ||
        intent == "query_camera_motion_status") {
      publish_for(command, index, target, true, "query_acknowledged");
      return;
    }
    if (target == VoiceTarget::Gimbal || target == VoiceTarget::Chassis) {
      if (system_state_.mode != SystemState::MODE_STANDBY) {
        publish_for(command, index, target, false, "manual_jog_only_allowed_in_standby");
        return;
      }
      publishers_.at(target)->publish(single_intent(command, index));
      publish_for(command, index, target, true, "standby_manual_jog_routed");
      return;
    }
    if (target == VoiceTarget::Camera) {
      dispatch_camera(command, index, intent);
      return;
    }
    dispatch_autonomy(command, index, intent);
  }

  void dispatch_camera(const Command& command, size_t index, const std::string& intent) {
    if (intent == "camera_stop_recording" &&
        system_state_.mode == SystemState::MODE_CINEMATIC &&
        system_state_.cinematic_state == SystemState::CINEMATIC_EXECUTING) {
      publish_for(command, index, VoiceTarget::Camera, false, "recording_locked_by_cinematic_action");
      return;
    }
    if (intent == "camera_take_photo") {
      if (system_state_.mode == SystemState::MODE_CINEMATIC &&
          system_state_.cinematic_state == SystemState::CINEMATIC_EXECUTING) {
        publish_for(command, index, VoiceTarget::Camera, false,
                    "photo_rejected_while_cinematic_executing");
        return;
      }
      if (!take_photo_client_->service_is_ready()) {
        publish_for(command, index, VoiceTarget::Camera, false, "camera_service_unavailable");
        return;
      }
      take_photo_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
      publish_for(command, index, VoiceTarget::Camera, true, "photo_requested");
      return;
    }
    const bool start = intent == "camera_start_recording";
    if (!start && intent != "camera_stop_recording") {
      publish_for(command, index, VoiceTarget::Camera, false, "unsupported_camera_intent");
      return;
    }
    if (!request_recording(start, "voice")) {
      publish_for(command, index, VoiceTarget::Camera, false, "camera_service_unavailable");
      return;
    }
    cinematic_started_recording_ = false;
    publish_for(command, index, VoiceTarget::Camera, true,
                start ? "recording_start_requested" : "recording_stop_requested");
  }

  void dispatch_autonomy(const Command& command, size_t index, const std::string& intent) {
    if (intent == "start_following") {
      if (system_state_.mode == SystemState::MODE_CINEMATIC) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "exit_cinematic_before_follow");
      } else {
        request_mode(SystemState::MODE_FOLLOW, "voice_start_following");
        publish_for(command, index, VoiceTarget::Autonomy, true, "follow_requested");
      }
      return;
    }
    if (intent == "stop_following") {
      if (system_state_.mode != SystemState::MODE_FOLLOW) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "not_in_follow_mode");
      } else {
        request_mode(SystemState::MODE_STANDBY, "voice_stop_following");
        publish_for(command, index, VoiceTarget::Autonomy, true, "standby_requested");
      }
      return;
    }
    if (intent == "enter_cinematic") {
      if (system_state_.mode != SystemState::MODE_STANDBY) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "cinematic_entry_requires_standby");
      } else {
        request_mode(SystemState::MODE_CINEMATIC, "voice_enter_cinematic");
        publish_for(command, index, VoiceTarget::Autonomy, true, "cinematic_entry_requested");
      }
      return;
    }
    if (intent == "exit_cinematic") {
      if (system_state_.mode != SystemState::MODE_CINEMATIC) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "not_in_cinematic_mode");
      } else {
        request_mode(SystemState::MODE_STANDBY, "voice_exit_cinematic");
        publish_for(command, index, VoiceTarget::Autonomy, true, "cinematic_exit_requested");
      }
      return;
    }
    if (intent == "distance_adjust") {
      if (system_state_.mode != SystemState::MODE_FOLLOW) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "distance_adjust_requires_follow");
      } else {
        const bool valid_distance = std::isfinite(command.distance) &&
          (command.distance_relative ? std::abs(command.distance) > 0.0F : command.distance > 0.0F);
        if (!valid_distance || !follow_distance_client_->service_is_ready()) {
          publish_for(command, index, VoiceTarget::Autonomy, false,
                      !valid_distance ? "distance_parameter_missing" : "follow_distance_service_unavailable");
        } else {
          auto request = std::make_shared<SetFollowDistance::Request>();
          request->distance_m = command.unit == "cm" ? command.distance / 100.0F : command.distance;
          request->relative = command.distance_relative;
          follow_distance_client_->async_send_request(request);
          publish_for(command, index, VoiceTarget::Autonomy, true, "follow_distance_requested");
        }
      }
      return;
    }
    if (is_cinematic_action(intent)) {
      if (system_state_.mode != SystemState::MODE_CINEMATIC ||
          system_state_.cinematic_state != SystemState::CINEMATIC_READY) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "cinematic_action_requires_ready_state");
        return;
      }
      auto filtered = single_intent(command, index);
      if (recording_state_ == RecordingState::RECORDING) {
        publishers_.at(VoiceTarget::Autonomy)->publish(filtered);
        publish_for(command, index, VoiceTarget::Autonomy, true, "cinematic_action_routed");
      } else if (request_recording(true, "cinematic_voice_interlock")) {
        cinematic_started_recording_ = true;
        pending_cinematic_ = filtered;
        pending_deadline_ = now() + rclcpp::Duration::from_seconds(8.0);
        publish_for(command, index, VoiceTarget::Autonomy, true, "waiting_for_recording_confirmation");
      } else {
        publish_for(command, index, VoiceTarget::Autonomy, false, "recording_interlock_unavailable");
      }
      return;
    }
    if (intent == "stop_cinematic") {
      if (system_state_.mode != SystemState::MODE_CINEMATIC) {
        publish_for(command, index, VoiceTarget::Autonomy, false, "not_in_cinematic_mode");
      } else {
        publishers_.at(VoiceTarget::Autonomy)->publish(single_intent(command, index));
        publish_for(command, index, VoiceTarget::Autonomy, true, "cinematic_stop_routed");
      }
      return;
    }
    publish_for(command, index, VoiceTarget::Autonomy, false, "unsupported_or_disallowed_in_current_mode");
  }

  void request_mode(uint8_t mode, const std::string& reason) {
    if (!set_mode_client_->service_is_ready()) return;
    auto request = std::make_shared<SetMode::Request>();
    request->mode = mode;
    request->reason = reason;
    set_mode_client_->async_send_request(request);
  }

  bool request_recording(bool recording, const std::string& requester) {
    if (!set_recording_client_->service_is_ready()) return false;
    auto request = std::make_shared<SetRecording::Request>();
    request->recording = recording;
    request->requester = requester;
    set_recording_client_->async_send_request(request);
    return true;
  }

  void publish_for(const Command& command, size_t index, VoiceTarget target,
                   bool accepted, const std::string& reason) {
    publish_status(command.header.frame_id, targetName(target),
                   index < command.intents.size() ? command.intents[index] : "",
                   index < command.confidences.size() ? command.confidences[index] : 0.0F,
                   accepted, reason);
  }

  void publish_status(const std::string& source, const std::string& target,
                      const std::string& intent, float confidence,
                      bool accepted, const std::string& reason) {
    Status status;
    status.header.stamp = now();
    status.header.frame_id = "voice_orchestrator";
    status.source = source;
    status.target = target;
    status.intent = intent;
    status.confidence = confidence;
    status.accepted = accepted;
    status.reason = reason;
    status_pub_->publish(status);
  }

  std::string input_topic_;
  double min_confidence_{0.6};
  double duplicate_window_sec_{0.5};
  double max_command_age_sec_{2.0};
  std::string last_key_;
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  bool have_system_state_{false};
  SystemState system_state_;
  uint8_t recording_state_{RecordingState::UNKNOWN};
  bool cinematic_started_recording_{false};
  std::optional<Command> pending_cinematic_;
  rclcpp::Time pending_deadline_{0, 0, RCL_ROS_TIME};
  std::map<VoiceTarget, rclcpp::Publisher<Command>::SharedPtr> publishers_;
  rclcpp::Publisher<Status>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr estop_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mux_mode_pub_;
  rclcpp::Subscription<Command>::SharedPtr input_sub_;
  rclcpp::Subscription<SystemState>::SharedPtr system_state_sub_;
  rclcpp::Subscription<RecordingState>::SharedPtr recording_state_sub_;
  rclcpp::Client<SetMode>::SharedPtr set_mode_client_;
  rclcpp::Client<SetRecording>::SharedPtr set_recording_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr take_photo_client_;
  rclcpp::Client<SetFollowDistance>::SharedPtr follow_distance_client_;
  rclcpp::TimerBase::SharedPtr pending_timer_;
};

}  // namespace external_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::VoiceCommandDispatcherNode>());
  rclcpp::shutdown();
  return 0;
}
