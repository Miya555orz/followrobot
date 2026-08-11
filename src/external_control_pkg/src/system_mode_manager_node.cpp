#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_servo_msgs/msg/system_state.hpp>
#include <vision_servo_msgs/srv/set_system_mode.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>

namespace external_control_pkg {

class SystemModeManagerNode : public rclcpp::Node {
public:
  using State = vision_servo_msgs::msg::SystemState;
  using SetMode = vision_servo_msgs::srv::SetSystemMode;

  SystemModeManagerNode() : Node("system_mode_manager") {
    state_pub_ = create_publisher<State>(
        "/system/state", rclcpp::QoS(1).reliable().transient_local());
    teleop_mode_pub_ = create_publisher<std_msgs::msg::String>(
        "/teleop/mode", rclcpp::QoS(5).reliable());
    set_mode_service_ = create_service<SetMode>(
        "/system/set_mode",
        std::bind(&SystemModeManagerNode::set_mode, this,
                  std::placeholders::_1, std::placeholders::_2));

    cinematic_status_sub_ = create_subscription<std_msgs::msg::String>(
        "/cinematic/status", rclcpp::QoS(10).reliable(),
        std::bind(&SystemModeManagerNode::cinematic_status, this,
                  std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/safety/estop_state", rclcpp::QoS(1).reliable(),
        [this](const std_msgs::msg::Bool::ConstSharedPtr msg) {
          if (emergency_stop_ != msg->data) {
            emergency_stop_ = msg->data;
            if (emergency_stop_) {
              apply_mode(State::MODE_STANDBY, "emergency_stop");
            } else {
              publish_state("emergency_stop_cleared");
            }
          }
        });

    cinematic_enter_ = create_client<std_srvs::srv::Trigger>("/cinematic/enter");
    cinematic_exit_ = create_client<std_srvs::srv::Trigger>("/cinematic/exit");
    cinematic_stop_ = create_client<std_srvs::srv::Trigger>("/cinematic/stop");
    heartbeat_ = create_wall_timer(std::chrono::seconds(1), [this]() {
      // /system/state is the business-state heartbeat.  Do not repeatedly
      // overwrite /teleop/mode here: the cinematic manager owns its internal
      // stop -> auto hand-over while entering CINEMATIC.
      publish_state(detail_);
    });
    apply_mode(State::MODE_STANDBY, "startup");
  }

private:
  static const char* mode_name(uint8_t mode) {
    switch (mode) {
      case State::MODE_FOLLOW: return "FOLLOW";
      case State::MODE_CINEMATIC: return "CINEMATIC";
      default: return "STANDBY";
    }
  }

  void set_mode(const SetMode::Request::SharedPtr request,
                SetMode::Response::SharedPtr response) {
    if (request->mode > State::MODE_CINEMATIC) {
      response->success = false;
      response->current_mode = mode_;
      response->state_version = state_version_;
      response->message = "unsupported_mode";
      return;
    }
    if (emergency_stop_ && request->mode != State::MODE_STANDBY) {
      response->success = false;
      response->current_mode = mode_;
      response->state_version = state_version_;
      response->message = "emergency_stop_active";
      return;
    }
    if (request->mode == mode_) {
      response->success = true;
      response->current_mode = mode_;
      response->state_version = state_version_;
      response->message = "mode_already_active";
      return;
    }
    if (request->mode == State::MODE_CINEMATIC &&
        !cinematic_enter_->service_is_ready()) {
      response->success = false;
      response->current_mode = mode_;
      response->state_version = state_version_;
      response->message = "cinematic_enter_service_unavailable";
      return;
    }
    apply_mode(request->mode, request->reason.empty() ? "mode_request" : request->reason);
    response->success = true;
    response->current_mode = mode_;
    response->state_version = state_version_;
    response->message = "mode_applied";
  }

  void apply_mode(uint8_t requested, const std::string& reason) {
    if (requested == State::MODE_CINEMATIC) {
      publish_mux_mode("stop");
      request_trigger(cinematic_enter_, "cinematic_enter");
      cinematic_state_ = State::CINEMATIC_INACTIVE;
    } else {
      if (mode_ == State::MODE_CINEMATIC) {
        request_trigger(cinematic_stop_, "cinematic_stop");
        request_trigger(cinematic_exit_, "cinematic_exit");
      }
      cinematic_state_ = State::CINEMATIC_INACTIVE;
      publish_mux_mode(requested == State::MODE_FOLLOW ? "auto" : "manual");
    }
    if (mode_ != requested || detail_ != reason) {
      mode_ = requested;
      detail_ = reason;
      ++state_version_;
    }
    publish_state(reason);
  }

  void request_trigger(
      const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
      const std::string& operation) {
    if (!client->service_is_ready()) {
      RCLCPP_WARN(get_logger(), "%s service is not ready", operation.c_str());
      return;
    }
    client->async_send_request(
        std::make_shared<std_srvs::srv::Trigger::Request>(),
        [this, operation](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
          const auto response = future.get();
          if (!response->success) {
            RCLCPP_ERROR(get_logger(), "%s rejected: %s", operation.c_str(),
                         response->message.c_str());
            if (operation == "cinematic_enter" && mode_ == State::MODE_CINEMATIC) {
              apply_mode(State::MODE_STANDBY, "cinematic_enter_failed");
            }
          }
        });
  }

  void cinematic_status(const std_msgs::msg::String::ConstSharedPtr message) {
    if (mode_ != State::MODE_CINEMATIC) return;
    uint8_t next = cinematic_state_;
    if (message->data.find("EXECUTING") != std::string::npos) {
      next = State::CINEMATIC_EXECUTING;
    } else if (message->data.find("READY") != std::string::npos) {
      next = State::CINEMATIC_READY;
    }
    if (next != cinematic_state_) {
      cinematic_state_ = next;
      detail_ = "cinematic_status_update";
      ++state_version_;
      publish_state(detail_);
    }
  }

  void publish_mux_mode(const std::string& value) {
    std_msgs::msg::String message;
    message.data = value;
    teleop_mode_pub_->publish(message);
  }

  void publish_state(const std::string& detail) {
    State state;
    state.header.stamp = now();
    state.header.frame_id = "system_mode_manager";
    state.mode = mode_;
    state.cinematic_state = cinematic_state_;
    state.state_version = state_version_;
    state.emergency_stop = emergency_stop_;
    state.mode_name = mode_name(mode_);
    state.detail = detail;
    state_pub_->publish(state);
  }

  uint8_t mode_{State::MODE_STANDBY};
  uint8_t cinematic_state_{State::CINEMATIC_INACTIVE};
  uint64_t state_version_{0};
  bool emergency_stop_{false};
  std::string detail_{"startup"};
  rclcpp::Publisher<State>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr teleop_mode_pub_;
  rclcpp::Service<SetMode>::SharedPtr set_mode_service_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cinematic_status_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cinematic_enter_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cinematic_exit_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr cinematic_stop_;
  rclcpp::TimerBase::SharedPtr heartbeat_;
};

}  // namespace external_control_pkg

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<external_control_pkg::SystemModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
