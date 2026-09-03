#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

namespace
{

rclcpp::QoS transient_qos()
{
  return rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
}

}  // namespace

class Tron1ModeManagerNode : public rclcpp::Node
{
public:
  enum class Mode
  {
    Idle,
    DeveloperMode,
    DeveloperSelfCheck,
    RemoteStandReady,
    RemoteWalkReady,
    DeviceSelfCheck,
    GimbalFollow,
    TronFollow,
    Estop
  };

  Tron1ModeManagerNode() : Node("tron1_mode_manager")
  {
    request_topic_ = declare_parameter<std::string>("request_topic", "/tron1/mode_request");
    state_topic_ = declare_parameter<std::string>("state_topic", "/tron1/mode_state");
    motion_authorized_topic_ =
      declare_parameter<std::string>("motion_authorized_topic", "/tron1/motion_authorized");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/safety/estop_state");
    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 10.0);
    allow_walk_motion_ = declare_parameter<bool>("allow_walk_motion", false);
    allow_tron_follow_motion_ = declare_parameter<bool>("allow_tron_follow_motion", true);

    if (publish_rate_hz_ <= 0.0) {
      throw std::invalid_argument("publish_rate_hz must be positive");
    }

    state_pub_ = create_publisher<std_msgs::msg::String>(state_topic_, transient_qos());
    motion_auth_pub_ =
      create_publisher<std_msgs::msg::Bool>(motion_authorized_topic_, transient_qos());
    request_sub_ = create_subscription<std_msgs::msg::String>(
      request_topic_, rclcpp::QoS(10).reliable(),
      std::bind(&Tron1ModeManagerNode::on_request, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_, transient_qos(),
      std::bind(&Tron1ModeManagerNode::on_estop, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Tron1ModeManagerNode::publish_state, this));

    publish_state();
    RCLCPP_WARN(
      get_logger(),
      "TRON1 mode manager started. request=%s state=%s motion_authorized=%s. "
      "Default state is IDLE; motion is not authorized.",
      request_topic_.c_str(), state_topic_.c_str(), motion_authorized_topic_.c_str());
  }

private:
  static std::string mode_name(const Mode mode)
  {
    switch (mode) {
      case Mode::Idle:
        return "IDLE";
      case Mode::DeveloperMode:
        return "DEVELOPER_MODE";
      case Mode::DeveloperSelfCheck:
        return "DEVELOPER_SELF_CHECK";
      case Mode::RemoteStandReady:
        return "REMOTE_STAND_READY";
      case Mode::RemoteWalkReady:
        return "REMOTE_WALK_READY";
      case Mode::DeviceSelfCheck:
        return "DEVICE_SELF_CHECK";
      case Mode::GimbalFollow:
        return "GIMBAL_FOLLOW";
      case Mode::TronFollow:
        return "TRON_FOLLOW";
      case Mode::Estop:
        return "ESTOP";
    }
    return "UNKNOWN";
  }

  bool request_transition(const std::string & request)
  {
    if (request == "estop") {
      mode_ = Mode::Estop;
      estop_latched_ = true;
      return true;
    }

    if (request == "reset" || request == "idle") {
      if (external_estop_active_) {
        mode_ = Mode::Estop;
        estop_latched_ = true;
        return false;
      }
      mode_ = Mode::Idle;
      estop_latched_ = false;
      return true;
    }

    if (mode_ == Mode::Estop || estop_latched_ || external_estop_active_) {
      if (request == "clear_estop" && !external_estop_active_) {
        mode_ = Mode::Idle;
        estop_latched_ = false;
        return true;
      }
      return false;
    }

    if (request == "developer_mode" && mode_ == Mode::Idle) {
      mode_ = Mode::DeveloperMode;
      return true;
    }
    if (request == "developer_self_check_pass" && mode_ == Mode::DeveloperMode) {
      mode_ = Mode::DeveloperSelfCheck;
      return true;
    }
    if (request == "stand_ready" && mode_ == Mode::DeveloperSelfCheck) {
      mode_ = Mode::RemoteStandReady;
      return true;
    }
    if (request == "walk_ready" && mode_ == Mode::RemoteStandReady) {
      mode_ = Mode::RemoteWalkReady;
      return true;
    }
    if (request == "device_self_check_pass" && mode_ == Mode::RemoteWalkReady) {
      mode_ = Mode::DeviceSelfCheck;
      return true;
    }
    if (request == "gimbal_follow" && mode_ == Mode::DeviceSelfCheck) {
      mode_ = Mode::GimbalFollow;
      return true;
    }
    if (request == "tron_follow" && mode_ == Mode::GimbalFollow) {
      mode_ = Mode::TronFollow;
      return true;
    }
    if (request == "back_to_walk" && mode_ == Mode::GimbalFollow) {
      mode_ = Mode::RemoteWalkReady;
      return true;
    }
    if (request == "back_to_gimbal_follow" && mode_ == Mode::TronFollow) {
      mode_ = Mode::GimbalFollow;
      return true;
    }

    return false;
  }

  bool motion_authorized() const
  {
    if (mode_ == Mode::Estop || estop_latched_ || external_estop_active_) {
      return false;
    }
    if (mode_ == Mode::RemoteWalkReady) {
      return allow_walk_motion_;
    }
    if (mode_ == Mode::TronFollow) {
      return allow_tron_follow_motion_;
    }
    return false;
  }

  void on_request(const std_msgs::msg::String::ConstSharedPtr msg)
  {
    const auto old_mode = mode_name(mode_);
    const auto accepted = request_transition(msg->data);
    if (accepted) {
      RCLCPP_WARN(
        get_logger(), "mode request accepted: %s, %s -> %s",
        msg->data.c_str(), old_mode.c_str(), mode_name(mode_).c_str());
    } else {
      RCLCPP_ERROR(
        get_logger(), "mode request rejected: %s while state=%s estop_latched=%s external_estop=%s",
        msg->data.c_str(), mode_name(mode_).c_str(),
        estop_latched_ ? "true" : "false", external_estop_active_ ? "true" : "false");
    }
    publish_state();
  }

  void on_estop(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    external_estop_active_ = msg->data;
    if (external_estop_active_) {
      mode_ = Mode::Estop;
      estop_latched_ = true;
      RCLCPP_ERROR(get_logger(), "external estop active; mode forced to ESTOP");
    }
    publish_state();
  }

  void publish_state()
  {
    std_msgs::msg::String state;
    state.data = mode_name(mode_);
    state_pub_->publish(state);

    std_msgs::msg::Bool auth;
    auth.data = motion_authorized();
    motion_auth_pub_->publish(auth);
  }

  Mode mode_ = Mode::Idle;
  bool estop_latched_ = false;
  bool external_estop_active_ = false;
  bool allow_walk_motion_ = false;
  bool allow_tron_follow_motion_ = true;
  double publish_rate_hz_ = 10.0;

  std::string request_topic_;
  std::string state_topic_;
  std::string motion_authorized_topic_;
  std::string estop_topic_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_auth_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr request_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tron1ModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}
