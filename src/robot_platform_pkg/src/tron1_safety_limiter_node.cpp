#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

namespace
{

double clamp_abs(double value, double limit)
{
  const auto abs_limit = std::abs(limit);
  return std::clamp(value, -abs_limit, abs_limit);
}

double step_toward(double current, double target, double max_step)
{
  const auto delta = target - current;
  if (std::abs(delta) <= max_step) {
    return target;
  }
  return current + std::copysign(max_step, delta);
}

}  // namespace

class Tron1SafetyLimiterNode : public rclcpp::Node
{
public:
  Tron1SafetyLimiterNode() : Node("tron1_safety_limiter")
  {
    const auto input_topic = declare_parameter<std::string>(
      "input_topic", "/fcr/cmd_vel_stamped");
    const auto output_topic = declare_parameter<std::string>(
      "output_topic", "/fcr_tron/cmd_vel");
    estop_topic_ = declare_parameter<std::string>("estop_topic", "/safety/estop_state");

    publish_rate_hz_ = declare_parameter<double>("publish_rate_hz", 50.0);
    input_timeout_sec_ = declare_parameter<double>("input_timeout_sec", 0.30);
    max_linear_x_ = declare_parameter<double>("max_linear_x", 0.10);
    max_linear_y_ = declare_parameter<double>("max_linear_y", 0.0);
    max_angular_z_ = declare_parameter<double>("max_angular_z", 0.30);
    max_accel_x_ = declare_parameter<double>("max_accel_x", 0.20);
    max_accel_y_ = declare_parameter<double>("max_accel_y", 0.20);
    max_accel_yaw_ = declare_parameter<double>("max_accel_yaw", 0.60);
    enable_motion_ = declare_parameter<bool>("enable_motion", false);
    enable_lateral_ = declare_parameter<bool>("enable_lateral", false);
    stop_immediately_on_estop_ = declare_parameter<bool>("stop_immediately_on_estop", true);

    if (publish_rate_hz_ <= 0.0 || input_timeout_sec_ <= 0.0 ||
      max_accel_x_ <= 0.0 || max_accel_y_ <= 0.0 || max_accel_yaw_ <= 0.0)
    {
      throw std::invalid_argument(
        "publish_rate_hz, input_timeout_sec and acceleration limits must be positive");
    }

    const auto command_qos = rclcpp::QoS(1).reliable().durability_volatile();
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic, command_qos);
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_topic, command_qos,
      std::bind(&Tron1SafetyLimiterNode::on_cmd, this, std::placeholders::_1));
    estop_sub_ = create_subscription<std_msgs::msg::Bool>(
      estop_topic_, rclcpp::QoS(1).reliable().transient_local(),
      std::bind(&Tron1SafetyLimiterNode::on_estop, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&Tron1SafetyLimiterNode::publish_limited_cmd, this));
    last_publish_time_ = now();

    RCLCPP_WARN(
      get_logger(),
      "TRON1 safety limiter: %s -> %s, enable_motion=%s, lateral=%s, "
      "limits x=%.3f y=%.3f yaw=%.3f, timeout=%.2fs",
      input_topic.c_str(), output_topic.c_str(),
      enable_motion_ ? "true" : "false",
      enable_lateral_ ? "true" : "false",
      max_linear_x_, enable_lateral_ ? max_linear_y_ : 0.0,
      max_angular_z_, input_timeout_sec_);
  }

private:
  void on_cmd(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
  {
    target_cmd_.linear.x = clamp_abs(msg->twist.linear.x, max_linear_x_);
    target_cmd_.linear.y =
      enable_lateral_ ? clamp_abs(msg->twist.linear.y, max_linear_y_) : 0.0;
    target_cmd_.angular.z = clamp_abs(msg->twist.angular.z, max_angular_z_);
    last_cmd_time_ = now();
    have_cmd_ = true;
  }

  void on_estop(const std_msgs::msg::Bool::ConstSharedPtr msg)
  {
    estop_active_ = msg->data;
    if (estop_active_ && stop_immediately_on_estop_) {
      target_cmd_ = geometry_msgs::msg::Twist();
      current_cmd_ = geometry_msgs::msg::Twist();
      cmd_pub_->publish(current_cmd_);
      RCLCPP_ERROR(get_logger(), "TRON1 safety limiter emergency stop active; output zeroed");
    }
  }

  void publish_limited_cmd()
  {
    auto requested = geometry_msgs::msg::Twist();
    const auto fresh = have_cmd_ && ((now() - last_cmd_time_).seconds() <= input_timeout_sec_);
    if (enable_motion_ && fresh && !estop_active_) {
      requested = target_cmd_;
    }

    const auto stamp = now();
    const auto dt = std::max(0.0, (stamp - last_publish_time_).seconds());
    last_publish_time_ = stamp;

    current_cmd_.linear.x = step_toward(
      current_cmd_.linear.x, requested.linear.x, max_accel_x_ * dt);
    current_cmd_.linear.y = step_toward(
      current_cmd_.linear.y, requested.linear.y, max_accel_y_ * dt);
    current_cmd_.angular.z = step_toward(
      current_cmd_.angular.z, requested.angular.z, max_accel_yaw_ * dt);

    current_cmd_.linear.y = enable_lateral_ ? current_cmd_.linear.y : 0.0;
    cmd_pub_->publish(current_cmd_);
  }

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr estop_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  geometry_msgs::msg::Twist target_cmd_;
  geometry_msgs::msg::Twist current_cmd_;
  rclcpp::Time last_cmd_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_publish_time_{0, 0, RCL_ROS_TIME};

  std::string estop_topic_;
  bool have_cmd_ = false;
  bool estop_active_ = false;
  bool enable_motion_ = false;
  bool enable_lateral_ = false;
  bool stop_immediately_on_estop_ = true;
  double publish_rate_hz_ = 50.0;
  double input_timeout_sec_ = 0.30;
  double max_linear_x_ = 0.10;
  double max_linear_y_ = 0.0;
  double max_angular_z_ = 0.30;
  double max_accel_x_ = 0.20;
  double max_accel_y_ = 0.20;
  double max_accel_yaw_ = 0.60;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Tron1SafetyLimiterNode>());
  rclcpp::shutdown();
  return 0;
}
