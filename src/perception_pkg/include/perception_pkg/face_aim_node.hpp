#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include <image_transport/image_transport.hpp>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_servo_msgs/msg/aim_target2_d.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>

#include "perception_pkg/qos.hpp"

namespace perception_pkg {

class FaceAimNode : public rclcpp::Node {
public:
  explicit FaceAimNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
  void tracks_callback(const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg);
  void try_match_and_process();
  void check_timeout();
  void publish_debug_image(
      const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
      const std_msgs::msg::Header& header,
      const vision_servo_msgs::msg::AimTarget2D& aim);
  rclcpp::Publisher<vision_servo_msgs::msg::AimTarget2D>::SharedPtr aim_pub_;
  image_transport::Subscriber image_sub_;
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr tracks_sub_;
  image_transport::Publisher debug_pub_;
  rclcpp::TimerBase::SharedPtr timeout_timer_;

  std::mutex mutex_;
  sensor_msgs::msg::Image::ConstSharedPtr pending_image_;
  vision_servo_msgs::msg::TargetArray::ConstSharedPtr pending_tracks_;
  std::chrono::steady_clock::time_point last_tracks_time_;

  double input_timeout_seconds_ = 2.0;
  bool publish_debug_ = true;
  bool was_in_timeout_ = false;

  double aim_offset_ratio_ = 0.20;
  double alpha_beta_alpha_ = 0.65;
  double alpha_beta_beta_ = 0.08;
  double covariance_alpha_ = 0.15;
  double initial_covariance_px2_ = 16.0;
  double lost_covariance_growth_px2_per_sec_ = 400.0;
  double min_filter_dt_seconds_ = 0.005;
  double max_filter_dt_seconds_ = 0.20;
  float filtered_x_ = 0.0f;
  float filtered_y_ = 0.0f;
  float filtered_vx_ = 0.0f;
  float filtered_vy_ = 0.0f;
  float covariance_x_ = 16.0f;
  float covariance_y_ = 16.0f;
  bool filter_initialized_ = false;
  int last_tracking_id_ = -1;
  rclcpp::Time last_filter_stamp_{0, 0, RCL_ROS_TIME};
  builtin_interfaces::msg::Time last_visible_source_stamp_;
};

}  // namespace perception_pkg
