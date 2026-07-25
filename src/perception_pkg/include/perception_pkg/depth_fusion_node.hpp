#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_servo_msgs/msg/target_array.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/core.hpp>

namespace perception_pkg
{

class DepthFusionNode : public rclcpp::Node
{
public:
  explicit DepthFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // --- subscriptions ---
  void tracks_callback(const vision_servo_msgs::msg::TargetArray::SharedPtr msg);
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void sony_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // --- core algorithm ---
  void process_frame();

  bool project_bbox_to_depth(
    const vision_servo_msgs::msg::Target & target,
    cv::Rect & roi,
    double assume_depth_m);

  float sample_depth_median(
    const cv::Mat & depth_img,
    const cv::Rect & roi,
    const vision_servo_msgs::msg::Target & target,
    float & valid_ratio) const;

  bool back_project(
    float u, float v, float depth_m,
    const cv::Mat & K,
    cv::Vec3f & p3d) const;

  // --- state ---
  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sony_info_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr targets_3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // cached data
  vision_servo_msgs::msg::TargetArray::SharedPtr last_tracks_;
  cv::Mat depth_image_;
  rclcpp::Time depth_stamp_;
  cv::Mat K_sony_;
  cv::Mat D_sony_;
  cv::Mat K_depth_;
  cv::Mat D_depth_;
  bool sony_info_ready_ = false;
  bool depth_info_ready_ = false;

  // parameters
  std::string sony_frame_;
  std::string gemini_depth_frame_;
  double default_depth_assume_;
  double roi_expand_margin_;
  int min_roi_pixels_;
  double min_depth_m_;
  double max_depth_m_;
  double min_valid_depth_ratio_;
  double depth_percentile_;
  double depth_temporal_alpha_;
  bool publish_debug_;
  double tracks_timeout_s_;
  double depth_timeout_s_;

  // temporal filter state
  int last_target_id_ = -1;
  double depth_filtered_ = 0.0;
};

}  // namespace perception_pkg
