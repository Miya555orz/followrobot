#pragma once

#include <rclcpp/rclcpp.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vision_servo_msgs/msg/target_array.hpp>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>

#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace perception_pkg
{

class DepthFusionNode : public rclcpp::Node
{
public:
  explicit DepthFusionNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  struct DepthSample
  {
    cv::Point2f pixel_sony{0.0F, 0.0F};
    cv::Vec3f point_sony{0.0F, 0.0F, 0.0F};
  };

  struct TargetFilterState
  {
    cv::Vec3f position{0.0F, 0.0F, 0.0F};
    cv::Vec3f velocity{0.0F, 0.0F, 0.0F};
    rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
    bool initialized{false};
  };

  void tracks_callback(const vision_servo_msgs::msg::TargetArray::SharedPtr msg);
  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  void depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);
  void sony_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  void process_frame(
    const vision_servo_msgs::msg::TargetArray & tracks,
    const cv::Mat & depth_meters,
    const rclcpp::Time & depth_stamp,
    const std::string & depth_frame);

  bool lookup_depth_to_sony(
    const rclcpp::Time & stamp,
    const std::string & depth_frame,
    Eigen::Isometry3d & transform) const;

  std::vector<DepthSample> reproject_depth_points(
    const cv::Mat & depth_meters,
    const Eigen::Isometry3d & depth_to_sony) const;

  bool estimate_target_position(
    const vision_servo_msgs::msg::Target & target,
    const std::vector<DepthSample> & points,
    cv::Vec3f & position,
    float & confidence,
    int & sample_count) const;

  void update_filter(
    int target_id,
    const rclcpp::Time & stamp,
    const cv::Vec3f & measured_position,
    cv::Vec3f & filtered_position,
    cv::Vec3f & filtered_velocity);

  void prune_filters(const rclcpp::Time & stamp);

  void publish_debug_image(
    const vision_servo_msgs::msg::TargetArray & output,
    const std::vector<DepthSample> & points,
    const std_msgs::msg::Header & header);
  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper & status);

  rclcpp::Subscription<vision_servo_msgs::msg::TargetArray>::SharedPtr tracks_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sony_info_sub_;
  rclcpp::Publisher<vision_servo_msgs::msg::TargetArray>::SharedPtr targets_3d_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  diagnostic_updater::Updater diagnostics_;

  mutable std::mutex data_mutex_;
  cv::Mat depth_image_meters_;
  rclcpp::Time depth_stamp_{0, 0, RCL_ROS_TIME};
  std::string depth_frame_from_message_;
  cv::Mat K_sony_;
  cv::Mat D_sony_;
  cv::Mat K_depth_;
  cv::Mat D_depth_;
  uint32_t sony_width_{0};
  uint32_t sony_height_{0};
  bool sony_info_ready_{false};
  bool depth_info_ready_{false};

  std::string tracks_topic_;
  std::string depth_topic_;
  std::string depth_info_topic_;
  std::string sony_info_topic_;
  std::string output_topic_;
  std::string debug_topic_;
  std::string sony_frame_;
  std::string gemini_depth_frame_;
  int depth_pixel_stride_;
  int min_target_samples_;
  double min_depth_m_;
  double max_depth_m_;
  double min_valid_depth_ratio_;
  double depth_percentile_;
  double depth_temporal_alpha_;
  double velocity_temporal_alpha_;
  double max_sync_skew_s_;
  double tracks_timeout_s_;
  double depth_timeout_s_;
  double filter_retention_s_;
  double max_position_jump_m_;
  bool use_distortion_;
  bool publish_debug_;

  std::unordered_map<int, TargetFilterState> target_filters_;
  std::string health_state_{"WAITING_INPUTS"};
  double last_sync_skew_ms_{-1.0};
  int last_fused_count_{0};
  std::size_t last_projected_point_count_{0};
};

}  // namespace perception_pkg
