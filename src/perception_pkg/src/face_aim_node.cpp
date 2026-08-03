#include "perception_pkg/face_aim_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <vision_servo_msgs/msg/aim_target2_d.hpp>

namespace perception_pkg {

FaceAimNode::FaceAimNode(const rclcpp::NodeOptions& options)
  : Node("face_aim_node", options)
{
  declare_parameter("input_timeout_seconds", 2.0);
  declare_parameter("publish_debug", true);
  declare_parameter("aim_offset_ratio", 0.20);
  declare_parameter("alpha_beta_alpha", 0.65);
  declare_parameter("alpha_beta_beta", 0.08);
  declare_parameter("covariance_alpha", 0.15);
  declare_parameter("initial_covariance_px2", 16.0);
  declare_parameter("lost_covariance_growth_px2_per_sec", 400.0);
  declare_parameter("min_filter_dt_seconds", 0.005);
  declare_parameter("max_filter_dt_seconds", 0.20);

  input_timeout_seconds_ = get_parameter("input_timeout_seconds").as_double();
  publish_debug_ = get_parameter("publish_debug").as_bool();
  aim_offset_ratio_ = get_parameter("aim_offset_ratio").as_double();
  alpha_beta_alpha_ = get_parameter("alpha_beta_alpha").as_double();
  alpha_beta_beta_ = get_parameter("alpha_beta_beta").as_double();
  covariance_alpha_ = get_parameter("covariance_alpha").as_double();
  initial_covariance_px2_ = get_parameter("initial_covariance_px2").as_double();
  lost_covariance_growth_px2_per_sec_ =
      get_parameter("lost_covariance_growth_px2_per_sec").as_double();
  min_filter_dt_seconds_ = get_parameter("min_filter_dt_seconds").as_double();
  max_filter_dt_seconds_ = get_parameter("max_filter_dt_seconds").as_double();

  alpha_beta_alpha_ = std::clamp(alpha_beta_alpha_, 0.0, 1.0);
  alpha_beta_beta_ = std::clamp(alpha_beta_beta_, 0.0, 1.0);
  covariance_alpha_ = std::clamp(covariance_alpha_, 0.0, 1.0);
  initial_covariance_px2_ = std::max(1.0, initial_covariance_px2_);
  min_filter_dt_seconds_ = std::max(1e-4, min_filter_dt_seconds_);
  max_filter_dt_seconds_ = std::max(
      min_filter_dt_seconds_, max_filter_dt_seconds_);

  image_sub_ = image_transport::create_subscription(
      this, "/sony/image_raw",
      std::bind(&FaceAimNode::image_callback, this, std::placeholders::_1),
      "raw", qos::image().get_rmw_qos_profile());

  tracks_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
      "/perception/tracks", qos::perception(),
      std::bind(&FaceAimNode::tracks_callback, this, std::placeholders::_1));

  aim_pub_ = create_publisher<vision_servo_msgs::msg::AimTarget2D>(
      "/perception/aim_target_2d", qos::perception());

  debug_pub_ = image_transport::create_publisher(this, "face_aim_debug");

  timeout_timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&FaceAimNode::check_timeout, this));

  last_tracks_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(get_logger(), "face_aim_node started");
}

void FaceAimNode::image_callback(
    const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
  RCLCPP_DEBUG(get_logger(), "image_callback: stamp=%u.%u",
      msg->header.stamp.sec, msg->header.stamp.nanosec);
  std::lock_guard<std::mutex> lock(mutex_);
  pending_image_ = msg;
  // Aim coordinates come from the tracker. The latest image is retained only
  // for dimensions and optional debug rendering.
  try_match_and_process();
}

void FaceAimNode::tracks_callback(
    const vision_servo_msgs::msg::TargetArray::ConstSharedPtr& msg)
{
  RCLCPP_DEBUG(get_logger(), "tracks_callback: stamp=%u.%u tracking_id=%d",
      msg->header.stamp.sec, msg->header.stamp.nanosec, msg->tracking_id);
  std::lock_guard<std::mutex> lock(mutex_);
  pending_tracks_ = msg;
  last_tracks_time_ = std::chrono::steady_clock::now();
  try_match_and_process();
}

void FaceAimNode::try_match_and_process()
{
  if (!pending_image_) {
    RCLCPP_DEBUG(get_logger(), "try_match: waiting for image");
    return;
  }
  if (!pending_tracks_) {
    RCLCPP_DEBUG(get_logger(), "try_match: waiting for tracks");
    return;
  }

  const float image_width = static_cast<float>(pending_image_->width);
  const float image_height = static_cast<float>(pending_image_->height);
  const auto& tracks = *pending_tracks_;

  vision_servo_msgs::msg::AimTarget2D aim;
  // Detection/tracking necessarily arrives after its source image. Requiring
  // the tracker stamp to equal the newest camera frame caused a permanent
  // mismatch at 30 Hz. The bbox already contains everything needed for aim;
  // preserve its source stamp so downstream freshness checks remain correct.
  aim.header = pending_tracks_->header;
  aim.tracking_id = -1;
  aim.valid = false;
  aim.confidence = 0.0f;
  aim.pixel_x = 0.0f;
  aim.pixel_y = 0.0f;
  aim.pixel_velocity_x = 0.0f;
  aim.pixel_velocity_y = 0.0f;
  aim.covariance_x = static_cast<float>(initial_covariance_px2_);
  aim.covariance_y = static_cast<float>(initial_covariance_px2_);
  aim.estimate_stamp = this->now();
  aim.source = vision_servo_msgs::msg::AimTarget2D::SHOULDERS;
  aim.predicted = false;

  if (tracks.tracking_id >= 0) {
    const auto it = std::find_if(
        tracks.targets.begin(), tracks.targets.end(),
        [id = tracks.tracking_id](const auto& t) {
          return t.id == id && t.class_name == "person";
        });

    if (it != tracks.targets.end()) {

      aim.tracking_id = it->id;

      const float raw_x = it->center[0];
      const float raw_y = it->bbox[1] + aim_offset_ratio_ * it->height;

      const rclcpp::Time measurement_stamp(
          tracks.header.stamp, RCL_ROS_TIME);
      const bool same_track = filter_initialized_ && it->id == last_tracking_id_;
      // LOST 轨迹只能延续已有真实观测，不能从一个纯预测框新建状态。
      if (!it->visible && !same_track) {
        filter_initialized_ = false;
        last_tracking_id_ = -1;
        aim_pub_->publish(aim);
        pending_tracks_.reset();
        return;
      }
      const double dt = same_track
          ? (measurement_stamp - last_filter_stamp_).seconds()
          : 0.0;
      const bool valid_dt = std::isfinite(dt) &&
          dt >= min_filter_dt_seconds_ && dt <= max_filter_dt_seconds_;

      if (!same_track || !valid_dt) {
        filtered_x_ = raw_x;
        filtered_y_ = raw_y;
        filtered_vx_ = 0.0f;
        filtered_vy_ = 0.0f;
        covariance_x_ = static_cast<float>(initial_covariance_px2_);
        covariance_y_ = static_cast<float>(initial_covariance_px2_);
        filter_initialized_ = true;
      } else {
        const float dt_f = static_cast<float>(dt);
        const float predicted_x = filtered_x_ + filtered_vx_ * dt_f;
        const float predicted_y = filtered_y_ + filtered_vy_ * dt_f;

        if (it->visible) {
          const float residual_x = raw_x - predicted_x;
          const float residual_y = raw_y - predicted_y;
          filtered_x_ = predicted_x +
              static_cast<float>(alpha_beta_alpha_) * residual_x;
          filtered_y_ = predicted_y +
              static_cast<float>(alpha_beta_alpha_) * residual_y;
          filtered_vx_ += static_cast<float>(alpha_beta_beta_) * residual_x / dt_f;
          filtered_vy_ += static_cast<float>(alpha_beta_beta_) * residual_y / dt_f;
          covariance_x_ = std::max(
              1.0f,
              static_cast<float>(1.0 - covariance_alpha_) * covariance_x_ +
              static_cast<float>(covariance_alpha_) * residual_x * residual_x);
          covariance_y_ = std::max(
              1.0f,
              static_cast<float>(1.0 - covariance_alpha_) * covariance_y_ +
              static_cast<float>(covariance_alpha_) * residual_y * residual_y);
        } else {
          // LOST 轨迹没有新测量：只传播状态，不把 tracker 的预测框再次
          // 当作测量更新，避免预测被重复计算而产生过冲。
          filtered_x_ = predicted_x;
          filtered_y_ = predicted_y;
          const float covariance_growth = static_cast<float>(
              lost_covariance_growth_px2_per_sec_ * dt);
          covariance_x_ += covariance_growth;
          covariance_y_ += covariance_growth;
        }
      }
      if (it->visible) {
        last_visible_source_stamp_ = tracks.header.stamp;
      }
      last_tracking_id_ = it->id;
      last_filter_stamp_ = measurement_stamp;

      aim.pixel_x = std::clamp(filtered_x_, 0.0f, image_width);
      aim.pixel_y = std::clamp(filtered_y_, 0.0f, image_height);
      aim.pixel_velocity_x = filtered_vx_;
      aim.pixel_velocity_y = filtered_vy_;
      aim.covariance_x = covariance_x_;
      aim.covariance_y = covariance_y_;
      // estimate_stamp 是当前滤波状态对应的时间坐标；LOST 时该状态已
      // 传播到本轮 tracker 图像时刻，控制器只需补偿此后新增的延迟。
      aim.estimate_stamp = tracks.header.stamp;
      aim.confidence = it->confidence;
      aim.predicted = !it->visible;
      aim.source = it->visible
          ? vision_servo_msgs::msg::AimTarget2D::SHOULDERS
          : vision_servo_msgs::msg::AimTarget2D::PREDICTED;
      if (!it->visible) {
        // header.stamp 表示最后一次真实 Sony 测量，而不是 tracker 本轮
        // 传播时刻。下游据此在 0.18/0.30 s 正确降权和撤权。
        aim.header.stamp = last_visible_source_stamp_;
      }
      aim.valid = true;

    } else {
      filter_initialized_ = false;
      last_tracking_id_ = -1;
    }
  }

  aim_pub_->publish(aim);
  RCLCPP_DEBUG(get_logger(), "published valid=%s id=%d source=%d (%.1f, %.1f)",
      aim.valid ? "true" : "false", aim.tracking_id, aim.source,
      aim.pixel_x, aim.pixel_y);

  if (publish_debug_) {
    publish_debug_image(pending_image_, aim.header, aim);
  }

  // Keep the latest image for the next tracks message. Only tracks are
  // consumed; otherwise a 30 Hz image can overwrite its matching delayed
  // tracks message before processing.
  pending_tracks_.reset();
}

void FaceAimNode::check_timeout()
{
  const double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(
      std::chrono::steady_clock::now() - last_tracks_time_).count();

  const bool in_timeout = elapsed > input_timeout_seconds_;

  if (in_timeout && !was_in_timeout_) {
    was_in_timeout_ = true;
    filter_initialized_ = false;
    last_tracking_id_ = -1;
    filtered_vx_ = 0.0f;
    filtered_vy_ = 0.0f;
    covariance_x_ = static_cast<float>(initial_covariance_px2_);
    covariance_y_ = static_cast<float>(initial_covariance_px2_);
    last_visible_source_stamp_ = builtin_interfaces::msg::Time();
    RCLCPP_INFO(get_logger(), "check_timeout: entering timeout");
    vision_servo_msgs::msg::AimTarget2D aim;
    aim.header.stamp = this->now();
    aim.header.frame_id = "";
    aim.tracking_id = -1;
    aim.valid = false;
    aim.confidence = 0.0f;
    aim.pixel_x = 0.0f;
    aim.pixel_y = 0.0f;
    aim.pixel_velocity_x = 0.0f;
    aim.pixel_velocity_y = 0.0f;
    aim.covariance_x = static_cast<float>(initial_covariance_px2_);
    aim.covariance_y = static_cast<float>(initial_covariance_px2_);
    aim.estimate_stamp = this->now();
    aim.source = vision_servo_msgs::msg::AimTarget2D::PREDICTED;
    aim.predicted = true;
    aim_pub_->publish(aim);
  } else if (!in_timeout && was_in_timeout_) {
    was_in_timeout_ = false;
    RCLCPP_INFO(get_logger(), "check_timeout: leaving timeout");
  }
}

void FaceAimNode::publish_debug_image(
    const sensor_msgs::msg::Image::ConstSharedPtr& image_msg,
    const std_msgs::msg::Header& header,
    const vision_servo_msgs::msg::AimTarget2D& aim)
{
  if (debug_pub_.getNumSubscribers() == 0) {
    return;
  }

  cv::Mat debug;
  try {
    const auto cv_ptr = cv_bridge::toCvShare(image_msg, "bgr8");
    debug = cv_ptr->image.clone();
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Debug image conversion failed (%s), using blank canvas", e.what());
    if (image_msg->height > 0 && image_msg->width > 0) {
      debug = cv::Mat::zeros(image_msg->height, image_msg->width, CV_8UC3);
    } else {
      debug = cv::Mat::zeros(480, 640, CV_8UC3);
    }
  }

  if (aim.valid) {
    const cv::Point aim_pt(
        static_cast<int>(aim.pixel_x),
        static_cast<int>(aim.pixel_y));
    const cv::Scalar color = aim.predicted
        ? cv::Scalar(0, 165, 255)
        : cv::Scalar(0, 255, 0);
    cv::circle(debug, aim_pt, 5, color, -1);
    cv::putText(debug,
        "ID:" + std::to_string(aim.tracking_id),
        cv::Point(aim_pt.x + 10, aim_pt.y - 10),
        cv::FONT_HERSHEY_SIMPLEX, 0.6, color, 2);
  }

  const auto bridge = cv_bridge::CvImage(header, "bgr8", debug).toImageMsg();
  debug_pub_.publish(bridge);
}

}  // namespace perception_pkg

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::FaceAimNode>(
      rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
