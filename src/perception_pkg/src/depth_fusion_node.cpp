#include "perception_pkg/depth_fusion_node.hpp"
#include "perception_pkg/qos.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

using namespace perception_pkg::qos;

namespace perception_pkg
{
namespace
{

double seconds_between(const rclcpp::Time & lhs, const rclcpp::Time & rhs)
{
  return std::abs((lhs - rhs).seconds());
}

cv::Mat copy_distortion(const std::vector<double> & values)
{
  if (values.empty()) {
    return cv::Mat();
  }
  cv::Mat result(1, static_cast<int>(values.size()), CV_64F);
  std::memcpy(result.data, values.data(), values.size() * sizeof(double));
  return result;
}

float clamp01(float value)
{
  return std::max(0.0F, std::min(1.0F, value));
}

float percentile(std::vector<float> values, double fraction)
{
  if (values.empty()) {
    return 0.0F;
  }
  const auto bounded = std::max(0.0, std::min(1.0, fraction));
  const auto index = static_cast<std::size_t>(
    std::llround(bounded * static_cast<double>(values.size() - 1)));
  std::nth_element(values.begin(), values.begin() + index, values.end());
  return values[index];
}

}  // namespace

DepthFusionNode::DepthFusionNode(const rclcpp::NodeOptions & options)
: Node("depth_fusion_node", options), diagnostics_(this)
{
  tracks_topic_ = declare_parameter("topics.tracks", "/perception/tracks");
  depth_topic_ = declare_parameter("topics.depth_image", "/camera/depth/image_raw");
  depth_info_topic_ = declare_parameter(
    "topics.depth_camera_info", "/camera/depth/camera_info");
  sony_info_topic_ = declare_parameter("topics.sony_camera_info", "/sony/camera_info");
  output_topic_ = declare_parameter("topics.targets_3d", "/perception/targets_3d");
  debug_topic_ = declare_parameter(
    "topics.debug_image", "/perception/targets_3d_debug");

  sony_frame_ = declare_parameter("frames.sony_optical", "sony_camera_optical_frame");
  gemini_depth_frame_ = declare_parameter(
    "frames.gemini_depth_optical", "camera_depth_optical_frame");
  depth_pixel_stride_ = declare_parameter("projection.depth_pixel_stride", 2);
  min_target_samples_ = declare_parameter("projection.min_target_samples", 40);
  use_distortion_ = declare_parameter("projection.use_distortion", false);
  min_depth_m_ = declare_parameter("depth.min_meters", 0.30);
  max_depth_m_ = declare_parameter("depth.max_meters", 10.0);
  min_valid_depth_ratio_ = declare_parameter("depth.min_valid_ratio", 0.10);
  depth_percentile_ = declare_parameter("depth.percentile", 0.40);
  torso_x_min_ratio_ = declare_parameter("depth.torso_x_min_ratio", 0.30);
  torso_x_max_ratio_ = declare_parameter("depth.torso_x_max_ratio", 0.70);
  torso_y_min_ratio_ = declare_parameter("depth.torso_y_min_ratio", 0.22);
  torso_y_max_ratio_ = declare_parameter("depth.torso_y_max_ratio", 0.68);
  xy_temporal_alpha_ = declare_parameter("filter.xy_position_alpha", 0.45);
  z_temporal_alpha_ = declare_parameter("filter.z_position_alpha", 0.28);
  velocity_temporal_alpha_ = declare_parameter("filter.velocity_alpha", 0.25);
  max_position_jump_m_ = declare_parameter("filter.max_position_jump_meters", 0.75);
  max_xy_speed_mps_ = declare_parameter("filter.max_xy_speed_mps", 3.0);
  max_z_speed_mps_ = declare_parameter("filter.max_z_speed_mps", 4.0);
  prediction_hold_s_ = declare_parameter("filter.prediction_hold_seconds", 0.45);
  filter_retention_s_ = declare_parameter("filter.retention_seconds", 3.0);
  degraded_confidence_ = declare_parameter("quality.degraded_confidence", 0.40);
  valid_confidence_ = declare_parameter("quality.valid_confidence", 0.65);
  max_sync_skew_s_ = declare_parameter("timing.max_sync_skew_seconds", 0.080);
  max_pair_wait_s_ = declare_parameter("timing.max_pair_wait_seconds", 0.120);
  max_data_age_s_ = declare_parameter("timing.max_data_age_seconds", 0.250);
  tracks_timeout_s_ = declare_parameter("timing.tracks_timeout_seconds", 0.50);
  depth_timeout_s_ = declare_parameter("timing.depth_timeout_seconds", 0.50);
  depth_queue_capacity_ = declare_parameter("timing.depth_queue_capacity", 8);
  track_queue_capacity_ = declare_parameter("timing.track_queue_capacity", 8);
  publish_debug_ = declare_parameter("debug.publish_image", false);

  if (depth_pixel_stride_ < 1 || min_target_samples_ < 1 ||
    min_depth_m_ <= 0.0 || max_depth_m_ <= min_depth_m_ ||
    depth_percentile_ < 0.0 || depth_percentile_ > 1.0 ||
    xy_temporal_alpha_ <= 0.0 || xy_temporal_alpha_ > 1.0 ||
    z_temporal_alpha_ <= 0.0 || z_temporal_alpha_ > 1.0 ||
    velocity_temporal_alpha_ <= 0.0 || velocity_temporal_alpha_ > 1.0 ||
    torso_x_min_ratio_ < 0.0 || torso_x_max_ratio_ > 1.0 ||
    torso_y_min_ratio_ < 0.0 || torso_y_max_ratio_ > 1.0 ||
    torso_x_max_ratio_ <= torso_x_min_ratio_ ||
    torso_y_max_ratio_ <= torso_y_min_ratio_ ||
    max_position_jump_m_ <= 0.0 || max_xy_speed_mps_ <= 0.0 ||
    max_z_speed_mps_ <= 0.0 || prediction_hold_s_ < 0.0 ||
    degraded_confidence_ < 0.0 || valid_confidence_ > 1.0 ||
    valid_confidence_ < degraded_confidence_ ||
    max_sync_skew_s_ < 0.0 || max_pair_wait_s_ < 0.0 || max_data_age_s_ <= 0.0 ||
    depth_timeout_s_ <= 0.0 || tracks_timeout_s_ <= 0.0 ||
    depth_queue_capacity_ < 2 || track_queue_capacity_ < 2)
  {
    throw std::invalid_argument("Invalid depth_fusion_node parameter combination");
  }

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tracks_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
    tracks_topic_, perception(),
    std::bind(&DepthFusionNode::tracks_callback, this, std::placeholders::_1));
  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    depth_topic_, image(),
    std::bind(&DepthFusionNode::depth_callback, this, std::placeholders::_1));
  depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    depth_info_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DepthFusionNode::depth_info_callback, this, std::placeholders::_1));
  sony_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    sony_info_topic_, rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DepthFusionNode::sony_info_callback, this, std::placeholders::_1));

  targets_3d_pub_ = create_publisher<vision_servo_msgs::msg::TargetArray>(
    output_topic_, perception());
  if (publish_debug_) {
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(debug_topic_, image());
  }
  diagnostics_.setHardwareID("sony_gemini_rigid_pair");
  diagnostics_.add("depth_fusion", this, &DepthFusionNode::produce_diagnostics);

  RCLCPP_INFO(
    get_logger(),
    "Depth fusion ready: %s -> %s, queues depth=%d tracks=%d, skew<=%.0fms wait<=%.0fms",
    gemini_depth_frame_.c_str(), sony_frame_.c_str(),
    depth_queue_capacity_, track_queue_capacity_,
    max_sync_skew_s_ * 1000.0, max_pair_wait_s_ * 1000.0);
}

void DepthFusionNode::tracks_callback(
  const vision_servo_msgs::msg::TargetArray::SharedPtr msg)
{
  const rclcpp::Time track_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const auto now = get_clock()->now();
  if (seconds_between(now, track_stamp) > tracks_timeout_s_) {
    ++dropped_track_count_;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting stale tracks (age %.1f ms)",
      seconds_between(now, track_stamp) * 1000.0);
    return;
  }

  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    TrackFrame frame;
    frame.message = msg;
    frame.stamp = track_stamp;
    frame.enqueued_at = now;
    const auto insertion = std::upper_bound(
      track_queue_.begin(), track_queue_.end(), track_stamp,
      [](const rclcpp::Time & stamp, const TrackFrame & queued) {
        return stamp < queued.stamp;
      });
    track_queue_.insert(insertion, std::move(frame));
    while (static_cast<int>(track_queue_.size()) > track_queue_capacity_) {
      track_queue_.pop_front();
      ++dropped_track_count_;
    }
  }
  process_available_matches();
}

void DepthFusionNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv::Mat meters;
    if (msg->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
      msg->encoding == sensor_msgs::image_encodings::MONO16)
    {
      const auto cv_ptr = cv_bridge::toCvShare(msg, msg->encoding);
      cv_ptr->image.convertTo(meters, CV_32FC1, 0.001);
    } else if (msg->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      meters = cv_bridge::toCvShare(
        msg, sensor_msgs::image_encodings::TYPE_32FC1)->image.clone();
    } else {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Unsupported depth encoding '%s'; expected 16UC1/mono16 or 32FC1",
        msg->encoding.c_str());
      return;
    }

    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      DepthFrame frame;
      frame.meters = std::move(meters);
      frame.stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
      frame.frame_id =
        msg->header.frame_id.empty() ? gemini_depth_frame_ : msg->header.frame_id;
      const auto insertion = std::upper_bound(
        depth_queue_.begin(), depth_queue_.end(), frame.stamp,
        [](const rclcpp::Time & stamp, const DepthFrame & queued) {
          return stamp < queued.stamp;
        });
      depth_queue_.insert(insertion, std::move(frame));
      while (static_cast<int>(depth_queue_.size()) > depth_queue_capacity_) {
        depth_queue_.pop_front();
        ++dropped_depth_count_;
      }
    }
    process_available_matches();
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Depth conversion failed: %s", error.what());
  }
}

void DepthFusionNode::process_available_matches()
{
  std::lock_guard<std::mutex> processing_lock(processing_mutex_);
  const auto now = get_clock()->now();
  auto matches = collect_matches(now);
  if (matches.empty()) {
    diagnostics_.force_update();
    return;
  }
  for (auto & match : matches) {
    last_sync_skew_ms_ = match.skew_seconds * 1000.0;
    last_output_age_ms_ =
      seconds_between(now, rclcpp::Time(
        match.tracks->header.stamp, get_clock()->get_clock_type())) * 1000.0;
    ++matched_pair_count_;
    process_frame(
      *match.tracks, match.depth.meters, match.depth.stamp, match.depth.frame_id);
  }
}

std::vector<DepthFusionNode::MatchedFrame> DepthFusionNode::collect_matches(
  const rclcpp::Time & now)
{
  std::vector<MatchedFrame> matches;
  std::lock_guard<std::mutex> lock(data_mutex_);
  prune_queues(now);

  if (!sony_info_ready_ || !depth_info_ready_ || depth_queue_.empty()) {
    health_state_ = "WAITING_INPUTS";
    last_depth_queue_size_ = depth_queue_.size();
    last_track_queue_size_ = track_queue_.size();
    return matches;
  }

  while (!track_queue_.empty() && !depth_queue_.empty()) {
    const auto & track = track_queue_.front();
    auto closest = std::min_element(
      depth_queue_.begin(), depth_queue_.end(),
      [&track](const DepthFrame & lhs, const DepthFrame & rhs) {
        return seconds_between(lhs.stamp, track.stamp) <
               seconds_between(rhs.stamp, track.stamp);
      });
    const double skew = seconds_between(closest->stamp, track.stamp);
    const double waited = std::max(0.0, (now - track.enqueued_at).seconds());
    const bool observed_future_depth = depth_queue_.back().stamp >= track.stamp;

    if (skew <= max_sync_skew_s_ &&
      (observed_future_depth || waited >= max_pair_wait_s_))
    {
      MatchedFrame match;
      match.tracks = track.message;
      match.depth = *closest;
      match.skew_seconds = skew;
      matches.push_back(std::move(match));
      track_queue_.pop_front();
      continue;
    }

    if (!observed_future_depth && waited < max_pair_wait_s_) {
      health_state_ = "WAITING_SYNC";
      break;
    }

    last_sync_skew_ms_ = skew * 1000.0;
    health_state_ = "TIME_UNSYNCED";
    track_queue_.pop_front();
    ++dropped_track_count_;
  }

  last_depth_queue_size_ = depth_queue_.size();
  last_track_queue_size_ = track_queue_.size();
  return matches;
}

void DepthFusionNode::prune_queues(const rclcpp::Time & now)
{
  while (!track_queue_.empty() &&
    seconds_between(now, track_queue_.front().stamp) > max_data_age_s_)
  {
    track_queue_.pop_front();
    ++dropped_track_count_;
  }
  while (!depth_queue_.empty() &&
    seconds_between(now, depth_queue_.front().stamp) > max_data_age_s_)
  {
    depth_queue_.pop_front();
    ++dropped_depth_count_;
  }

  if (!track_queue_.empty()) {
    const auto oldest_usable =
      track_queue_.front().stamp - rclcpp::Duration::from_seconds(max_sync_skew_s_);
    while (depth_queue_.size() > 1 && depth_queue_[1].stamp < oldest_usable) {
      depth_queue_.pop_front();
      ++dropped_depth_count_;
    }
  }
}

void DepthFusionNode::depth_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  K_depth_ = cv::Mat(3, 3, CV_64F);
  std::memcpy(K_depth_.data, msg->k.data(), 9 * sizeof(double));
  D_depth_ = copy_distortion(msg->d);
  depth_info_ready_ = true;
}

void DepthFusionNode::sony_info_callback(
  const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  K_sony_ = cv::Mat(3, 3, CV_64F);
  std::memcpy(K_sony_.data, msg->k.data(), 9 * sizeof(double));
  D_sony_ = copy_distortion(msg->d);
  sony_width_ = msg->width;
  sony_height_ = msg->height;
  sony_info_ready_ = true;
}

bool DepthFusionNode::lookup_depth_to_sony(
  const rclcpp::Time & stamp,
  const std::string & depth_frame,
  Eigen::Isometry3d & transform)
{
  try {
    // tf2 returns T_target_source. This is directly T_sony_depth; no inverse is required.
    const auto tf = tf_buffer_->lookupTransform(
      sony_frame_,
      depth_frame.empty() ? gemini_depth_frame_ : depth_frame,
      stamp,
      rclcpp::Duration::from_seconds(0.05));
    transform = tf2::transformToEigen(tf);
    depth_to_sony_tf_available_ = true;
    return true;
  } catch (const tf2::TransformException & error) {
    depth_to_sony_tf_available_ = false;
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Missing depth-to-Sony TF at sensor timestamp: %s", error.what());
    return false;
  }
}

std::vector<DepthFusionNode::DepthSample> DepthFusionNode::reproject_depth_points(
  const cv::Mat & depth_meters,
  const Eigen::Isometry3d & depth_to_sony) const
{
  cv::Mat K_depth;
  cv::Mat K_sony;
  cv::Mat D_sony;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    K_depth = K_depth_.clone();
    K_sony = K_sony_.clone();
    D_sony = D_sony_.clone();
  }

  const double dfx = K_depth.at<double>(0, 0);
  const double dfy = K_depth.at<double>(1, 1);
  const double dcx = K_depth.at<double>(0, 2);
  const double dcy = K_depth.at<double>(1, 2);
  const double sfx = K_sony.at<double>(0, 0);
  const double sfy = K_sony.at<double>(1, 1);
  const double scx = K_sony.at<double>(0, 2);
  const double scy = K_sony.at<double>(1, 2);

  std::vector<DepthSample> result;
  result.reserve(
    static_cast<std::size_t>(depth_meters.total()) /
    static_cast<std::size_t>(depth_pixel_stride_ * depth_pixel_stride_));

  for (int v = 0; v < depth_meters.rows; v += depth_pixel_stride_) {
    const float * row = depth_meters.ptr<float>(v);
    for (int u = 0; u < depth_meters.cols; u += depth_pixel_stride_) {
      const float z = row[u];
      if (!std::isfinite(z) || z < min_depth_m_ || z > max_depth_m_) {
        continue;
      }
      const Eigen::Vector3d point_depth(
        (static_cast<double>(u) - dcx) * z / dfx,
        (static_cast<double>(v) - dcy) * z / dfy,
        z);
      const Eigen::Vector3d point_sony = depth_to_sony * point_depth;
      if (point_sony.z() <= 0.0) {
        continue;
      }

      double image_u = sfx * point_sony.x() / point_sony.z() + scx;
      double image_v = sfy * point_sony.y() / point_sony.z() + scy;
      if (use_distortion_ && !D_sony.empty()) {
        std::vector<cv::Point3d> object_points{
          cv::Point3d(point_sony.x(), point_sony.y(), point_sony.z())};
        std::vector<cv::Point2d> image_points;
        cv::projectPoints(
          object_points, cv::Vec3d::all(0.0), cv::Vec3d::all(0.0),
          K_sony, D_sony, image_points);
        image_u = image_points.front().x;
        image_v = image_points.front().y;
      }
      if (image_u < 0.0 || image_v < 0.0 ||
        image_u >= static_cast<double>(sony_width_) ||
        image_v >= static_cast<double>(sony_height_))
      {
        continue;
      }

      DepthSample sample;
      sample.pixel_sony = cv::Point2f(
        static_cast<float>(image_u), static_cast<float>(image_v));
      sample.point_sony = cv::Vec3f(
        static_cast<float>(point_sony.x()),
        static_cast<float>(point_sony.y()),
        static_cast<float>(point_sony.z()));
      result.push_back(sample);
    }
  }
  return result;
}

bool DepthFusionNode::estimate_target_position(
  const vision_servo_msgs::msg::Target & target,
  const std::vector<DepthSample> & points,
  PositionEstimate & estimate) const
{
  const float x_min = target.bbox[0];
  const float y_min = target.bbox[1];
  const float x_max = target.bbox[2];
  const float y_max = target.bbox[3];
  if (x_max <= x_min || y_max <= y_min) {
    return false;
  }
  const float width = x_max - x_min;
  const float height = y_max - y_min;
  const float roi_x_min = x_min + width * static_cast<float>(torso_x_min_ratio_);
  const float roi_x_max = x_min + width * static_cast<float>(torso_x_max_ratio_);
  const float roi_y_min = y_min + height * static_cast<float>(torso_y_min_ratio_);
  const float roi_y_max = y_min + height * static_cast<float>(torso_y_max_ratio_);
  const float roi_area =
    std::max(1.0F, (roi_x_max - roi_x_min) * (roi_y_max - roi_y_min));

  std::vector<const DepthSample *> candidates;
  candidates.reserve(256);
  std::vector<float> sony_depths;
  for (const auto & point : points) {
    if (point.pixel_sony.x >= roi_x_min && point.pixel_sony.x < roi_x_max &&
      point.pixel_sony.y >= roi_y_min && point.pixel_sony.y < roi_y_max)
    {
      candidates.push_back(&point);
      sony_depths.push_back(point.point_sony[2]);
    }
  }
  estimate.sample_count = static_cast<int>(candidates.size());
  if (estimate.sample_count < min_target_samples_) {
    return false;
  }

  const float seed_depth = percentile(sony_depths, depth_percentile_);
  std::vector<float> deviations;
  deviations.reserve(sony_depths.size());
  for (const float value : sony_depths) {
    deviations.push_back(std::abs(value - seed_depth));
  }
  const float mad = percentile(deviations, 0.50);
  const float band = std::max(0.08F, 2.5F * mad);

  cv::Vec3d sum(0.0, 0.0, 0.0);
  int foreground_count = 0;
  for (const auto * candidate : candidates) {
    const float z = candidate->point_sony[2];
    if (std::abs(z - seed_depth) > band) {
      continue;
    }
    sum[0] += candidate->point_sony[0];
    sum[1] += candidate->point_sony[1];
    sum[2] += candidate->point_sony[2];
    ++foreground_count;
  }
  if (foreground_count < min_target_samples_) {
    return false;
  }

  estimate.position = cv::Vec3f(
    static_cast<float>(sum[0] / foreground_count),
    static_cast<float>(sum[1] / foreground_count),
    static_cast<float>(sum[2] / foreground_count));

  const float sampling_area =
    roi_area / static_cast<float>(depth_pixel_stride_ * depth_pixel_stride_);
  const float coverage = static_cast<float>(foreground_count) / std::max(1.0F, sampling_area);
  const float consistency = std::exp(-mad / 0.15F);
  estimate.valid_ratio = clamp01(coverage);
  estimate.depth_spread = mad;
  estimate.confidence = clamp01(
    0.55F * std::min(1.0F, coverage / static_cast<float>(min_valid_depth_ratio_)) +
    0.45F * consistency);
  return true;
}

bool DepthFusionNode::update_filter(
  int target_id,
  const rclcpp::Time & stamp,
  const cv::Vec3f & measured_position,
  cv::Vec3f & filtered_position,
  cv::Vec3f & filtered_velocity)
{
  auto & state = target_filters_[target_id];
  if (!state.initialized) {
    state.position = measured_position;
    state.velocity = cv::Vec3f::all(0.0F);
    state.stamp = stamp;
    state.last_measurement_stamp = stamp;
    state.consecutive_rejections = 0;
    state.initialized = true;
  } else {
    const double dt = (stamp - state.stamp).seconds();
    if (dt <= 0.0) {
      return false;
    }
    if (dt > filter_retention_s_) {
      state.position = measured_position;
      state.velocity = cv::Vec3f::all(0.0F);
      state.consecutive_rejections = 0;
    } else {
      const cv::Vec3f predicted =
        state.position + state.velocity * static_cast<float>(dt);
      const cv::Vec3f residual = measured_position - predicted;
      const float jump = cv::norm(residual);
      const float xy_speed =
        std::hypot(residual[0], residual[1]) / static_cast<float>(dt);
      const float z_speed = std::abs(residual[2]) / static_cast<float>(dt);
      if (jump > max_position_jump_m_ ||
        xy_speed > max_xy_speed_mps_ || z_speed > max_z_speed_mps_)
      {
        ++state.consecutive_rejections;
        ++rejected_measurement_count_;
        return false;
      }
      const cv::Vec3f previous = state.position;
      state.position[0] =
        static_cast<float>(xy_temporal_alpha_) * measured_position[0] +
        static_cast<float>(1.0 - xy_temporal_alpha_) * predicted[0];
      state.position[1] =
        static_cast<float>(xy_temporal_alpha_) * measured_position[1] +
        static_cast<float>(1.0 - xy_temporal_alpha_) * predicted[1];
      state.position[2] =
        static_cast<float>(z_temporal_alpha_) * measured_position[2] +
        static_cast<float>(1.0 - z_temporal_alpha_) * predicted[2];
      const cv::Vec3f measured_velocity =
        (state.position - previous) / static_cast<float>(dt);
      state.velocity =
        static_cast<float>(velocity_temporal_alpha_) * measured_velocity +
        static_cast<float>(1.0 - velocity_temporal_alpha_) * state.velocity;
      state.consecutive_rejections = 0;
    }
    state.stamp = stamp;
    state.last_measurement_stamp = stamp;
  }
  filtered_position = state.position;
  filtered_velocity = state.velocity;
  return true;
}

bool DepthFusionNode::predict_filter(
  int target_id,
  const rclcpp::Time & stamp,
  cv::Vec3f & predicted_position,
  cv::Vec3f & predicted_velocity,
  float & prediction_age)
{
  const auto found = target_filters_.find(target_id);
  if (found == target_filters_.end() || !found->second.initialized) {
    return false;
  }
  auto & state = found->second;
  const double age = (stamp - state.last_measurement_stamp).seconds();
  if (age < 0.0 || age > prediction_hold_s_) {
    return false;
  }
  const double dt = std::max(0.0, (stamp - state.stamp).seconds());
  predicted_position =
    state.position + state.velocity * static_cast<float>(std::min(dt, prediction_hold_s_));
  predicted_velocity = state.velocity;
  prediction_age = static_cast<float>(age);
  state.position = predicted_position;
  state.stamp = stamp;
  return std::isfinite(predicted_position[0]) &&
         std::isfinite(predicted_position[1]) &&
         std::isfinite(predicted_position[2]) &&
         predicted_position[2] >= min_depth_m_ &&
         predicted_position[2] <= max_depth_m_;
}

void DepthFusionNode::prune_filters(const rclcpp::Time & stamp)
{
  for (auto it = target_filters_.begin(); it != target_filters_.end();) {
    if ((stamp - it->second.stamp).seconds() > filter_retention_s_) {
      it = target_filters_.erase(it);
    } else {
      ++it;
    }
  }
}

void DepthFusionNode::process_frame(
  const vision_servo_msgs::msg::TargetArray & tracks,
  const cv::Mat & depth_meters,
  const rclcpp::Time & depth_stamp,
  const std::string & depth_frame)
{
  Eigen::Isometry3d depth_to_sony = Eigen::Isometry3d::Identity();
  if (!lookup_depth_to_sony(depth_stamp, depth_frame, depth_to_sony)) {
    health_state_ = "WAITING_TF";
    diagnostics_.force_update();
    return;
  }
  const auto points = reproject_depth_points(depth_meters, depth_to_sony);
  last_projected_point_count_ = points.size();

  vision_servo_msgs::msg::TargetArray output;
  output.header = tracks.header;
  output.header.frame_id = sony_frame_;
  output.tracking_id = tracks.tracking_id;
  const rclcpp::Time track_stamp(tracks.header.stamp, get_clock()->get_clock_type());
  int fused_count = 0;
  int predicted_count = 0;

  for (const auto & target : tracks.targets) {
    auto fused = target;
    fused.header = output.header;
    fused.position = {{0.0F, 0.0F, 0.0F}};
    fused.velocity = {{0.0F, 0.0F, 0.0F}};
    fused.depth_confidence = 0.0F;
    fused.fusion_state = vision_servo_msgs::msg::Target::FUSION_STATE_INVALID;
    fused.raw_position = {{0.0F, 0.0F, 0.0F}};
    fused.depth_valid_ratio = 0.0F;
    fused.depth_spread = 0.0F;
    fused.fusion_age = 0.0F;

    if (target.tracking_state == vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED &&
      target.visible)
    {
      PositionEstimate estimate;
      if (estimate_target_position(target, points, estimate))
      {
        cv::Vec3f filtered_position;
        cv::Vec3f filtered_velocity;
        if (update_filter(
            target.id, track_stamp, estimate.position,
            filtered_position, filtered_velocity))
        {
          fused.position = {{
            filtered_position[0], filtered_position[1], filtered_position[2]}};
          fused.velocity = {{
            filtered_velocity[0], filtered_velocity[1], filtered_velocity[2]}};
          fused.raw_position = {{
            estimate.position[0], estimate.position[1], estimate.position[2]}};
          fused.depth_valid_ratio = estimate.valid_ratio;
          fused.depth_spread = estimate.depth_spread;
          const float sync_confidence = clamp01(
            1.0F - static_cast<float>(
              seconds_between(track_stamp, depth_stamp) /
              std::max(max_sync_skew_s_, 1e-6)));
          const float detection_quality = clamp01(target.confidence);
          fused.depth_confidence = estimate.confidence *
            (0.75F + 0.25F * sync_confidence) *
            (0.80F + 0.20F * detection_quality);
          if (fused.depth_confidence >= valid_confidence_) {
            fused.fusion_state =
              vision_servo_msgs::msg::Target::FUSION_STATE_VALID;
          } else if (fused.depth_confidence >= degraded_confidence_) {
            fused.fusion_state =
              vision_servo_msgs::msg::Target::FUSION_STATE_DEGRADED;
          } else {
            fused.depth_confidence = 0.0F;
          }
          if (fused.fusion_state !=
            vision_servo_msgs::msg::Target::FUSION_STATE_INVALID)
          {
            ++fused_count;
          }
        }
      }
    }

    // Only the explicitly locked target may retain a short predicted 3D pose.
    // Prediction never masquerades as a real measurement and must not drive
    // chassis translation.
    if (fused.fusion_state ==
      vision_servo_msgs::msg::Target::FUSION_STATE_INVALID &&
      target.id == tracks.tracking_id)
    {
      cv::Vec3f predicted_position;
      cv::Vec3f predicted_velocity;
      float prediction_age = 0.0F;
      if (predict_filter(
          target.id, track_stamp, predicted_position,
          predicted_velocity, prediction_age))
      {
        fused.position = {{
          predicted_position[0], predicted_position[1], predicted_position[2]}};
        fused.velocity = {{
          predicted_velocity[0], predicted_velocity[1], predicted_velocity[2]}};
        fused.fusion_age = prediction_age;
        fused.depth_confidence = clamp01(
          static_cast<float>(degraded_confidence_) *
          (1.0F - prediction_age /
          static_cast<float>(std::max(prediction_hold_s_, 1e-6))));
        fused.fusion_state =
          vision_servo_msgs::msg::Target::FUSION_STATE_PREDICTED;
        ++predicted_count;
      }
    }
    output.targets.push_back(std::move(fused));
  }

  prune_filters(track_stamp);
  targets_3d_pub_->publish(output);
  last_fused_count_ = fused_count;
  predicted_target_count_ = static_cast<std::size_t>(predicted_count);
  health_state_ = points.empty() ? "NO_VALID_DEPTH" :
    (fused_count > 0 ? "READY" :
    (predicted_count > 0 ? "PREDICTING" : "NO_VALID_TARGET"));
  diagnostics_.force_update();
  if (publish_debug_ && debug_pub_) {
    publish_debug_image(output, points, output.header);
  }
}

void DepthFusionNode::produce_diagnostics(
  diagnostic_updater::DiagnosticStatusWrapper & status)
{
  if (health_state_ == "READY") {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, health_state_);
  } else if (
    health_state_ == "WAITING_INPUTS" || health_state_ == "WAITING_SYNC" ||
    health_state_ == "PREDICTING" || health_state_ == "NO_VALID_TARGET")
  {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, health_state_);
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, health_state_);
  }
  status.add("sync_skew_ms", last_sync_skew_ms_);
  status.add("fused_target_count", last_fused_count_);
  status.add("projected_depth_point_count", last_projected_point_count_);
  status.add("depth_queue_size", last_depth_queue_size_);
  status.add("track_queue_size", last_track_queue_size_);
  status.add("matched_pair_count", matched_pair_count_);
  status.add("dropped_track_count", dropped_track_count_);
  status.add("dropped_depth_count", dropped_depth_count_);
  status.add("rejected_measurement_count", rejected_measurement_count_);
  status.add("predicted_target_count", predicted_target_count_);
  status.add("output_age_ms", last_output_age_ms_);
  status.add("direct_depth_tf_loaded", depth_to_sony_tf_available_);
  status.add("sony_frame", sony_frame_);
  status.add("gemini_depth_frame", gemini_depth_frame_);
}

void DepthFusionNode::publish_debug_image(
  const vision_servo_msgs::msg::TargetArray & output,
  const std::vector<DepthSample> & points,
  const std_msgs::msg::Header & header)
{
  cv::Mat canvas(
    static_cast<int>(sony_height_), static_cast<int>(sony_width_), CV_8UC3,
    cv::Scalar(20, 20, 20));
  for (const auto & point : points) {
    const float normalized = clamp01(
      static_cast<float>((point.point_sony[2] - min_depth_m_) /
      (max_depth_m_ - min_depth_m_)));
    const cv::Scalar color(255.0 * (1.0 - normalized), 80.0, 255.0 * normalized);
    cv::circle(
      canvas,
      cv::Point(
        static_cast<int>(point.pixel_sony.x),
        static_cast<int>(point.pixel_sony.y)),
      1, color, -1);
  }
  for (const auto & target : output.targets) {
    const cv::Scalar color =
      target.depth_confidence > 0.0F ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255);
    cv::rectangle(
      canvas,
      cv::Rect(
        cv::Point(static_cast<int>(target.bbox[0]), static_cast<int>(target.bbox[1])),
        cv::Point(static_cast<int>(target.bbox[2]), static_cast<int>(target.bbox[3]))),
      color, 2);
    cv::putText(
      canvas,
      "ID " + std::to_string(target.id) + " z=" +
      cv::format("%.2f", target.position[2]) + " c=" +
      cv::format("%.2f", target.depth_confidence),
      cv::Point(
        static_cast<int>(target.bbox[0]),
        std::max(18, static_cast<int>(target.bbox[1]) - 5)),
      cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  }
  debug_pub_->publish(
    *cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, canvas).toImageMsg());
}

}  // namespace perception_pkg

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<perception_pkg::DepthFusionNode>());
  rclcpp::shutdown();
  return 0;
}
