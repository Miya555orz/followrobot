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
  depth_temporal_alpha_ = declare_parameter("filter.position_alpha", 0.35);
  velocity_temporal_alpha_ = declare_parameter("filter.velocity_alpha", 0.25);
  max_position_jump_m_ = declare_parameter("filter.max_position_jump_meters", 1.25);
  filter_retention_s_ = declare_parameter("filter.retention_seconds", 3.0);
  max_sync_skew_s_ = declare_parameter("timing.max_sync_skew_seconds", 0.050);
  tracks_timeout_s_ = declare_parameter("timing.tracks_timeout_seconds", 0.50);
  depth_timeout_s_ = declare_parameter("timing.depth_timeout_seconds", 0.50);
  publish_debug_ = declare_parameter("debug.publish_image", false);

  if (depth_pixel_stride_ < 1 || min_target_samples_ < 1 ||
    min_depth_m_ <= 0.0 || max_depth_m_ <= min_depth_m_ ||
    depth_percentile_ < 0.0 || depth_percentile_ > 1.0 ||
    depth_temporal_alpha_ <= 0.0 || depth_temporal_alpha_ > 1.0 ||
    velocity_temporal_alpha_ <= 0.0 || velocity_temporal_alpha_ > 1.0 ||
    max_sync_skew_s_ < 0.0 || depth_timeout_s_ <= 0.0 || tracks_timeout_s_ <= 0.0)
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
    "Depth fusion ready: depth points in %s are transformed into %s at track timestamps",
    gemini_depth_frame_.c_str(), sony_frame_.c_str());
}

void DepthFusionNode::tracks_callback(
  const vision_servo_msgs::msg::TargetArray::SharedPtr msg)
{
  const rclcpp::Time track_stamp(msg->header.stamp, get_clock()->get_clock_type());
  const auto now = get_clock()->now();
  if (seconds_between(now, track_stamp) > tracks_timeout_s_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting stale tracks (age %.1f ms)",
      seconds_between(now, track_stamp) * 1000.0);
    return;
  }

  cv::Mat depth;
  rclcpp::Time depth_stamp(0, 0, get_clock()->get_clock_type());
  std::string depth_frame;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (depth_image_meters_.empty() || !sony_info_ready_ || !depth_info_ready_) {
      health_state_ = "WAITING_INPUTS";
      diagnostics_.force_update();
      return;
    }
    depth = depth_image_meters_.clone();
    depth_stamp = depth_stamp_;
    depth_frame = depth_frame_from_message_;
  }

  const double skew = seconds_between(track_stamp, depth_stamp);
  const double depth_age = seconds_between(now, depth_stamp);
  if (skew > max_sync_skew_s_ || depth_age > depth_timeout_s_) {
    health_state_ = "TIME_UNSYNCED";
    last_sync_skew_ms_ = skew * 1000.0;
    diagnostics_.force_update();
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Rejecting unsynchronised depth (track-depth %.1f ms, depth age %.1f ms)",
      skew * 1000.0, depth_age * 1000.0);
    return;
  }

  last_sync_skew_ms_ = skew * 1000.0;
  process_frame(*msg, depth, depth_stamp, depth_frame);
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

    std::lock_guard<std::mutex> lock(data_mutex_);
    depth_image_meters_ = std::move(meters);
    depth_stamp_ = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    depth_frame_from_message_ =
      msg->header.frame_id.empty() ? gemini_depth_frame_ : msg->header.frame_id;
  } catch (const cv_bridge::Exception & error) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Depth conversion failed: %s", error.what());
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
  Eigen::Isometry3d & transform) const
{
  try {
    // tf2 returns T_target_source. This is directly T_sony_depth; no inverse is required.
    const auto tf = tf_buffer_->lookupTransform(
      sony_frame_,
      depth_frame.empty() ? gemini_depth_frame_ : depth_frame,
      stamp,
      rclcpp::Duration::from_seconds(0.05));
    transform = tf2::transformToEigen(tf);
    return true;
  } catch (const tf2::TransformException & error) {
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
  cv::Vec3f & position,
  float & confidence,
  int & sample_count) const
{
  const float x_min = target.bbox[0];
  const float y_min = target.bbox[1];
  const float x_max = target.bbox[2];
  const float y_max = target.bbox[3];
  const float area = std::max(1.0F, (x_max - x_min) * (y_max - y_min));
  if (x_max <= x_min || y_max <= y_min) {
    return false;
  }

  std::vector<const DepthSample *> candidates;
  candidates.reserve(256);
  std::vector<float> sony_depths;
  for (const auto & point : points) {
    if (point.pixel_sony.x >= x_min && point.pixel_sony.x < x_max &&
      point.pixel_sony.y >= y_min && point.pixel_sony.y < y_max)
    {
      candidates.push_back(&point);
      sony_depths.push_back(point.point_sony[2]);
    }
  }
  sample_count = static_cast<int>(candidates.size());
  if (sample_count < min_target_samples_) {
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

  position = cv::Vec3f(
    static_cast<float>(sum[0] / foreground_count),
    static_cast<float>(sum[1] / foreground_count),
    static_cast<float>(sum[2] / foreground_count));

  const float sampling_area =
    area / static_cast<float>(depth_pixel_stride_ * depth_pixel_stride_);
  const float coverage = static_cast<float>(foreground_count) / std::max(1.0F, sampling_area);
  const float consistency = std::exp(-mad / 0.15F);
  confidence = clamp01(
    0.55F * std::min(1.0F, coverage / static_cast<float>(min_valid_depth_ratio_)) +
    0.45F * consistency);
  return true;
}

void DepthFusionNode::update_filter(
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
    state.initialized = true;
  } else {
    const double dt = (stamp - state.stamp).seconds();
    const float jump = cv::norm(measured_position - state.position);
    if (dt <= 0.0 || dt > filter_retention_s_ || jump > max_position_jump_m_) {
      state.position = measured_position;
      state.velocity = cv::Vec3f::all(0.0F);
    } else {
      const cv::Vec3f previous = state.position;
      state.position =
        static_cast<float>(depth_temporal_alpha_) * measured_position +
        static_cast<float>(1.0 - depth_temporal_alpha_) * state.position;
      const cv::Vec3f measured_velocity =
        (state.position - previous) / static_cast<float>(dt);
      state.velocity =
        static_cast<float>(velocity_temporal_alpha_) * measured_velocity +
        static_cast<float>(1.0 - velocity_temporal_alpha_) * state.velocity;
    }
    state.stamp = stamp;
  }
  filtered_position = state.position;
  filtered_velocity = state.velocity;
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

  for (const auto & target : tracks.targets) {
    auto fused = target;
    fused.header = output.header;
    fused.position = {{0.0F, 0.0F, 0.0F}};
    fused.velocity = {{0.0F, 0.0F, 0.0F}};
    fused.depth_confidence = 0.0F;

    if (target.tracking_state == vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED &&
      target.visible)
    {
      cv::Vec3f measured_position;
      float confidence = 0.0F;
      int sample_count = 0;
      if (estimate_target_position(
          target, points, measured_position, confidence, sample_count))
      {
        cv::Vec3f filtered_position;
        cv::Vec3f filtered_velocity;
        update_filter(
          target.id, track_stamp, measured_position, filtered_position, filtered_velocity);
        fused.position = {{
          filtered_position[0], filtered_position[1], filtered_position[2]}};
        fused.velocity = {{
          filtered_velocity[0], filtered_velocity[1], filtered_velocity[2]}};
        const float sync_confidence = clamp01(
          1.0F - static_cast<float>(
            seconds_between(track_stamp, depth_stamp) / std::max(max_sync_skew_s_, 1e-6)));
        fused.depth_confidence = confidence * (0.75F + 0.25F * sync_confidence);
        ++fused_count;
      }
    }
    output.targets.push_back(std::move(fused));
  }

  prune_filters(track_stamp);
  targets_3d_pub_->publish(output);
  last_fused_count_ = fused_count;
  health_state_ = points.empty() ? "NO_VALID_DEPTH" : "READY";
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
  } else if (health_state_ == "WAITING_INPUTS") {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, health_state_);
  } else {
    status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, health_state_);
  }
  status.add("sync_skew_ms", last_sync_skew_ms_);
  status.add("fused_target_count", last_fused_count_);
  status.add("projected_depth_point_count", last_projected_point_count_);
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
