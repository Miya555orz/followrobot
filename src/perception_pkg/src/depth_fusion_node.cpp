#include "perception_pkg/depth_fusion_node.hpp"
#include "perception_pkg/qos.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Geometry>
#include <algorithm>

namespace perception_pkg
{

DepthFusionNode::DepthFusionNode(const rclcpp::NodeOptions & options)
: Node("depth_fusion_node", options)
{
  sony_frame_ = declare_parameter("sony_frame", "sony_camera_optical_frame");
  gemini_depth_frame_ = declare_parameter("gemini_depth_frame", "camera_depth_optical_frame");
  default_depth_assume_ = declare_parameter("default_depth_assume", 2.0);
  roi_expand_margin_ = declare_parameter("roi_expand_margin", 0.20);
  min_roi_pixels_ = declare_parameter("min_roi_pixels", 25);
  min_depth_m_ = declare_parameter("min_depth_meters", 0.30);
  max_depth_m_ = declare_parameter("max_depth_meters", 10.0);
  min_valid_depth_ratio_ = declare_parameter("min_valid_depth_ratio", 0.30);
  depth_percentile_ = declare_parameter("depth_percentile", 0.50);
  depth_temporal_alpha_ = declare_parameter("depth_temporal_alpha", 0.30);
  publish_debug_ = declare_parameter("publish_debug_image", false);
  tracks_timeout_s_ = declare_parameter("tracks_timeout_seconds", 0.5);
  depth_timeout_s_ = declare_parameter("depth_timeout_seconds", 1.0);

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  tracks_sub_ = create_subscription<vision_servo_msgs::msg::TargetArray>(
    "/perception/tracks", perception_qos::reliable_keep_last(1),
    std::bind(&DepthFusionNode::tracks_callback, this, std::placeholders::_1));

  depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/depth/image_raw", perception_qos::best_effort_keep_last(1),
    std::bind(&DepthFusionNode::depth_callback, this, std::placeholders::_1));

  depth_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/camera/depth/camera_info", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DepthFusionNode::depth_info_callback, this, std::placeholders::_1));

  sony_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    "/sony/camera_info", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&DepthFusionNode::sony_info_callback, this, std::placeholders::_1));

  targets_3d_pub_ = create_publisher<vision_servo_msgs::msg::TargetArray>(
    "/perception/targets_3d", perception_qos::reliable_keep_last(1));

  if (publish_debug_) {
    debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
      "/perception/targets_3d_debug", 1);
  }

  RCLCPP_INFO(get_logger(), "depth_fusion_node started. Waiting for TF: %s -> %s",
              sony_frame_.c_str(), gemini_depth_frame_.c_str());
}

// ==================== callbacks ====================

void DepthFusionNode::tracks_callback(const vision_servo_msgs::msg::TargetArray::SharedPtr msg)
{
  last_tracks_ = msg;
  if (depth_image_.empty()) return;
  if (!sony_info_ready_ || !depth_info_ready_) return;
  process_frame();
}

void DepthFusionNode::depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    depth_image_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1)->image;
    depth_stamp_ = msg->header.stamp;
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "cv_bridge: %s", e.what());
  }
}

void DepthFusionNode::depth_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (depth_info_ready_) return;
  K_depth_ = cv::Mat(3, 3, CV_64F);
  std::memcpy(K_depth_.data, msg->k.data(), 9 * sizeof(double));
  D_depth_ = cv::Mat(1, 5, CV_64F);
  std::memcpy(D_depth_.data, msg->d.data(), 5 * sizeof(double));
  depth_info_ready_ = true;
  RCLCPP_INFO(get_logger(), "Depth intrinsics: %dx%d", msg->width, msg->height);
}

void DepthFusionNode::sony_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
{
  if (sony_info_ready_) return;
  K_sony_ = cv::Mat(3, 3, CV_64F);
  std::memcpy(K_sony_.data, msg->k.data(), 9 * sizeof(double));
  D_sony_ = cv::Mat(1, 5, CV_64F);
  std::memcpy(D_sony_.data, msg->d.data(), 5 * sizeof(double));
  sony_info_ready_ = true;
  RCLCPP_INFO(get_logger(), "Sony intrinsics: %dx%d", msg->width, msg->height);
}

// ==================== core ====================

void DepthFusionNode::process_frame()
{
  // Refresh TF if not yet cached
  static bool tf_cached = false;
  static Eigen::Isometry3d T_s2d = Eigen::Isometry3d::Identity();

  if (!tf_cached) {
    try {
      auto tf = tf_buffer_->lookupTransform(
        gemini_depth_frame_, sony_frame_, tf2::TimePointZero, tf2::Duration(0));
      Eigen::Isometry3d T_d2s = tf2::transformToEigen(tf).cast<double>();
      T_s2d = T_d2s.inverse();
      tf_cached = true;
      RCLCPP_INFO(get_logger(), "TF cached: %s -> %s", sony_frame_.c_str(), gemini_depth_frame_.c_str());
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF not ready: %s", e.what());
      return;
    }
  }

  auto out = vision_servo_msgs::msg::TargetArray();
  out.header = last_tracks_->header;
  out.tracking_id = last_tracks_->tracking_id;

  for (auto & t : last_tracks_->targets) {
    auto t3d = t;

    if (t.tracking_state != vision_servo_msgs::msg::Target::TRACKING_STATE_CONFIRMED ||
        !t.visible || t.bbox[2] <= t.bbox[0] || t.bbox[3] <= t.bbox[1]) {
      t3d.position = {{0.0f, 0.0f, 0.0f}};
      t3d.depth_confidence = 0.0f;
      out.targets.push_back(t3d);
      continue;
    }

    // Step 1: project bbox corners via TF to depth image, get ROI
    cv::Rect roi;
    if (!project_bbox_to_depth(t, roi, default_depth_assume_)) {
      t3d.position = {{0.0f, 0.0f, 0.0f}};
      t3d.depth_confidence = 0.0f;
      out.targets.push_back(t3d);
      continue;
    }

    // Step 2: sample depth median in ROI
    float valid_ratio = 0.0f;
    float depth_median = sample_depth_median(depth_image_, roi, t, valid_ratio);

    if (valid_ratio < min_valid_depth_ratio_ ||
        depth_median < static_cast<float>(min_depth_m_) ||
        depth_median > static_cast<float>(max_depth_m_)) {
      t3d.position = {{0.0f, 0.0f, 0.0f}};
      t3d.depth_confidence = 0.0f;
      out.targets.push_back(t3d);
      continue;
    }

    // Step 3: temporal smoothing
    double alpha = depth_temporal_alpha_;
    if (t.id == last_target_id_ && depth_filtered_ > 0.0) {
      depth_filtered_ = alpha * depth_median + (1.0 - alpha) * depth_filtered_;
    } else {
      depth_filtered_ = depth_median;
    }
    last_target_id_ = t.id;

    // Step 4: back-project Sony bbox center to 3D using fused depth
    cv::Vec3f p3d;
    float cx = t.center[0];
    float cy = t.center[1];
    if (!back_project(cx, cy, static_cast<float>(depth_filtered_), K_sony_, p3d)) {
      t3d.position = {{0.0f, 0.0f, 0.0f}};
      t3d.depth_confidence = 0.0f;
      out.targets.push_back(t3d);
      continue;
    }

    t3d.position = {{p3d[0], p3d[1], p3d[2]}};
    t3d.depth_confidence = std::min(valid_ratio / static_cast<float>(min_valid_depth_ratio_), 1.0f);
    out.targets.push_back(t3d);
  }

  targets_3d_pub_->publish(out);
}

bool DepthFusionNode::project_bbox_to_depth(
  const vision_servo_msgs::msg::Target & target,
  cv::Rect & roi,
  double assume_depth_m) const
{
  // Look up transform (static, cached in caller via static variable in process_frame)
  Eigen::Isometry3d T_s2d;
  try {
    auto tf = tf_buffer_->lookupTransform(
      gemini_depth_frame_, sony_frame_, tf2::TimePointZero, tf2::Duration(0));
    T_s2d = tf2::transformToEigen(tf).cast<double>().inverse();
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "TF error: %s", e.what());
    return false;
  }

  // Unproject 4 bbox corners to Sony 3D rays at assumed depth, then project to depth image
  double fx = K_sony_.at<double>(0, 0);
  double fy = K_sony_.at<double>(1, 1);
  double cx = K_sony_.at<double>(0, 2);
  double cy = K_sony_.at<double>(1, 2);

  double dfx = K_depth_.at<double>(0, 0);
  double dfy = K_depth_.at<double>(1, 1);
  double dcx = K_depth_.at<double>(0, 2);
  double dcy = K_depth_.at<double>(1, 2);

  std::vector<cv::Point2i> projected_corners;
  float corners[4][2] = {
    {target.bbox[0], target.bbox[1]},  // top-left
    {target.bbox[2], target.bbox[1]},  // top-right
    {target.bbox[2], target.bbox[3]},  // bottom-right
    {target.bbox[0], target.bbox[3]}   // bottom-left
  };

  for (auto & corner : corners) {
    // Sony pixel -> Sony 3D at assumed depth
    double xs = (corner[0] - cx) / fx * assume_depth_m;
    double ys = (corner[1] - cy) / fy * assume_depth_m;
    double zs = assume_depth_m;

    // Transform to Gemini depth frame
    Eigen::Vector3d ps(xs, ys, zs);
    Eigen::Vector3d pd = T_s2d * ps;

    if (pd.z() <= 0.01) continue;

    // Project to depth pixel
    int ud = static_cast<int>(dfx * pd.x() / pd.z() + dcx + 0.5);
    int vd = static_cast<int>(dfy * pd.y() / pd.z() + dcy + 0.5);

    if (ud >= 0 && ud < depth_image_.cols && vd >= 0 && vd < depth_image_.rows) {
      projected_corners.push_back(cv::Point2i(ud, vd));
    }
  }

  if (projected_corners.size() < 3) return false;

  // Compute AABB and expand by margin
  int min_x = projected_corners[0].x, max_x = projected_corners[0].x;
  int min_y = projected_corners[0].y, max_y = projected_corners[0].y;
  for (auto & p : projected_corners) {
    min_x = std::min(min_x, p.x);
    max_x = std::max(max_x, p.x);
    min_y = std::min(min_y, p.y);
    max_y = std::max(max_y, p.y);
  }

  int expand_w = static_cast<int>((max_x - min_x) * roi_expand_margin_);
  int expand_h = static_cast<int>((max_y - min_y) * roi_expand_margin_);

  roi.x = std::max(0, min_x - expand_w);
  roi.y = std::max(0, min_y - expand_h);
  roi.width = std::min(max_x + expand_w, depth_image_.cols - 1) - roi.x;
  roi.height = std::min(max_y + expand_h, depth_image_.rows - 1) - roi.y;

  return roi.width >= min_roi_pixels_ && roi.height >= min_roi_pixels_;
}

float DepthFusionNode::sample_depth_median(
  const cv::Mat & depth_img,
  const cv::Rect & roi,
  const vision_servo_msgs::msg::Target & /*target*/,
  float & valid_ratio) const
{
  valid_ratio = 0.0f;
  if (roi.width < min_roi_pixels_ || roi.height < min_roi_pixels_) return 0.0f;

  std::vector<float> samples;
  samples.reserve(roi.area());
  int valid = 0, total = roi.area();

  for (int v = roi.y; v < roi.y + roi.height && v < depth_img.rows; ++v) {
    const uint16_t * row = depth_img.ptr<uint16_t>(v);
    for (int u = roi.x; u < roi.x + roi.width && u < depth_img.cols; ++u) {
      float z = row[u] * 0.001f;  // mm -> m
      if (z > static_cast<float>(min_depth_m_) && z < static_cast<float>(max_depth_m_)) {
        samples.push_back(z);
        ++valid;
      }
    }
  }

  valid_ratio = static_cast<float>(valid) / static_cast<float>(std::max(total, 1));
  if (samples.empty()) return 0.0f;

  size_t k = static_cast<size_t>(samples.size() * depth_percentile_);
  std::nth_element(samples.begin(), samples.begin() + k, samples.end());
  return samples[k];
}

bool DepthFusionNode::back_project(
  float u, float v, float depth_m,
  const cv::Mat & K,
  cv::Vec3f & p3d) const
{
  double fx = K.at<double>(0, 0);
  double fy = K.at<double>(1, 1);
  double cx = K.at<double>(0, 2);
  double cy = K.at<double>(1, 2);
  p3d[0] = (u - static_cast<float>(cx)) / static_cast<float>(fx) * depth_m;
  p3d[1] = (v - static_cast<float>(cy)) / static_cast<float>(fy) * depth_m;
  p3d[2] = depth_m;
  return true;
}

}  // namespace perception_pkg

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(perception_pkg::DepthFusionNode)

// Standalone entry point
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<perception_pkg::DepthFusionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
