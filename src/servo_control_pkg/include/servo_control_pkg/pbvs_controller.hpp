/**
 * @file pbvs_controller.hpp
 * @brief PBVS（基于位置的视觉伺服）控制器。
 *
 * 核心思想：使用融合得到的目标三维点，分别控制水平视线角、垂直视线角
 * 与前向距离。人体检测没有刚体姿态，因此这里不构造虚假的 6D 姿态。
 *
 * 输入位置坐标遵循 Sony optical frame：X右、Y下、Z前。
 * 角度快环使用二维瞄准点修正后的 X/Y，距离慢环使用融合深度 Z。
 */

#pragma once

#include "servo_control_pkg/servo_controller_base.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace servo_control_pkg {

class PBVSController : public ServoControllerBase {
public:
  /// 默认构造函数：使用空 NodeOptions，pluginlib 通过带参构造加载
  PBVSController() : PBVSController(rclcpp::NodeOptions()) {}

  /**
   * @brief 带参数构造，由 pluginlib::ClassLoader 调用。
   * @param options ROS2 节点选项（包含参数覆盖）
   */
  explicit PBVSController(const rclcpp::NodeOptions& options);

  /**
   * @brief 主控制迭代：根据当前目标观测计算相机速度。
   * @param target 当前跟踪目标（包含 3D 位置和边界框）
   * @param dt     距上次更新的时间间隔 (s)，PBVS 当前未使用（P 控制器无状态依赖）
   * @return 相机系 6-DOF 速度 [vx,vy,vz, ωx,ωy,ωz]^T，未初始化则返回 nullopt
   */
  std::optional<Eigen::Matrix<double, 6, 1>> computeVelocity(
    const vision_servo_msgs::msg::Target& target, double dt) override;

  /// @return 控制器类型标识字符串 "PBVS"（用于日志和模式切换）
  std::string getControllerType() const override { return "PBVS"; }

  /**
   * @brief 从宿主节点拉取参数。
   * @param node servo_manager 节点的引用，PBVS 专属参数从其读取
   * @note 由 servo_manager 在加载/切换插件后显式调用。
   */
  void configureFromNode(const rclcpp::Node& node) override;

  /**
   * @brief 用当前目标观测生成一个伺服目标。
   * @param target           当前目标观测（3D 位置 + bbox）
   * @param desired_depth    期望深度 (m)，目标将始终被保持在相机正前方此距离
   * @param feature_tolerance 收敛阈值，误差范数低于此值视为已跟踪到位
   * @return 是否成功设置目标
   */
  bool setGoalFromTarget(
    const vision_servo_msgs::msg::Target& target,
    double desired_depth,
    double feature_tolerance) override;

  /**
   * @brief 使用相机内参初始化控制器。
   * @param fx, fy 焦距（像素），来自 CameraInfo::k[0], k[4]
   * @param cx, cy 主点（像素），来自 CameraInfo::k[2], k[5]
   * @param width, height 图像分辨率（像素）
   * @return 初始化是否成功
   */
  bool initialize(double fx, double fy, double cx, double cy,
                  int width, int height) override;

  bool isConverged() const override;

private:
  // ── PBVS 控制参数 ──────────────────────────────────────────────
  double translational_gain_;
  double rotational_gain_;
  double pitch_gain_;
  double pitch_filter_alpha_;
  double pitch_acceleration_limit_;
  double depth_filter_alpha_;
  double linear_acceleration_limit_;
  double lateral_gain_;
  double yaw_deadband_rad_;
  double pitch_deadband_rad_;
  double depth_deadband_m_;
  bool enable_lateral_translation_;

  // Errors are controller-specific PBVS values:
  // [bearing_yaw, bearing_pitch, depth, lateral_x, lateral_y, 0].
  double last_yaw_error_{0.0};
  double last_pitch_error_{0.0};
  double filtered_pitch_error_{0.0};
  double last_pitch_velocity_{0.0};
  double last_depth_error_{0.0};
  double filtered_depth_error_{0.0};
  double last_linear_velocity_{0.0};
  bool point_goal_set_;
};

}  // namespace servo_control_pkg
