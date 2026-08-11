#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <stdexcept>
#include <vector>

#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vision_servo_msgs/msg/camera_recording_state.hpp>
#include <vision_servo_msgs/msg/system_state.hpp>
#include <vision_servo_msgs/srv/set_camera_recording.hpp>

#include "sony_camera_pkg/sony_camera_streamer.hpp"

namespace sony_camera_pkg {
namespace {

using SteadyClock = std::chrono::steady_clock;

rcl_interfaces::msg::ParameterDescriptor descriptor(const std::string& description) {
  rcl_interfaces::msg::ParameterDescriptor result;
  result.description = description;
  result.read_only = true;
  return result;
}

rcl_interfaces::msg::ParameterDescriptor integer_descriptor(
    const std::string& description, int64_t minimum, int64_t maximum) {
  auto result = descriptor(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  range.step = 1;
  result.integer_range.push_back(range);
  return result;
}

rcl_interfaces::msg::ParameterDescriptor floating_descriptor(
    const std::string& description, double minimum, double maximum) {
  auto result = descriptor(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = minimum;
  range.to_value = maximum;
  result.floating_point_range.push_back(range);
  return result;
}

int64_t steady_nanoseconds() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      SteadyClock::now().time_since_epoch()).count();
}

bool calibration_intrinsics_valid(const sensor_msgs::msg::CameraInfo& info) {
  return std::isfinite(info.k[0]) && std::isfinite(info.k[4]) &&
      std::isfinite(info.k[2]) && std::isfinite(info.k[5]) &&
      info.k[0] > 0.0 && info.k[4] > 0.0;
}

}  // namespace

class SonyCameraNode final : public rclcpp::Node {
public:
  SonyCameraNode()
      : Node("sony_camera_node"), diagnostics_(this) {
    camera_index_ = declare_parameter<int>(
        "camera_index", 1, integer_descriptor("One-based CRSDK camera index.", 1, 64));
    camera_name_ = declare_parameter<std::string>(
        "camera_name", "sony_zv_e10_ii", descriptor("Name stored in CameraInfoManager."));
    camera_frame_ = declare_parameter<std::string>(
        "camera_frame", "sony_camera_optical_frame",
        descriptor("ROS optical frame used by Image and CameraInfo."));
    camera_info_url_ = declare_parameter<std::string>(
        "camera_info_url", "", descriptor("camera_info_manager calibration URL."));
    publish_rate_ = declare_parameter<double>(
        "publish_rate", 30.0, floating_descriptor("Maximum published live-view rate.", 0.1, 120.0));
    reconnect_initial_delay_ = declare_parameter<double>(
        "reconnect_initial_delay", 1.0,
        floating_descriptor("Seconds before the first reconnect attempt.", 0.1, 60.0));
    reconnect_max_delay_ = declare_parameter<double>(
        "reconnect_max_delay", 30.0,
        floating_descriptor("Maximum exponential reconnect delay in seconds.", 0.1, 600.0));
    stream_stall_timeout_ = declare_parameter<double>(
        "stream_stall_timeout", 5.0,
        floating_descriptor("Reconnect when no decoded frame arrives for this many seconds.", 1.0, 120.0));
    if (camera_name_.empty() || camera_frame_.empty()) {
      throw std::invalid_argument("camera_name and camera_frame must not be empty");
    }
    if (reconnect_max_delay_ < reconnect_initial_delay_) {
      throw std::invalid_argument(
          "reconnect_max_delay must be greater than or equal to reconnect_initial_delay");
    }

    image_pub_ = image_transport::create_publisher(
        this, "image_raw", rclcpp::SensorDataQoS().keep_last(1).get_rmw_qos_profile());
    camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(
        "camera_info", rclcpp::SensorDataQoS().keep_last(1));
    recording_state_pub_ = create_publisher<vision_servo_msgs::msg::CameraRecordingState>(
        "/sony/recording_status", rclcpp::QoS(1).reliable().transient_local());
    recording_service_ = create_service<vision_servo_msgs::srv::SetCameraRecording>(
        "/sony/set_recording",
        std::bind(&SonyCameraNode::set_recording_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    take_photo_service_ = create_service<std_srvs::srv::Trigger>(
        "/sony/take_photo",
        std::bind(&SonyCameraNode::take_photo_callback, this,
                  std::placeholders::_1, std::placeholders::_2));
    system_state_sub_ = create_subscription<vision_servo_msgs::msg::SystemState>(
        "/system/state", rclcpp::QoS(1).reliable().transient_local(),
        [this](const vision_servo_msgs::msg::SystemState::ConstSharedPtr state) {
          cinematic_recording_lock_ =
              state->mode == vision_servo_msgs::msg::SystemState::MODE_CINEMATIC &&
              state->cinematic_state ==
                  vision_servo_msgs::msg::SystemState::CINEMATIC_EXECUTING;
        });

    camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        this, camera_name_, camera_info_url_);
    // This also creates the standard relative set_camera_info service. Read the
    // manager on every frame so a successful calibration request takes effect
    // immediately; CameraInfoManager protects its state internally.
    (void)camera_info_manager_->getCameraInfo();

    diagnostics_.setHardwareID(camera_name_);
    diagnostics_.add("sony_camera", this, &SonyCameraNode::diagnostic_callback);
    maintenance_timer_ = create_wall_timer(
        std::chrono::seconds(1), std::bind(&SonyCameraNode::maintenance_callback, this));
    reconnect_delay_ = reconnect_initial_delay_;
  }

  ~SonyCameraNode() override {
    streamer_.release();
  }

  bool start() {
    if (!streamer_.init()) {
      RCLCPP_FATAL(get_logger(), "Sony CRSDK initialization failed");
      return false;
    }
    if (!connect_and_stream()) {
      schedule_reconnect();
      RCLCPP_WARN(
          get_logger(), "Initial Sony camera connection failed; automatic reconnect is active");
    }
    return true;
  }

private:
  bool connect_and_stream() {
    streamer_.disconnect();
    if (!streamer_.connect(camera_index_)) {
      return false;
    }
    if (!streamer_.startStreaming(
            [this](const std::vector<uint8_t>& jpeg, uint32_t width, uint32_t height) {
              frame_callback(jpeg, width, height);
            })) {
      RCLCPP_ERROR(get_logger(), "Sony live-view stream failed to start");
      streamer_.disconnect();
      return false;
    }

    reconnect_delay_ = reconnect_initial_delay_;
    stream_started_steady_ns_.store(steady_nanoseconds(), std::memory_order_relaxed);
    RCLCPP_INFO(
        get_logger(), "Sony live view connected: image_raw + camera_info, max_rate=%.1f Hz",
        publish_rate_);
    return true;
  }

  void schedule_reconnect() {
    next_reconnect_ = SteadyClock::now() + std::chrono::duration_cast<SteadyClock::duration>(
        std::chrono::duration<double>(reconnect_delay_));
    reconnect_delay_ = std::min(reconnect_delay_ * 2.0, reconnect_max_delay_);
  }

  void maintenance_callback() {
    refresh_recording_state();
    if (streamer_.isConnected()) {
      const int64_t last_frame = last_frame_steady_ns_.load(std::memory_order_relaxed);
      const int64_t stream_started =
          stream_started_steady_ns_.load(std::memory_order_relaxed);
      const int64_t reference = std::max(last_frame, stream_started);
      const double frame_silence = reference > 0
          ? static_cast<double>(steady_nanoseconds() - reference) * 1.0e-9
          : 0.0;
      if (frame_silence <= stream_stall_timeout_) {
        return;
      }
      RCLCPP_ERROR(
          get_logger(), "Sony live view stalled for %.1f s; reconnecting", frame_silence);
      streamer_.disconnect();
    }
    if (SteadyClock::now() < next_reconnect_) {
      return;
    }

    RCLCPP_WARN(get_logger(), "Attempting to reconnect Sony camera");
    if (!connect_and_stream()) {
      schedule_reconnect();
    }
  }

  static uint8_t recording_message_state(RecordingState state) {
    using Message = vision_servo_msgs::msg::CameraRecordingState;
    switch (state) {
      case RecordingState::Stopped: return Message::STOPPED;
      case RecordingState::Recording: return Message::RECORDING;
      case RecordingState::Error: return Message::ERROR;
      case RecordingState::IntervalWaiting: return Message::STARTING;
      default: return Message::UNKNOWN;
    }
  }

  static const char* recording_state_name(uint8_t state) {
    using Message = vision_servo_msgs::msg::CameraRecordingState;
    switch (state) {
      case Message::STOPPED: return "STOPPED";
      case Message::STARTING: return "STARTING";
      case Message::RECORDING: return "RECORDING";
      case Message::STOPPING: return "STOPPING";
      case Message::ERROR: return "ERROR";
      default: return "UNKNOWN";
    }
  }

  void publish_recording_state(uint8_t state, uint32_t sdk_code, const std::string& detail) {
    vision_servo_msgs::msg::CameraRecordingState message;
    message.header.stamp = now();
    message.header.frame_id = camera_frame_;
    message.state = state;
    message.state_name = recording_state_name(state);
    message.connected = streamer_.isConnected();
    message.command_pending = recording_command_pending_;
    message.sdk_code = sdk_code;
    message.detail = detail;
    recording_state_pub_->publish(message);
    last_recording_state_ = state;
    last_recording_sdk_code_ = sdk_code;
  }

  void refresh_recording_state() {
    using Message = vision_servo_msgs::msg::CameraRecordingState;
    if (!streamer_.isConnected()) {
      recording_command_pending_ = false;
      publish_recording_state(Message::UNKNOWN, 0, "camera_disconnected");
      return;
    }
    uint32_t sdk_code = 0;
    const uint8_t observed = recording_message_state(streamer_.recordingState(sdk_code));
    if (observed == Message::UNKNOWN && sdk_code != 0) {
      publish_recording_state(Message::ERROR, sdk_code, "recording_state_query_failed");
      return;
    }
    if (recording_command_pending_) {
      const bool reached = pending_recording_target_
          ? observed == Message::RECORDING
          : observed == Message::STOPPED;
      if (reached) {
        recording_command_pending_ = false;
      } else if (SteadyClock::now() >= recording_command_deadline_) {
        recording_command_pending_ = false;
        publish_recording_state(Message::ERROR, sdk_code, "recording_transition_timeout");
        return;
      } else {
        publish_recording_state(
            pending_recording_target_ ? Message::STARTING : Message::STOPPING,
            sdk_code, "waiting_for_camera_confirmation");
        return;
      }
    }
    publish_recording_state(observed, sdk_code, "camera_property_confirmed");
  }

  void set_recording_callback(
      const std::shared_ptr<vision_servo_msgs::srv::SetCameraRecording::Request> request,
      std::shared_ptr<vision_servo_msgs::srv::SetCameraRecording::Response> response) {
    using Message = vision_servo_msgs::msg::CameraRecordingState;
    if (!streamer_.isConnected()) {
      response->accepted = false;
      response->state = Message::UNKNOWN;
      response->message = "Sony camera is disconnected";
      return;
    }
    if (!request->recording && cinematic_recording_lock_) {
      response->accepted = false;
      response->state = last_recording_state_;
      response->sdk_code = last_recording_sdk_code_;
      response->message = "Recording is locked while cinematic action is executing";
      return;
    }
    const bool already_reached = request->recording
        ? last_recording_state_ == Message::RECORDING
        : last_recording_state_ == Message::STOPPED;
    if (already_reached && !recording_command_pending_) {
      response->accepted = true;
      response->state = last_recording_state_;
      response->sdk_code = last_recording_sdk_code_;
      response->message = "Recording state already matches request";
      return;
    }
    if (recording_command_pending_) {
      response->accepted = false;
      response->state = last_recording_state_;
      response->sdk_code = last_recording_sdk_code_;
      response->message = "Another recording transition is pending";
      return;
    }
    uint32_t sdk_code = 0;
    if (!streamer_.setRecording(request->recording, sdk_code)) {
      response->accepted = false;
      response->state = Message::ERROR;
      response->sdk_code = sdk_code;
      response->message = "CRSDK rejected movie recording command";
      publish_recording_state(Message::ERROR, sdk_code, response->message);
      return;
    }
    pending_recording_target_ = request->recording;
    recording_command_pending_ = true;
    recording_command_deadline_ = SteadyClock::now() + std::chrono::seconds(5);
    const uint8_t transition_state = request->recording ? Message::STARTING : Message::STOPPING;
    publish_recording_state(
        transition_state, sdk_code,
        request->requester.empty() ? "recording_command_accepted" : request->requester);
    response->accepted = true;
    response->state = transition_state;
    response->sdk_code = sdk_code;
    response->message = "Command accepted; wait for recording_status confirmation";
  }

  void take_photo_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    uint32_t sdk_code = 0;
    response->success = streamer_.takePhoto(sdk_code);
    std::ostringstream message;
    message << (response->success ? "Photo command accepted" : "Photo command failed")
            << " (sdk=0x" << std::hex << std::uppercase << sdk_code << ")";
    response->message = message.str();
  }

  void frame_callback(
      const std::vector<uint8_t>& jpeg_data, uint32_t reported_width, uint32_t reported_height) {
    const int64_t now_steady = steady_nanoseconds();
    const int64_t minimum_period = static_cast<int64_t>(1.0e9 / publish_rate_);
    const int64_t previous_publish = last_publish_steady_ns_.load(std::memory_order_relaxed);
    if (previous_publish > 0 && now_steady - previous_publish < minimum_period) {
      dropped_by_rate_.fetch_add(1, std::memory_order_relaxed);
      return;
    }

    cv::Mat frame = cv::imdecode(jpeg_data, cv::IMREAD_COLOR);
    if (frame.empty()) {
      decode_failures_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 5000, "Failed to decode Sony live-view JPEG frame");
      return;
    }

    if (reported_width > 0 && reported_height > 0 &&
        (frame.cols != static_cast<int>(reported_width) ||
         frame.rows != static_cast<int>(reported_height))) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "CRSDK reported %ux%u but JPEG decoded as %dx%d",
          reported_width, reported_height, frame.cols, frame.rows);
    }

    rclcpp::Time stamp = now();
    {
      std::lock_guard<std::mutex> lock(stamp_mutex_);
      if (last_stamp_.nanoseconds() > 0 && stamp <= last_stamp_) {
        stamp = rclcpp::Time(last_stamp_.nanoseconds() + 1, get_clock()->get_clock_type());
      }
      last_stamp_ = stamp;
    }

    std_msgs::msg::Header header;
    header.stamp = stamp;
    header.frame_id = camera_frame_;
    auto image = cv_bridge::CvImage(
        header, sensor_msgs::image_encodings::BGR8, frame).toImageMsg();

    auto camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header = header;
    if (camera_info.width == 0 || camera_info.height == 0) {
      camera_info.width = static_cast<uint32_t>(frame.cols);
      camera_info.height = static_cast<uint32_t>(frame.rows);
      calibration_valid_for_stream_.store(false, std::memory_order_relaxed);
    } else if (
        camera_info.width != static_cast<uint32_t>(frame.cols) ||
        camera_info.height != static_cast<uint32_t>(frame.rows)) {
      calibration_mismatches_.fetch_add(1, std::memory_order_relaxed);
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Calibration resolution %ux%u does not match Sony live view %dx%d",
          camera_info.width, camera_info.height, frame.cols, frame.rows);
      // Never publish valid-looking intrinsics for a different resolution.
      camera_info = sensor_msgs::msg::CameraInfo();
      camera_info.header = header;
      camera_info.width = static_cast<uint32_t>(frame.cols);
      camera_info.height = static_cast<uint32_t>(frame.rows);
      calibration_valid_for_stream_.store(false, std::memory_order_relaxed);
    } else if (!calibration_intrinsics_valid(camera_info)) {
      RCLCPP_ERROR_THROTTLE(
          get_logger(), *get_clock(), 10000,
          "Calibration dimensions match the stream but focal lengths are invalid");
      camera_info = sensor_msgs::msg::CameraInfo();
      camera_info.header = header;
      camera_info.width = static_cast<uint32_t>(frame.cols);
      camera_info.height = static_cast<uint32_t>(frame.rows);
      calibration_valid_for_stream_.store(false, std::memory_order_relaxed);
    } else {
      calibration_valid_for_stream_.store(true, std::memory_order_relaxed);
    }

    image_pub_.publish(image);
    camera_info_pub_->publish(camera_info);
    last_width_.store(static_cast<uint32_t>(frame.cols), std::memory_order_relaxed);
    last_height_.store(static_cast<uint32_t>(frame.rows), std::memory_order_relaxed);
    last_publish_steady_ns_.store(now_steady, std::memory_order_relaxed);
    last_frame_steady_ns_.store(now_steady, std::memory_order_relaxed);
    published_frames_.fetch_add(1, std::memory_order_relaxed);
    frames_since_diagnostic_.fetch_add(1, std::memory_order_relaxed);
  }

  void diagnostic_callback(diagnostic_updater::DiagnosticStatusWrapper& status) {
    const bool connected = streamer_.isConnected();
    const int64_t last_frame = last_frame_steady_ns_.load(std::memory_order_relaxed);
    const double frame_age = last_frame > 0
        ? static_cast<double>(steady_nanoseconds() - last_frame) * 1.0e-9
        : -1.0;
    const uint64_t recent_frames =
        frames_since_diagnostic_.exchange(0, std::memory_order_relaxed);

    if (!streamer_.isInitialized()) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "CRSDK is not initialized");
    } else if (!connected) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Camera disconnected; reconnecting");
    } else if (frame_age < 0.0 || frame_age > stream_stall_timeout_) {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Camera connected but frames are stale");
    } else {
      status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
    }

    status.add("connected", connected);
    status.add("published_frames_total", published_frames_.load(std::memory_order_relaxed));
    status.add("published_frames_last_second", recent_frames);
    status.add("decode_failures", decode_failures_.load(std::memory_order_relaxed));
    status.add("consecutive_capture_failures", streamer_.consecutiveCaptureFailures());
    status.add("rate_limited_frames", dropped_by_rate_.load(std::memory_order_relaxed));
    status.add(
        "calibration_resolution_mismatches",
        calibration_mismatches_.load(std::memory_order_relaxed));
    status.add("last_frame_age_seconds", frame_age);
    status.add("current_width", last_width_.load(std::memory_order_relaxed));
    status.add("current_height", last_height_.load(std::memory_order_relaxed));
    status.add("camera_frame", camera_frame_);
    status.add(
        "calibration_valid_for_stream",
        calibration_valid_for_stream_.load(std::memory_order_relaxed));
    status.add("recording_state", recording_state_name(last_recording_state_));
    status.add("recording_command_pending", recording_command_pending_);
  }

  int camera_index_ = 1;
  std::string camera_name_;
  std::string camera_frame_;
  std::string camera_info_url_;
  double publish_rate_ = 30.0;
  double reconnect_initial_delay_ = 1.0;
  double reconnect_max_delay_ = 30.0;
  double stream_stall_timeout_ = 5.0;
  double reconnect_delay_ = 1.0;
  SteadyClock::time_point next_reconnect_ = SteadyClock::time_point::min();

  SonyCameraStreamer streamer_;
  image_transport::Publisher image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<vision_servo_msgs::msg::CameraRecordingState>::SharedPtr
      recording_state_pub_;
  rclcpp::Service<vision_servo_msgs::srv::SetCameraRecording>::SharedPtr
      recording_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr take_photo_service_;
  rclcpp::Subscription<vision_servo_msgs::msg::SystemState>::SharedPtr
      system_state_sub_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  diagnostic_updater::Updater diagnostics_;
  rclcpp::TimerBase::SharedPtr maintenance_timer_;

  std::mutex stamp_mutex_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  std::atomic<int64_t> last_publish_steady_ns_{0};
  std::atomic<int64_t> last_frame_steady_ns_{0};
  std::atomic<int64_t> stream_started_steady_ns_{0};
  std::atomic<uint64_t> published_frames_{0};
  std::atomic<uint64_t> frames_since_diagnostic_{0};
  std::atomic<uint64_t> decode_failures_{0};
  std::atomic<uint64_t> dropped_by_rate_{0};
  std::atomic<uint64_t> calibration_mismatches_{0};
  std::atomic<uint32_t> last_width_{0};
  std::atomic<uint32_t> last_height_{0};
  std::atomic<bool> calibration_valid_for_stream_{false};
  bool recording_command_pending_{false};
  bool pending_recording_target_{false};
  bool cinematic_recording_lock_{false};
  uint8_t last_recording_state_{
      vision_servo_msgs::msg::CameraRecordingState::UNKNOWN};
  uint32_t last_recording_sdk_code_{0};
  SteadyClock::time_point recording_command_deadline_{
      SteadyClock::time_point::min()};
};

}  // namespace sony_camera_pkg

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sony_camera_pkg::SonyCameraNode>();
  if (!node->start()) {
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::spin(node);
  node.reset();
  rclcpp::shutdown();
  return 0;
}
