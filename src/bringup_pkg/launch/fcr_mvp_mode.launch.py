"""Common complete real-hardware bringup for one safe MVP control mode."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    servo_share = FindPackageShare("servo_control_pkg")

    system = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("bringup_pkg"), "launch", "fcr_bringup.launch.py"
        ]),
        launch_arguments={
            "use_sim": "false",
            "can_interface": LaunchConfiguration("can_interface"),
            "gimbal_control_mode": LaunchConfiguration("gimbal_control_mode"),
            "gimbal_speed_control_byte": LaunchConfiguration(
                "gimbal_speed_control_byte"
            ),
            "enable_imu": "false",
            "enable_chassis": LaunchConfiguration("enable_chassis"),
            "enable_gimbal": LaunchConfiguration("enable_gimbal"),
            "model_path": LaunchConfiguration("model_path"),
            "detection_device": LaunchConfiguration("detection_device"),
            "enable_camera_motion_compensation": LaunchConfiguration(
                "enable_camera_motion_compensation"
            ),
            "enable_detection": LaunchConfiguration("enable_detection"),
            "enable_tracking": LaunchConfiguration("enable_tracking"),
            "use_mock_detector": LaunchConfiguration("use_mock_detector"),
            "enable_sony_camera": LaunchConfiguration("enable_sony_camera"),
            "enable_depth_fusion": LaunchConfiguration("enable_depth_fusion"),
            "start_gemini": LaunchConfiguration("start_gemini"),
            "gemini_serial_number": LaunchConfiguration("gemini_serial_number"),
            "fusion_calibration_file": LaunchConfiguration("fusion_calibration_file"),
            "fusion_publish_debug_image": LaunchConfiguration(
                "fusion_publish_debug_image"
            ),
            "enable_servo": "false",
            "sony_camera_info_url": LaunchConfiguration("sony_camera_info_url"),
            "use_foxglove": LaunchConfiguration("use_foxglove"),
            "foxglove_address": LaunchConfiguration("foxglove_address"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
            "remote_max_frame_age_ms": LaunchConfiguration(
                "remote_max_frame_age_ms"
            ),
            "gemini_depth_width": LaunchConfiguration("gemini_depth_width"),
            "gemini_depth_height": LaunchConfiguration("gemini_depth_height"),
            "gemini_depth_fps": LaunchConfiguration("gemini_depth_fps"),
            "use_rviz": "false",
            "mux_cmd_vel_output_topic": LaunchConfiguration("mux_cmd_vel_output_topic"),
        }.items(),
    )

    mvp = IncludeLaunchDescription(
        PathJoinSubstitution([
            servo_share, "launch", "mvp_follow_core.launch.py"
        ]),
        launch_arguments={
            "mvp_config": LaunchConfiguration("mvp_config"),
            "yaw_sign": LaunchConfiguration("yaw_sign"),
            "pitch_sign": LaunchConfiguration("pitch_sign"),
            "base_yaw_sign": LaunchConfiguration("base_yaw_sign"),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can1"),
        # Fixed real-hardware strategy: MVP AUTO uses continuous speed commands.
        # Manual arrow keys use the separate GimbalNudge incremental-position path.
        DeclareLaunchArgument("gimbal_control_mode", default_value="speed"),
        DeclareLaunchArgument("gimbal_speed_control_byte", default_value="128"),
        DeclareLaunchArgument("enable_chassis", default_value="false"),
        DeclareLaunchArgument("enable_gimbal", default_value="true"),
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution([
                EnvironmentVariable("HOME"), "fcr_models", "yolov8n_fp16.engine"
            ]),
        ),
        DeclareLaunchArgument("detection_device", default_value="tensorrt"),
        DeclareLaunchArgument("enable_detection", default_value="true"),
        DeclareLaunchArgument("enable_tracking", default_value="true"),
        DeclareLaunchArgument("use_mock_detector", default_value="false"),
        DeclareLaunchArgument("enable_sony_camera", default_value="true"),
        DeclareLaunchArgument(
            "enable_camera_motion_compensation", default_value="false"
        ),
        DeclareLaunchArgument("enable_depth_fusion", default_value="false"),
        DeclareLaunchArgument("start_gemini", default_value="true"),
        DeclareLaunchArgument("gemini_serial_number", default_value=""),
        DeclareLaunchArgument(
            "fusion_calibration_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("perception_pkg"),
                "config",
                "calibration",
                "sony_gemini_extrinsics.yaml",
            ]),
        ),
        DeclareLaunchArgument("fusion_publish_debug_image", default_value="false"),
        DeclareLaunchArgument("enable_mvp", default_value="true"),
        DeclareLaunchArgument(
            "sony_camera_info_url",
            default_value=[
                TextSubstitution(text="file://"),
                PathJoinSubstitution([
                    EnvironmentVariable("HOME"),
                    ".ros",
                    "camera_info",
                    "sony_zv_e10_ii.yaml",
                ]),
            ],
        ),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        DeclareLaunchArgument("remote_max_frame_age_ms", default_value="1000"),
        DeclareLaunchArgument(
            "mvp_config",
            default_value=PathJoinSubstitution([
                servo_share, "config", "mvp_gimbal_only.yaml"
            ]),
        ),
        DeclareLaunchArgument(
            "mux_cmd_vel_output_topic",
            default_value="/cmd_vel",
            description="command_mux最终TwistStamped速度输出话题；TRON适配时改为/fcr/cmd_vel_stamped",
        ),
        DeclareLaunchArgument("gemini_depth_width", default_value="848"),
        DeclareLaunchArgument("gemini_depth_height", default_value="480"),
        DeclareLaunchArgument("gemini_depth_fps", default_value="10"),
        DeclareLaunchArgument(
            "yaw_sign",
            default_value="1.0",
            description="RS2 yaw direction multiplier (real can1 hardware: +1).",
        ),
        DeclareLaunchArgument(
            "pitch_sign",
            default_value="-1.0",
            description="RS2 pitch direction multiplier; verify with a vertical test.",
        ),
        DeclareLaunchArgument(
            "base_yaw_sign",
            default_value="-1.0",
            description="Relative gimbal yaw to ROS base angular.z direction.",
        ),
        system,
        TimerAction(period=4.0, actions=[mvp], condition=IfCondition(
            LaunchConfiguration("enable_mvp"))),
    ])
