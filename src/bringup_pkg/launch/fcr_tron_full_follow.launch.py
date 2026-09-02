"""FCR full-follow command path prepared for an external TRON1 controller.

This launch keeps the legacy three-wheel chassis driver disabled. FCR still
uses command_mux for safety, but its TwistStamped output is moved away from
/cmd_vel. The TRON controller should subscribe to the limiter output topic
(/fcr_tron/cmd_vel by default), avoiding type conflicts and stray /cmd_vel
publishers during bring-up.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    fcr = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("bringup_pkg"), "launch", "fcr_mvp_mode.launch.py"
        ]),
        launch_arguments={
            "enable_chassis": "false",
            "can_interface": LaunchConfiguration("can_interface"),
            "gimbal_control_mode": LaunchConfiguration("gimbal_control_mode"),
            "gimbal_speed_control_byte": LaunchConfiguration(
                "gimbal_speed_control_byte"
            ),
            "enable_gimbal": LaunchConfiguration("enable_gimbal"),
            "enable_detection": LaunchConfiguration("enable_detection"),
            "enable_tracking": LaunchConfiguration("enable_tracking"),
            "enable_sony_camera": LaunchConfiguration("enable_sony_camera"),
            "enable_depth_fusion": LaunchConfiguration("enable_depth_fusion"),
            "start_gemini": LaunchConfiguration("start_gemini"),
            "enable_mvp": LaunchConfiguration("enable_mvp"),
            "gemini_depth_width": LaunchConfiguration("gemini_depth_width"),
            "gemini_depth_height": LaunchConfiguration("gemini_depth_height"),
            "gemini_depth_fps": LaunchConfiguration("gemini_depth_fps"),
            "mvp_config": PathJoinSubstitution([
                FindPackageShare("servo_control_pkg"),
                "config",
                "mvp_full_follow.yaml",
            ]),
            "mux_cmd_vel_output_topic": "/fcr/cmd_vel_stamped",
            "use_foxglove": LaunchConfiguration("use_foxglove"),
        }.items(),
    )

    tron1_safety_limiter = Node(
        package="robot_platform_pkg",
        executable="tron1_safety_limiter_node",
        name="tron1_safety_limiter",
        output="screen",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("robot_platform_pkg"),
                "config",
                "tron1_safety_limiter.yaml",
            ]),
            {
                "enable_motion": LaunchConfiguration("enable_motion"),
                "enable_lateral": LaunchConfiguration("enable_lateral"),
                "input_timeout_sec": LaunchConfiguration("input_timeout_sec"),
                "max_linear_x": LaunchConfiguration("max_linear_x"),
                "max_linear_y": LaunchConfiguration("max_linear_y"),
                "max_angular_z": LaunchConfiguration("max_angular_z"),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_gemini",
            default_value="true",
            description="Start Gemini depth through FCR depth_fusion launch.",
        ),
        DeclareLaunchArgument(
            "enable_gimbal",
            default_value="false",
            description="Start the DJI RS2 gimbal driver. Keep false for base-only migration.",
        ),
        DeclareLaunchArgument(
            "can_interface",
            default_value="can1",
            description="Linux SocketCAN interface used by the DJI RS2 gimbal.",
        ),
        DeclareLaunchArgument(
            "gimbal_control_mode",
            default_value="incremental_position",
            description="RS2 command mode for communication bring-up: speed or incremental_position.",
        ),
        DeclareLaunchArgument("gimbal_speed_control_byte", default_value="128"),
        DeclareLaunchArgument(
            "enable_detection",
            default_value="false",
            description="Start YOLO detection. Keep false for first TRON bridge checks.",
        ),
        DeclareLaunchArgument(
            "enable_tracking",
            default_value="false",
            description="Start target tracking. Keep false for first TRON bridge checks.",
        ),
        DeclareLaunchArgument(
            "enable_sony_camera",
            default_value="false",
            description="Start Sony RGB camera. Keep false for first TRON bridge checks.",
        ),
        DeclareLaunchArgument(
            "enable_depth_fusion",
            default_value="true",
            description="Start Gemini depth, calibrated TF, and /perception/targets_3d fusion.",
        ),
        DeclareLaunchArgument(
            "enable_mvp",
            default_value="false",
            description="Start the existing MVP follow controller. Keep false for first TRON bridge checks.",
        ),
        DeclareLaunchArgument("gemini_depth_width", default_value="424"),
        DeclareLaunchArgument("gemini_depth_height", default_value="240"),
        DeclareLaunchArgument("gemini_depth_fps", default_value="10"),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="Publish non-zero Twist to TRON. false keeps output zero.",
        ),
        DeclareLaunchArgument(
            "enable_lateral",
            default_value="false",
            description="Forward FCR linear.y to TRON. false is safer for first tests.",
        ),
        DeclareLaunchArgument("input_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("max_linear_x", default_value="0.03"),
        DeclareLaunchArgument("max_linear_y", default_value="0.03"),
        DeclareLaunchArgument("max_angular_z", default_value="0.10"),
        DeclareLaunchArgument("use_foxglove", default_value="false"),
        fcr,
        TimerAction(period=1.0, actions=[tron1_safety_limiter]),
    ])
