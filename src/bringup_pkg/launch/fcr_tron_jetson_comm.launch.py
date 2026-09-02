"""Safe full communication bring-up for TRON1 + Jetson + DJI RS2 + Gemini depth.

This launch intentionally builds the ROS/DDS communication graph before any
base motion is allowed. The TRON1 controller is optional and, when enabled, is
forced to consume only /fcr_tron/cmd_vel from the safety limiter.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    jetson_fcr = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("bringup_pkg"), "launch", "fcr_tron_full_follow.launch.py"
        ]),
        launch_arguments={
            "enable_gimbal": LaunchConfiguration("enable_gimbal"),
            "can_interface": LaunchConfiguration("rs2_can_interface"),
            "gimbal_control_mode": LaunchConfiguration("gimbal_control_mode"),
            "gimbal_speed_control_byte": LaunchConfiguration("gimbal_speed_control_byte"),
            "enable_detection": LaunchConfiguration("enable_detection"),
            "enable_tracking": LaunchConfiguration("enable_tracking"),
            "enable_sony_camera": LaunchConfiguration("enable_sony_camera"),
            "enable_depth_fusion": LaunchConfiguration("enable_depth_fusion"),
            "enable_mvp": LaunchConfiguration("enable_mvp"),
            "enable_motion": LaunchConfiguration("enable_motion"),
            "enable_lateral": "false",
            "start_gemini": LaunchConfiguration("start_depth_camera"),
            "gemini_serial_number": LaunchConfiguration("depth_camera_serial_number"),
            "gemini_depth_width": LaunchConfiguration("depth_width"),
            "gemini_depth_height": LaunchConfiguration("depth_height"),
            "gemini_depth_fps": LaunchConfiguration("depth_fps"),
            "max_linear_x": LaunchConfiguration("max_linear_x"),
            "max_linear_y": "0.0",
            "max_angular_z": LaunchConfiguration("max_angular_z"),
            "input_timeout_sec": LaunchConfiguration("input_timeout_sec"),
            "use_foxglove": LaunchConfiguration("use_foxglove"),
        }.items(),
    )

    tron_hw = ExecuteProcess(
        cmd=[
            "ros2",
            "launch",
            "robot_hw",
            "pointfoot_hw.launch.py",
            "fcr_cmd_vel_topic:=/fcr_tron/cmd_vel",
        ],
        name="tron1_robot_hw_launch",
        output="screen",
        additional_env={
            "ROBOT_TYPE": LaunchConfiguration("tron_robot_type"),
            "RL_TYPE": LaunchConfiguration("tron_rl_type"),
        },
        condition=IfCondition(LaunchConfiguration("start_tron_hw")),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "start_tron_hw",
            default_value="false",
            description=(
                "Start official TRON1 robot_hw. Keep false until the robot is "
                "on a stand or in a protected open area."
            ),
        ),
        DeclareLaunchArgument("tron_robot_type", default_value="WF_TRON1A"),
        DeclareLaunchArgument("tron_rl_type", default_value="isaacgym"),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="false keeps /fcr_tron/cmd_vel zero even if upstream commands arrive.",
        ),
        DeclareLaunchArgument(
            "max_linear_x",
            default_value="0.03",
            description="First-real-test forward/reverse speed limit in m/s.",
        ),
        DeclareLaunchArgument(
            "max_angular_z",
            default_value="0.10",
            description="First-real-test yaw speed limit in rad/s.",
        ),
        DeclareLaunchArgument("input_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument(
            "enable_gimbal",
            default_value="true",
            description="Start DJI RS2 gimbal driver on the Jetson side.",
        ),
        DeclareLaunchArgument(
            "rs2_can_interface",
            default_value="can0",
            description="SocketCAN interface for DJI RS2; verify with ip/link stats before real tests.",
        ),
        DeclareLaunchArgument(
            "gimbal_control_mode",
            default_value="incremental_position",
            description="Safer RS2 mode for communication bring-up; use speed only after direction checks.",
        ),
        DeclareLaunchArgument("gimbal_speed_control_byte", default_value="128"),
        DeclareLaunchArgument(
            "start_depth_camera",
            default_value="true",
            description="Start the Orbbec/Gemini depth camera driver through depth_fusion launch.",
        ),
        DeclareLaunchArgument(
            "enable_depth_fusion",
            default_value="true",
            description="Start calibrated depth-fusion node and Gemini/Sony depth TF chain.",
        ),
        DeclareLaunchArgument("depth_camera_serial_number", default_value=""),
        DeclareLaunchArgument("depth_width", default_value="424"),
        DeclareLaunchArgument("depth_height", default_value="240"),
        DeclareLaunchArgument("depth_fps", default_value="10"),
        DeclareLaunchArgument(
            "enable_sony_camera",
            default_value="false",
            description="Keep false for depth-camera-only communication checks.",
        ),
        DeclareLaunchArgument(
            "enable_detection",
            default_value="false",
            description="Keep false while validating hardware communication only.",
        ),
        DeclareLaunchArgument(
            "enable_tracking",
            default_value="false",
            description="Keep false while validating hardware communication only.",
        ),
        DeclareLaunchArgument(
            "enable_mvp",
            default_value="false",
            description="Keep false so no visual-follow autonomy is active during communication bring-up.",
        ),
        DeclareLaunchArgument("use_foxglove", default_value="true"),
        jetson_fcr,
        TimerAction(period=2.0, actions=[tron_hw]),
    ])
