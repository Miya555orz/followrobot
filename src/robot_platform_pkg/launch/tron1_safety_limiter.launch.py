"""Standalone launch for the TRON1 safety speed limiter.

The node converts FCR's TwistStamped command stream to the geometry_msgs/Twist
stream consumed by the TRON1 controller, with speed limits, ramp limits,
timeout stop, and emergency-stop gating.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare("robot_platform_pkg"),
        "config",
        "tron1_safety_limiter.yaml",
    ])

    limiter = Node(
        package="robot_platform_pkg",
        executable="tron1_safety_limiter_node",
        name="tron1_safety_limiter",
        output="screen",
        parameters=[
            config_file,
            {
                "input_topic": LaunchConfiguration("input_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "estop_topic": LaunchConfiguration("estop_topic"),
                "estop_clear_topic": LaunchConfiguration("estop_clear_topic"),
                "motion_authorized_topic": LaunchConfiguration("motion_authorized_topic"),
                "limiter_state_topic": LaunchConfiguration("limiter_state_topic"),
                "enable_motion": ParameterValue(LaunchConfiguration("enable_motion"), value_type=bool),
                "enable_lateral": ParameterValue(LaunchConfiguration("enable_lateral"), value_type=bool),
                "input_timeout_sec": ParameterValue(LaunchConfiguration("input_timeout_sec"), value_type=float),
                "estop_timeout_sec": ParameterValue(
                    LaunchConfiguration("estop_timeout_sec"), value_type=float
                ),
                "motion_authorized_timeout_sec": ParameterValue(
                    LaunchConfiguration("motion_authorized_timeout_sec"), value_type=float
                ),
                "max_linear_x": ParameterValue(LaunchConfiguration("max_linear_x"), value_type=float),
                "max_linear_y": ParameterValue(LaunchConfiguration("max_linear_y"), value_type=float),
                "max_angular_z": ParameterValue(LaunchConfiguration("max_angular_z"), value_type=float),
                "max_accel_x": ParameterValue(LaunchConfiguration("max_accel_x"), value_type=float),
                "max_accel_y": ParameterValue(LaunchConfiguration("max_accel_y"), value_type=float),
                "max_accel_yaw": ParameterValue(LaunchConfiguration("max_accel_yaw"), value_type=float),
                "stop_immediately_on_zero_cmd": ParameterValue(
                    LaunchConfiguration("stop_immediately_on_zero_cmd"), value_type=bool
                ),
                "stop_immediately_on_timeout": ParameterValue(
                    LaunchConfiguration("stop_immediately_on_timeout"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("input_topic", default_value="/fcr/cmd_vel_stamped"),
        DeclareLaunchArgument("output_topic", default_value="/fcr_tron/cmd_vel"),
        DeclareLaunchArgument("estop_topic", default_value="/safety/estop_state"),
        DeclareLaunchArgument("estop_clear_topic", default_value="/tron1/limiter_clear_estop"),
        DeclareLaunchArgument(
            "motion_authorized_topic", default_value="/tron1/motion_authorized"
        ),
        DeclareLaunchArgument("limiter_state_topic", default_value="/tron1/limiter_state"),
        DeclareLaunchArgument(
            "enable_motion",
            default_value="false",
            description="false forces zero output even when input commands arrive.",
        ),
        DeclareLaunchArgument(
            "enable_lateral",
            default_value="false",
            description="false forces linear.y to zero for TRON1 first tests.",
        ),
        DeclareLaunchArgument("input_timeout_sec", default_value="0.25"),
        DeclareLaunchArgument("estop_timeout_sec", default_value="0.50"),
        DeclareLaunchArgument("motion_authorized_timeout_sec", default_value="0.50"),
        DeclareLaunchArgument("max_linear_x", default_value="0.03"),
        DeclareLaunchArgument("max_linear_y", default_value="0.0"),
        DeclareLaunchArgument("max_angular_z", default_value="0.10"),
        DeclareLaunchArgument("max_accel_x", default_value="0.06"),
        DeclareLaunchArgument("max_accel_y", default_value="0.06"),
        DeclareLaunchArgument("max_accel_yaw", default_value="0.20"),
        DeclareLaunchArgument("stop_immediately_on_zero_cmd", default_value="true"),
        DeclareLaunchArgument("stop_immediately_on_timeout", default_value="true"),
        limiter,
    ])
