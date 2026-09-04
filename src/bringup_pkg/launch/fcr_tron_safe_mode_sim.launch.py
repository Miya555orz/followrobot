"""TRON1 安全模式管理 + safety limiter 仿真/topic 验证启动。

此 launch 不启动真实 TRON1，不启动官方 robot_hw，不发布速度命令。
它只启动：
  1. tron1_mode_manager_node：状态机和运动授权；
  2. tron1_safety_limiter_node：限速、急停、timeout、授权门控。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    limiter_config = PathJoinSubstitution(
        [
            FindPackageShare("robot_platform_pkg"),
            "config",
            "tron1_safety_limiter.yaml",
        ]
    )

    mode_manager = Node(
        package="robot_platform_pkg",
        executable="tron1_mode_manager_node",
        name="tron1_mode_manager",
        output="screen",
        parameters=[
            {
                "allow_walk_motion": False,
                "allow_tron_follow_motion": ParameterValue(
                    LaunchConfiguration("allow_tron_follow_motion"), value_type=bool
                ),
            }
        ],
    )

    limiter = Node(
        package="robot_platform_pkg",
        executable="tron1_safety_limiter_node",
        name="tron1_safety_limiter",
        output="screen",
        parameters=[
            limiter_config,
            {
                "enable_motion": ParameterValue(
                    LaunchConfiguration("enable_motion"), value_type=bool
                ),
                "max_linear_x": ParameterValue(
                    LaunchConfiguration("max_linear_x"), value_type=float
                ),
                "max_linear_y": 0.0,
                "max_angular_z": ParameterValue(
                    LaunchConfiguration("max_angular_z"), value_type=float
                ),
                "enable_lateral": False,
                "input_timeout_sec": ParameterValue(
                    LaunchConfiguration("input_timeout_sec"), value_type=float
                ),
                "motion_authorized_timeout_sec": ParameterValue(
                    LaunchConfiguration("motion_authorized_timeout_sec"), value_type=float
                ),
            },
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "enable_motion",
                default_value="false",
                description="默认 false；只有验收脚本会显式传 true，禁止人工误用为真机启动入口。",
            ),
            DeclareLaunchArgument("allow_tron_follow_motion", default_value="false"),
            DeclareLaunchArgument("max_linear_x", default_value="0.03"),
            DeclareLaunchArgument("max_angular_z", default_value="0.10"),
            DeclareLaunchArgument("input_timeout_sec", default_value="0.25"),
            DeclareLaunchArgument("motion_authorized_timeout_sec", default_value="0.50"),
            mode_manager,
            limiter,
        ]
    )
