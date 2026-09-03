"""TRON1 安全模式管理节点。

该节点不直接发布速度，只发布 `/tron1/motion_authorized` 授权信号。
真实速度仍必须经过 `tron1_safety_limiter_node`。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mode_manager = Node(
        package="robot_platform_pkg",
        executable="tron1_mode_manager_node",
        name="tron1_mode_manager",
        output="screen",
        parameters=[
            {
                "request_topic": LaunchConfiguration("request_topic"),
                "state_topic": LaunchConfiguration("state_topic"),
                "motion_authorized_topic": LaunchConfiguration(
                    "motion_authorized_topic"
                ),
                "estop_topic": LaunchConfiguration("estop_topic"),
                "allow_walk_motion": ParameterValue(
                    LaunchConfiguration("allow_walk_motion"), value_type=bool
                ),
                "allow_tron_follow_motion": ParameterValue(
                    LaunchConfiguration("allow_tron_follow_motion"), value_type=bool
                ),
            }
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("request_topic", default_value="/tron1/mode_request"),
            DeclareLaunchArgument("state_topic", default_value="/tron1/mode_state"),
            DeclareLaunchArgument(
                "motion_authorized_topic", default_value="/tron1/motion_authorized"
            ),
            DeclareLaunchArgument("estop_topic", default_value="/safety/estop_state"),
            DeclareLaunchArgument(
                "allow_walk_motion",
                default_value="false",
                description="默认 false：行走准备态不授权真实底盘运动。",
            ),
            DeclareLaunchArgument(
                "allow_tron_follow_motion",
                default_value="true",
                description="仅进入 TRON_FOLLOW 且无急停时，才允许 limiter 放行。",
            ),
            mode_manager,
        ]
    )
