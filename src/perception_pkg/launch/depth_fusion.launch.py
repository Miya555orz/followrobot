"""Bring up Gemini 335 depth, calibrated static extrinsics, and 3D fusion."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_gemini = LaunchConfiguration("start_gemini")
    calibration_file = LaunchConfiguration("calibration_file")

    arguments = [
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "start_gemini",
            default_value="true",
            description="Start the in-workspace Orbbec Gemini 330-series driver",
        ),
        DeclareLaunchArgument(
            "calibration_file",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("perception_pkg"),
                    "config",
                    "calibration",
                    "sony_gemini_extrinsics.yaml",
                ]
            ),
            description="Schema-v3 Sony-to-Gemini depth calibration",
        ),
        DeclareLaunchArgument(
            "fusion_params",
            default_value=PathJoinSubstitution(
                [FindPackageShare("perception_pkg"), "config", "depth_fusion_params.yaml"]
            ),
        ),
        DeclareLaunchArgument("gemini_serial_number", default_value=""),
        DeclareLaunchArgument("gemini_depth_width", default_value="848"),
        DeclareLaunchArgument("gemini_depth_height", default_value="480"),
        DeclareLaunchArgument("gemini_depth_fps", default_value="10"),
        DeclareLaunchArgument("publish_debug_image", default_value="false"),
    ]

    gemini = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("orbbec_camera"), "launch", "gemini_330_series.launch.py"]
            )
        ),
        launch_arguments={
            "camera_name": "camera",
            "serial_number": LaunchConfiguration("gemini_serial_number"),
            "enable_depth": "true",
            "depth_width": LaunchConfiguration("gemini_depth_width"),
            "depth_height": LaunchConfiguration("gemini_depth_height"),
            "depth_fps": LaunchConfiguration("gemini_depth_fps"),
            "depth_format": "ANY",
            # The schema-v3 calibration publishes a direct depth->Sony TF, so
            # the Gemini color stream is no longer required at runtime.
            "enable_color": "false",
            "enable_left_ir": "false",
            "enable_right_ir": "false",
            "enable_point_cloud": "false",
            "enable_colored_point_cloud": "false",
            "enable_accel": "false",
            "enable_gyro": "false",
            "depth_registration": "false",
            "publish_tf": "true",
        }.items(),
        condition=IfCondition(start_gemini),
    )

    extrinsics = Node(
        package="perception_pkg",
        executable="extrinsics_tf_publisher.py",
        name="sony_gemini_extrinsics_tf_publisher",
        output="screen",
        arguments=["--calibration", calibration_file],
        parameters=[{"use_sim_time": ParameterValue(use_sim_time, value_type=bool)}],
    )

    fusion = Node(
        package="perception_pkg",
        executable="depth_fusion_node",
        name="depth_fusion_node",
        output="screen",
        parameters=[
            LaunchConfiguration("fusion_params"),
            {
                "use_sim_time": ParameterValue(use_sim_time, value_type=bool),
                "debug.publish_image": ParameterValue(
                    LaunchConfiguration("publish_debug_image"), value_type=bool
                ),
            },
        ],
    )

    return LaunchDescription(arguments + [gemini, extrinsics, fusion])
