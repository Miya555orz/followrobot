"""Sony UVC camera + FCR perception smoke-test launch.

This launch is intentionally small: it publishes a Sony ZV-E10M2 UVC stream
from /dev/video8 as /sony/image_raw and then starts the existing detection /
tracking / face-aim pipeline. It does not start the gimbal or any chassis node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("perception_pkg")

    image_topic = LaunchConfiguration("image_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")

    sony_uvc_publisher = Node(
        package="perception_pkg",
        executable="video_publisher.py",
        name="sony_uvc_publisher",
        output="screen",
        arguments=[
            "--device",
            LaunchConfiguration("video_device"),
            "--topic",
            image_topic,
            "--camera-info-topic",
            camera_info_topic,
            "--frame-id",
            LaunchConfiguration("frame_id"),
            "--width",
            LaunchConfiguration("width"),
            "--height",
            LaunchConfiguration("height"),
            "--fps",
            LaunchConfiguration("fps"),
            "--camera-info-yaml",
            LaunchConfiguration("camera_info_yaml"),
        ],
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "sony_image_topic": image_topic,
            "enable_detection": LaunchConfiguration("enable_detection"),
            "enable_tracking": LaunchConfiguration("enable_tracking"),
            "enable_face_aim": LaunchConfiguration("enable_face_aim"),
            "device": LaunchConfiguration("inference_device"),
            "model_path": LaunchConfiguration("model_path"),
            "confidence_threshold": LaunchConfiguration("confidence_threshold"),
            "debug_image_topic": LaunchConfiguration("debug_image_topic"),
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("video_device", default_value="/dev/video8"),
            DeclareLaunchArgument("image_topic", default_value="/sony/image_raw"),
            DeclareLaunchArgument("camera_info_topic", default_value="/sony/camera_info"),
            DeclareLaunchArgument("frame_id", default_value="sony_camera_optical_frame"),
            DeclareLaunchArgument("width", default_value="1280"),
            DeclareLaunchArgument("height", default_value="720"),
            DeclareLaunchArgument("fps", default_value="15"),
            DeclareLaunchArgument(
                "camera_info_yaml",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "calibration", "sony_zv_e10_ii.yaml"]
                ),
            ),
            DeclareLaunchArgument("enable_detection", default_value="true"),
            DeclareLaunchArgument("enable_tracking", default_value="true"),
            DeclareLaunchArgument("enable_face_aim", default_value="true"),
            DeclareLaunchArgument("inference_device", default_value="cpu"),
            DeclareLaunchArgument(
                "model_path",
                default_value=PathJoinSubstitution([package_share, "models", "yolov8n.onnx"]),
            ),
            DeclareLaunchArgument("confidence_threshold", default_value="0.25"),
            DeclareLaunchArgument("debug_image_topic", default_value="/perception/debug_image"),
            sony_uvc_publisher,
            perception_launch,
        ]
    )
