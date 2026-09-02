"""Minimal real-hardware manual test: can0 gimbal driver plus command mux."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    gimbal = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("robot_platform_pkg"),
            "launch",
            "gimbal_bringup.launch.py",
        ]),
        launch_arguments={
            "use_sim": "false",
            "can_interface": LaunchConfiguration("can_interface"),
            "control_mode": LaunchConfiguration("control_mode"),
            "speed_control_byte": LaunchConfiguration("speed_control_byte"),
        }.items(),
    )
    mux = IncludeLaunchDescription(
        PathJoinSubstitution([
            FindPackageShare("teleop_control_pkg"),
            "launch",
            "remote_control.launch.py",
        ]),
        launch_arguments={"start_keyboard": "false"}.items(),
    )
    return LaunchDescription([
        DeclareLaunchArgument("can_interface", default_value="can0"),
        DeclareLaunchArgument("control_mode", default_value="incremental_position"),
        DeclareLaunchArgument("speed_control_byte", default_value="128"),
        gimbal,
        mux,
    ])
