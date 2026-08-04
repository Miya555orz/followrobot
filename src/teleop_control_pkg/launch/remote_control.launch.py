"""Start the Jetson-side safety command mux and optional terminal keyboard."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution(
        [FindPackageShare("teleop_control_pkg"), "config", "remote_control.yaml"]
    )
    mux = Node(
        package="teleop_control_pkg",
        executable="command_mux_node",
        name="command_mux",
        output="screen",
        parameters=[config],
    )
    manual_jog = Node(
        package="teleop_control_pkg",
        executable="manual_jog_manager_node",
        name="manual_jog_manager",
        output="screen",
        parameters=[PathJoinSubstitution([
            FindPackageShare("teleop_control_pkg"),
            "config",
            "manual_jog.yaml",
        ])],
        condition=IfCondition(LaunchConfiguration("start_manual_jog")),
    )
    keyboard = Node(
        package="teleop_control_pkg",
        executable="keyboard_platform_teleop",
        name="keyboard_platform_teleop",
        output="screen",
        emulate_tty=True,
        parameters=[config],
        condition=IfCondition(LaunchConfiguration("start_keyboard")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "start_keyboard",
            default_value="false",
            description="Start terminal keyboard; ros2 run in an interactive shell is preferred.",
        ),
        DeclareLaunchArgument(
            "start_manual_jog",
            default_value="true",
            description="Start bounded MANUAL_JOG action manager.",
        ),
        mux,
        manual_jog,
        keyboard,
    ])
