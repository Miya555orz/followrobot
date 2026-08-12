# bringup_pkg/launch/fcr_bringup.launch.py
"""
FCR 生产环境一键启动文件。

启动顺序（分阶段定时启动，确保依赖就绪）：
  1. t=0s:  机器人平台（底盘/云台/IMU/里程计硬件驱动）
  2. t=2s:  感知管线（检测 + 跟踪）—— 等待相机驱动就绪
  3. t=3s:  伺服控制 —— 等待感知输出 + 平台状态反馈就绪
  4. t=4s:  可视化（RViz2 + 远程感知监控/Foxglove，可选）

用法：
  ros2 launch bringup_pkg fcr_bringup.launch.py use_sim:=false use_rviz:=true
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition


def generate_launch_description():
    # ── 启动配置参数 ──────────────────────────────────────────
    use_sim = LaunchConfiguration("use_sim")
    controller_plugin = LaunchConfiguration("controller_plugin")
    conf_threshold = LaunchConfiguration("confidence_threshold")
    tracker_type = LaunchConfiguration("tracker_type")
    model_path = LaunchConfiguration("model_path")
    detection_device = LaunchConfiguration("detection_device")
    use_mock_detector = LaunchConfiguration("use_mock_detector")
    enable_detection = LaunchConfiguration("enable_detection")

    # 各包的共享目录，用于引用 launch 文件
    pkg_share = FindPackageShare("bringup_pkg")
    perception_share = FindPackageShare("perception_pkg")
    servo_share = FindPackageShare("servo_control_pkg")
    platform_share = FindPackageShare("robot_platform_pkg")
    sony_share = FindPackageShare("sony_camera_pkg")
    remote_monitor_share = FindPackageShare("remote_monitor_pkg")
    teleop_share = FindPackageShare("teleop_control_pkg")
    external_control_share = FindPackageShare("external_control_pkg")

    # Jetson端唯一顶层模式状态源；与外部语音计算机是否上线无关。
    system_mode_manager = Node(
        package="external_control_pkg",
        executable="system_mode_manager_node",
        name="system_mode_manager",
        output="screen",
    )

    # ── 1. 机器人平台（硬件驱动层） ─────────────────────────
    platform_launch = IncludeLaunchDescription(
        PathJoinSubstitution([platform_share, "launch", "platform.launch.py"]),
        launch_arguments={
            "use_sim": use_sim,
            "enable_imu": LaunchConfiguration("enable_imu"),
            "enable_chassis": LaunchConfiguration("enable_chassis"),
            "can_interface": LaunchConfiguration("can_interface"),
            "gimbal_control_mode": LaunchConfiguration("gimbal_control_mode"),
            "gimbal_speed_control_byte": LaunchConfiguration(
                "gimbal_speed_control_byte"
            ),
        }.items(),
    )

    # ── 1b. Sony RGB相机（实机模式）─────────────────────────
    sony_launch = IncludeLaunchDescription(
        PathJoinSubstitution([sony_share, "launch", "sony_camera.launch.py"]),
        launch_arguments={
            "enable_perception": "false",
            "camera_index": LaunchConfiguration("sony_camera_index"),
            "camera_info_url": LaunchConfiguration("sony_camera_info_url"),
            "image_topic": LaunchConfiguration("sony_image_topic"),
        }.items(),
        condition=IfCondition(
            PythonExpression([
                "'", LaunchConfiguration("enable_sony_camera"), "' == 'true' and '",
                use_sim, "' != 'true'",
            ])
        ),
    )

    # 安全仲裁是执行器最终控制话题的唯一发布者。
    remote_control_launch = IncludeLaunchDescription(
        PathJoinSubstitution([teleop_share, "launch", "remote_control.launch.py"]),
        launch_arguments={"start_keyboard": "false"}.items(),
    )

    # Mock和真实检测互斥；跟踪节点在两种模式下都可以运行。
    effective_detection = PythonExpression([
        "'", enable_detection, "' == 'true' and '", use_mock_detector, "' != 'true'"
    ])

    # ── 2. 感知管线 ─────────────────────────────────────────
    perception_launch = IncludeLaunchDescription(
        PathJoinSubstitution([perception_share, "launch", "perception.launch.py"]),
        launch_arguments={
            "model_path": model_path,
            "device": detection_device,
            "confidence_threshold": conf_threshold,
            "tracker_type": tracker_type,
            "enable_camera_motion_compensation": LaunchConfiguration(
                "enable_camera_motion_compensation"
            ),
            "enable_detection": effective_detection,
            "enable_tracking": LaunchConfiguration("enable_tracking"),
            "sony_image_topic": LaunchConfiguration("sony_image_topic"),
        }.items(),
    )

    # The independent Windows computer performs ASR, classification and
    # parameter extraction. Jetson accepts only versioned candidate commands;
    # its dispatcher remains the authoritative state/safety gate.
    voice_control_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [external_control_share, "launch", "voice_control.launch.py"]
        ),
        launch_arguments={
            "start_wake_up_node": "false",
            "start_text_http_bridge": LaunchConfiguration(
                "voice_start_text_http_bridge"
            ),
            "text_http_auth_token": LaunchConfiguration(
                "voice_http_auth_token"
            ),
            "start_intent_classifier": "false",
            "publish_cloud_intents": "false",
            "start_dispatcher": "true",
            "start_command_router": "true",
            "start_chassis_control": LaunchConfiguration("enable_chassis"),
            "start_keyboard_node": "false",
            "min_confidence": LaunchConfiguration("voice_min_confidence"),
            "manual_cmd_gimbal_topic": "/voice/unused_manual_cmd_gimbal",
            "autonomy_cmd_gimbal_topic": "/autonomy/cmd_gimbal",
            "router_output_cmd_topic": "/auto/cmd_gimbal",
            "manual_cmd_vel_topic": "/voice/unused_manual_cmd_vel",
            "autonomy_cmd_vel_topic": "/autonomy/cmd_vel",
            "chassis_output_cmd_topic": "/auto/cmd_vel",
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_voice_control")),
    )

    # Calibrated Gemini depth fusion is opt-in. It owns Gemini startup and the
    # rigid Sony->Gemini static transform, while the RGB perception launch
    # above remains usable on its own.
    depth_fusion_launch = IncludeLaunchDescription(
        PathJoinSubstitution([perception_share, "launch", "depth_fusion.launch.py"]),
        launch_arguments={
            "start_gemini": LaunchConfiguration("start_gemini"),
            "gemini_serial_number": LaunchConfiguration("gemini_serial_number"),
            "calibration_file": LaunchConfiguration("fusion_calibration_file"),
            "publish_debug_image": LaunchConfiguration("fusion_publish_debug_image"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_depth_fusion")),
    )

    # ── 2b. Mock 检测器（仿真模式下的合成检测结果） ──────────
    mock_detector = Node(
        package="simulation_pkg",
        executable="mock_detector.py",
        name="mock_detector",
        output="screen",
        condition=IfCondition(use_mock_detector),
    )

    # ── 3. 伺服控制 ─────────────────────────────────────────
    servo_launch = IncludeLaunchDescription(
        PathJoinSubstitution([servo_share, "launch", "servo_control.launch.py"]),
        launch_arguments={
            "controller_plugin": controller_plugin,
            "allocation_ratio": LaunchConfiguration("servo_allocation_ratio"),
            "auto_start": LaunchConfiguration("servo_auto_start"),
            "target_timeout": LaunchConfiguration("servo_target_timeout"),
            "camera_info_input": LaunchConfiguration("servo_camera_info_topic"),
            "target_input": LaunchConfiguration("servo_target_topic"),
            "aim_target_input": LaunchConfiguration("servo_aim_target_topic"),
            "enable_servo_manager": LaunchConfiguration(
                "enable_servo_manager"),
            "enable_gimbal_visual_servo": LaunchConfiguration(
                "enable_gimbal_visual_servo"),
            "enable_cinematic_motion": LaunchConfiguration(
                "enable_cinematic_motion"),
            "allow_chassis_translation": LaunchConfiguration(
                "servo_allow_chassis_translation"),
            # With voice enabled, its router owns /auto and PBVS publishes to
            # the lower-priority autonomy inputs. Without voice, preserve the
            # original direct PBVS -> command_mux path.
            "cmd_vel_output": PythonExpression([
                "'/autonomy/cmd_vel' if '",
                LaunchConfiguration("enable_voice_control"),
                "' == 'true' else '/auto/cmd_vel'",
            ]),
            "cmd_gimbal_output": PythonExpression([
                "'/autonomy/cmd_gimbal' if '",
                LaunchConfiguration("enable_voice_control"),
                "' == 'true' else '/auto/cmd_gimbal'",
            ]),
        }.items(),
        condition=IfCondition(LaunchConfiguration("enable_servo")),
    )

    # ── 4. RViz2（可选可视化） ──────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_share, "rviz", "fcr_system.rviz"])],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # ── 5. 只读远程感知监控 + Foxglove Bridge ──────────────
    remote_monitor_launch = IncludeLaunchDescription(
        PathJoinSubstitution(
            [remote_monitor_share, "launch", "remote_monitor.launch.py"]
        ),
        launch_arguments={
            "use_sim_time": use_sim,
            "enable_visualizer": "true",
            "enable_foxglove": "true",
            "foxglove_address": LaunchConfiguration("foxglove_address"),
            "foxglove_port": LaunchConfiguration("foxglove_port"),
            "image_topic": LaunchConfiguration("sony_image_topic"),
            "model_path": model_path,
            "inference_backend": detection_device,
            "yolo_model": LaunchConfiguration("yolo_model_name"),
            "aim_target_topic": "/perception/aim_target_2d",
            "servo_state_topic": "/servo/state",
            "enable_future_inputs": LaunchConfiguration(
                "enable_monitor_future_inputs"
            ),
            "remote_publish_rate_hz": LaunchConfiguration(
                "remote_publish_rate_hz"
            ),
            "remote_max_width": LaunchConfiguration("remote_max_width"),
            "jpeg_quality": LaunchConfiguration("remote_jpeg_quality"),
            "max_frame_age_ms": LaunchConfiguration("remote_max_frame_age_ms"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_foxglove")),
    )

    # 分阶段启动：平台 → 感知 → 控制 → 可视化
    # 使用 TimerAction 确保前置节点已就绪
    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="false",
                              description="是否启用仿真模式"),
        DeclareLaunchArgument(
            "enable_imu", default_value="false",
            description="真实BNO055后端完成前保持false；仿真可显式设为true"),
        DeclareLaunchArgument(
            "enable_chassis", default_value="true",
            description="是否启动底盘驱动和里程计；纯云台测试应设为false"),
        DeclareLaunchArgument(
            "can_interface", default_value="can0",
            description="DJI RS2云台使用的Linux SocketCAN接口；USB-CAN实机可设为can1"),
        DeclareLaunchArgument(
            "gimbal_control_mode", default_value="incremental_position",
            description="云台命令模式：speed 或 incremental_position"),
        DeclareLaunchArgument(
            "gimbal_speed_control_byte", default_value="128",
            description="DJI RS2速度控制字，128代表0x80"),
        DeclareLaunchArgument("controller_plugin",
                              default_value="servo_control_pkg::IBVSController",
                              description="视觉伺服控制器插件类名"),
        DeclareLaunchArgument("confidence_threshold", default_value="0.10",
                              description="YOLO发布阈值；ByteTrack需要保留低分候选"),
        DeclareLaunchArgument(
            "tracker_type", default_value="bytetrack",
            description="二维跟踪器：bytetrack 或 legacy_iou",
        ),
        DeclareLaunchArgument(
            "enable_camera_motion_compensation", default_value="true",
            description="是否用Sony图像光流补偿相机/云台运动",
        ),
        DeclareLaunchArgument(
            "model_path",
            default_value=PathJoinSubstitution(
                [perception_share, "models", "yolov8n.onnx"]
            ),
            description="YOLO ONNX 模型路径",
        ),
        DeclareLaunchArgument(
            "detection_device",
            default_value="cpu",
            description="检测推理设备：cpu、cuda_fp16、cuda_fp32 或 tensorrt",
        ),
        DeclareLaunchArgument("enable_detection", default_value="true",
                              description="是否启动真实 YOLO 检测节点"),
        DeclareLaunchArgument("enable_tracking", default_value="true",
                              description="是否启动目标跟踪节点"),
        DeclareLaunchArgument(
            "enable_depth_fusion", default_value="false",
            description="启用Gemini 335、刚性外参TF和/perception/targets_3d",
        ),
        DeclareLaunchArgument(
            "start_gemini", default_value="true",
            description="融合启动文件是否同时启动Gemini 335驱动",
        ),
        DeclareLaunchArgument(
            "gemini_serial_number", default_value="",
            description="可选的Gemini设备序列号",
        ),
        DeclareLaunchArgument(
            "fusion_calibration_file",
            default_value=PathJoinSubstitution(
                [
                    perception_share,
                    "config",
                    "calibration",
                    "sony_gemini_extrinsics.yaml",
                ]
            ),
            description="包含Gemini原厂depth-to-color外参的schema-v3刚性标定文件",
        ),
        DeclareLaunchArgument(
            "fusion_publish_debug_image", default_value="false",
            description="发布深度点重投影调试图/perception/targets_3d_debug",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false",
                              description="是否启动 RViz2"),
        DeclareLaunchArgument("use_foxglove", default_value="false",
                              description="是否启动远程感知监控与 Foxglove WebSocket 桥接"),
        DeclareLaunchArgument("foxglove_address", default_value="0.0.0.0",
                              description="Foxglove Bridge 监听地址"),
        DeclareLaunchArgument("foxglove_port", default_value="8765",
                              description="Foxglove Bridge WebSocket 端口"),
        DeclareLaunchArgument("yolo_model_name", default_value="yolov8n",
                              description="远程状态面板显示的模型名称"),
        DeclareLaunchArgument(
            "enable_monitor_future_inputs", default_value="false",
            description="预留观察 target_3d/cmd_vel/gimbal_state，不发布控制指令",
        ),
        DeclareLaunchArgument(
            "remote_publish_rate_hz", default_value="10.0",
            description="仅限制Foxglove标注图帧率，不影响检测和跟踪",
        ),
        DeclareLaunchArgument(
            "remote_max_width", default_value="960",
            description="Foxglove标注图最大宽度；0表示保留原始宽度",
        ),
        DeclareLaunchArgument(
            "remote_jpeg_quality", default_value="65",
            description="Foxglove标注图JPEG质量（20-95）",
        ),
        DeclareLaunchArgument(
            "remote_max_frame_age_ms", default_value="300",
            description="标注图超过此源时间戳年龄后直接丢弃，防止延迟积累",
        ),
        DeclareLaunchArgument("use_mock_detector", default_value="false",
                              description="是否使用合成检测器（绕过 YOLO）"),
        DeclareLaunchArgument("enable_sony_camera", default_value="true",
                              description="实机模式下是否启动Sony相机"),
        DeclareLaunchArgument("sony_image_topic", default_value="/sony/image_raw",
                              description="2D检测使用的RGB图像话题"),
        DeclareLaunchArgument("sony_camera_index", default_value="1",
                              description="CRSDK枚举得到的一基相机序号"),
        DeclareLaunchArgument(
            "sony_camera_info_url", default_value="",
            description="Sony精确分辨率标定文件，例如file:///home/nvidia/.ros/camera_info/sony.yaml",
        ),
        DeclareLaunchArgument(
            "enable_servo", default_value="false",
            description="是否启动视觉伺服；默认关闭，由操作者显式授权",
        ),
        DeclareLaunchArgument(
            "servo_auto_start", default_value="false",
            description="收到有效3D目标后是否自动进入伺服闭环；生产环境默认关闭",
        ),
        DeclareLaunchArgument(
            "servo_target_timeout", default_value="0.45",
            description="3D目标停止更新后伺服发布零速度的超时（秒）",
        ),
        DeclareLaunchArgument(
            "servo_camera_info_topic", default_value="/sony/camera_info",
            description="伺服控制使用的CameraInfo话题",
        ),
        DeclareLaunchArgument(
            "servo_target_topic", default_value="/perception/tracks",
            description="伺服使用的TargetArray；第二阶段直接使用2D跟踪结果",
        ),
        DeclareLaunchArgument(
            "servo_aim_target_topic", default_value="/perception/aim_target_2d",
            description="独立Sony二维云台快环使用的高频瞄准目标",
        ),
        DeclareLaunchArgument(
            "enable_servo_manager", default_value="true",
            description="启用PBVS/IBVS统一伺服管理器；纯二维云台模式应关闭",
        ),
        DeclareLaunchArgument(
            "enable_gimbal_visual_servo", default_value="true",
            description="启用独立二维云台快环；不依赖Gemini、3D融合或PBVS分配器",
        ),
        DeclareLaunchArgument(
            "enable_cinematic_motion", default_value="false",
            description="启用四种可取消运镜任务；需要PBVS、3D融合和底盘",
        ),
        DeclareLaunchArgument(
            "servo_allow_chassis_translation", default_value="false",
            description="无可靠深度前保持false，仅允许云台和底盘转向",
        ),
        DeclareLaunchArgument(
            "servo_allocation_ratio", default_value="0.5",
            description="偏航分配比例：0为纯云台，1为纯底盘",
        ),
        DeclareLaunchArgument(
            "enable_voice_control", default_value="false",
            description="启动Jetson候选意图仲裁及执行桥接；不启动本地语音模型",
        ),
        DeclareLaunchArgument(
            "voice_min_confidence", default_value="0.60",
            description="Jetson接受独立计算机候选意图的最低置信度",
        ),
        DeclareLaunchArgument(
            "voice_start_text_http_bridge", default_value="true",
            description="启动Windows语音代理使用的结构化候选HTTP入口",
        ),
        DeclareLaunchArgument(
            "voice_http_auth_token",
            default_value=EnvironmentVariable(
                "FCR_VOICE_AUTH_TOKEN", default_value=""
            ),
            description="Windows到Jetson语音HTTP网关的Bearer共享令牌",
        ),

        # 阶段 1：平台驱动 (t=0s)
        platform_launch,
        remote_control_launch,
        system_mode_manager,
        voice_control_launch,
        sony_launch,
        mock_detector,

        # 阶段 2：感知管线 (t=2s，等待相机驱动就绪)
        TimerAction(period=2.0, actions=[perception_launch]),
        TimerAction(period=2.0, actions=[depth_fusion_launch]),

        # 阶段 3：伺服控制 (t=3s，等待感知输出 + 平台状态)
        TimerAction(period=3.0, actions=[servo_launch]),

        # 可视化 (t=4s)
        TimerAction(period=4.0, actions=[rviz_node, remote_monitor_launch]),
    ])
