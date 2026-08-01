# 视觉感知与跟随链路故障复盘

> 适用平台：Jetson Orin Nano 8GB、Ubuntu 22.04、ROS 2 Humble  
> 主要设备：Sony ZV-E10 II、Orbbec Gemini 335、DJI RS2、移动底盘  
> 当前冻结基线：`v3.3.27`（提交 `72c918d`）及其后的文档/忽略规则提交

## 1. 文档目的

本文记录视觉模块从单相机检测发展到双相机三维跟随过程中实际出现过的主要
问题、根因、处理方法和防止复发的检查项。它不是单次测试日志，而是后续开发、
换机部署和现场排障的工程基线。

PBVS当前实现的结构、控制方程、状态机和云台—底盘分配详见
[`pbvs_servo_control_technical_design.md`](pbvs_servo_control_technical_design.md)。

排障时必须按链路分层，不要看到“机器人不动”就直接调整控制增益：

```text
设备与总线
  -> 相机驱动与时间戳
  -> 图像与 CameraInfo
  -> YOLO 检测
  -> ByteTrack 跟踪
  -> 双相机 TF 与深度融合
  -> PBVS/IBVS 伺服
  -> command_mux 仲裁
  -> 底盘和云台驱动
  -> Foxglove 观察层
```

## 2. 当前已验证链路

```text
Sony ZV-E10 II
  -> /sony/image_raw + /sony/camera_info
  -> YOLOv8n TensorRT
  -> /perception/detections
  -> ByteTrack（XYAH Kalman + Hungarian + 状态门控）
  -> /perception/tracks

Orbbec Gemini 335
  -> /camera/depth/image_raw + /camera/depth/camera_info
  -> depth_fusion_node
  -> /perception/targets_3d

/perception/targets_3d + /perception/aim_target_2d
  -> servo_manager（PBVS）
  -> /auto/cmd_vel + /auto/cmd_gimbal
  -> command_mux
  -> /cmd_vel + /cmd_gimbal
  -> 底盘 + DJI RS2

观察层：
/perception/tracking_image/compressed + /perception/monitor_status
  -> foxglove_bridge
  -> Foxglove Studio
```

## 3. 当前冻结参数

以下参数是当前实机“底盘明显更顺滑”时使用的默认值。除非有新的同场景
rosbag 对照数据，不要仅凭主观感受同时修改多项参数。

### 3.1 PBVS

文件：`src/servo_control_pkg/config/pbvs_params.yaml`

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `desired_depth` | `2.0 m` | 期望跟随距离 |
| `max_linear_velocity` | `0.08 m/s` | 前后速度上限 |
| `linear_acceleration_limit` | `0.20 m/s²` | PBVS线速度斜坡 |
| `depth_filter_alpha` | `0.20` | 深度误差低通 |
| `depth_deadband_m` | `0.20 m` | 距离死区 |
| `pbvs_max_source_age` | `0.45 s` | 3D目标源时间戳最大年龄 |
| `pbvs_translation_hold_timeout` | `0.35 s` | 深度帧间保持最近可信Z |
| `pbvs_target_switch_confirmation` | `0.60 s` | 新ID切换确认时间 |

总启动文件中的 `servo_target_timeout` 同步冻结为 `0.45 s`。

### 3.2 控制仲裁

文件：`src/teleop_control_pkg/config/remote_control.yaml`

| 参数 | 当前值 | 作用 |
| --- | ---: | --- |
| `publish_rate_hz` | `50 Hz` | 最终指令发布频率 |
| `command_timeout_ms` | `150 ms` | 手动指令租约 |
| `auto_command_timeout_ms` | `350 ms` | 自动指令独立租约 |
| `max_linear_x/y` | `0.08 m/s` | 仲裁层线速度上限 |
| `max_angular_z` | `0.25 rad/s` | 仲裁层转速上限 |
| `max_accel_x/y` | `0.20 m/s²` | 非零目标加速斜坡 |
| `max_accel_yaw` | `0.50 rad/s²` | 偏航加速斜坡 |
| `max_auto_decel_x/y` | `0.60 m/s²` | 自动零速的受限减速 |
| `max_auto_decel_yaw` | `1.20 rad/s²` | 自动偏航零速的受限减速 |

安全语义保持不变：

- 自动控制器短暂发布零速：受限减速，避免反复刹停和起步；
- 手动松键、软件急停、模式安全停、指令真正超时：立即输出零速；
- `command_mux` 必须是 `/cmd_vel` 和 `/cmd_gimbal` 的唯一最终发布者。

### 3.3 DJI RS2

文件：`src/robot_platform_pkg/config/gimbal_params.yaml`

- 驱动层最大偏航/俯仰速度：`1.0 rad/s`；
- 速度指令发送上限：`20 Hz`；
- 增量位置目标发送周期：`0.1 s`；
- 增量位置单步安全上限：`5°`；
- 指令看门狗：`0.5 s`。

一键启动脚本负责识别 `gs_usb` 对应的真实 USB-CAN 接口。不能因为接口名字
恰好是 `can1` 就使用板载 `mttcan`；判断依据必须是驱动归属和收发反馈。

## 4. 故障与对策

### 4.1 Windows虚拟环境激活命令错误

**现象**

PowerShell执行 `source yolo_env/bin/activate`，提示无法识别 `source`。

**根因**

`source` 是 Bash 命令，Windows PowerShell 的虚拟环境目录和激活脚本不同。

**对策**

```powershell
.\yolo_env\Scripts\Activate.ps1
```

中途中止模型导出通常不需要清理整个环境，只需删除未完成的目标文件后重新导出。

### 4.2 YOLO模型与推理后端混淆

**现象**

- 不清楚 `yolov8n.pt`、ONNX和TensorRT engine之间的关系；
- Python推理成功，但不代表ROS 2 C++后端一定正确。

**根因**

三者承担不同角色：

```text
yolov8n.pt（训练好权重）
  -> 导出 yolov8n.onnx（跨平台计算图）
  -> Jetson本机生成 yolov8n_fp16.engine（设备相关TensorRT引擎）
```

Python Ultralytics会代为完成预处理、输出解析和NMS，而C++后端必须自行保证
letterbox、输出维度、坐标恢复和NMS一致。

**对策**

- `.pt` 和 `.onnx` 不作为普通源码直接提交；
- 模型通过GitHub Release或部署步骤获取；
- TensorRT engine只能在目标Jetson/对应TensorRT版本上生成；
- Python摄像头测试只证明模型可用，C++ ROS节点仍需单独验收。

### 4.3 Sony SDK架构和安装目录错误

**现象**

构建时找不到：

```text
CameraRemote_SDK.h
libCr_Core.so
```

**根因**

- Jetson是 `aarch64`，应使用 `Linux 64bit (ARMv8)` SDK；
- `.wxdownload` 是浏览器未完成下载的临时文件；
- SDK未复制到包约定的 `sdk/include` 和 `sdk/lib`。

**对策**

- 只部署SDK运行所需头文件、核心库和适配器库；
- 保持源码期望目录；
- Sony SDK有独立许可，不提交GitHub；
- 构建前用 `test -f` 验证头文件和库存在。

### 4.4 rosdep键无法解析

**现象**

```text
Cannot locate rosdep definition for [ament_python]
Cannot locate rosdep definition for [nlohmann-json3-dev]
```

**根因**

- `ament_python` 被错误地当作系统 rosdep 依赖；
- Debian包名不一定能直接作为有效rosdep键。

**对策**

- Python包在 `package.xml` 中使用正确的构建类型声明；
- JSON依赖通过系统包或正确rosdep键安装；
- `rosdep -r` 显示“可解析依赖已安装”不代表未解析键可以永久忽略。

### 4.5 colcon符号链接残留

**现象**

```text
failed to create symbolic link ... existing path cannot be removed: Is a directory
```

**根因**

曾使用不同构建方式，`build/install` 中旧目录与 `--symlink-install` 的目标冲突。

**对策**

只清理报错包对应的 `build/<pkg>` 和 `install/<pkg>` 后重编，不要无理由删除整个
工作区，也不要删除源码。

### 4.6 CMake链接签名混用

**现象**

`target_link_libraries` 报错：plain signature和keyword signature混用。

**根因**

Humble中的 `ament_target_dependencies()` 内部使用了plain形式，同一个target又
使用了 `PRIVATE/PUBLIC` 形式。

**对策**

同一target统一链接风格；修改后只编译受影响包及上游依赖。

### 4.7 缺少diagnostic_updater头文件

**现象**

```text
diagnostic_updater/diagnostic_updater.hpp: No such file or directory
```

**根因**

系统依赖未安装，或者 `package.xml/CMakeLists.txt` 未完整声明。

**对策**

安装Humble对应包，并同时补齐 `find_package`、target依赖和 `package.xml` 声明。

### 4.8 OpenCV 4.5与4.8 ABI混装

**现象**

`ldd` 同时出现：

```text
libopencv_*.so.4.5d
libopencv_*.so.408
```

并产生链接冲突警告、image_transport插件不稳定等问题。

**根因**

系统 `cv_bridge` 按OpenCV 4.5构建，而工程节点按本地OpenCV 4.8链接。

**对策**

- 确定全工作区唯一OpenCV ABI；
- 从源码重建 `vision_opencv/cv_bridge` 以匹配OpenCV 4.8；
- 用 `ldd` 检查关键节点，只允许同一系列OpenCV库；
- 不把 `OpenCV_DIR` 盲目传给不使用OpenCV的包。

### 4.9 Sony内参文件缺失

**现象**

```text
Unable to open ... ~/.ros/camera_info/sony_zv_e10_ii.yaml
```

**根因**

驱动启动时指定了默认URL，但对应文件尚未生成。

**对策**

- 使用固定焦段、固定分辨率重新标定；
- 当前Sony标定分辨率为 `1024x680`；
- 把成功参数保存到仓库的标定目录，并部署到运行时路径；
- 改变焦段、对焦方式或分辨率后重新验证内参。

### 4.10 Sony LiveView错误和连接超时

**现象**

```text
Connection timeout
LiveView error: 0x00008000
No cameras found
CRSDK reported 640x428 but JPEG decoded as 1024x680
```

**根因**

可能包括相机USB模式、其他进程占用、相机状态未就绪、USB链路不稳定，以及
CRSDK元数据尺寸与实际JPEG尺寸不一致。

**对策**

- 确认相机USB远程控制模式和唯一客户端；
- 先验证CRSDK连接，再启动完整感知；
- 以实际JPEG解码尺寸为发布图像尺寸；
- 错误日志限频，并保留重连退避，避免刷屏掩盖根因。

### 4.11 Foxglove压缩图像话题不存在

**现象**

查询 `/perception/tracking_image/compressed` 无发布，但列表中出现
`/tracking_image/compressed`。

**根因**

image_transport的基话题命名/命名空间配置不一致，或插件受OpenCV ABI混装影响。

**对策**

- 明确基话题是 `/perception/tracking_image`；
- 检查 `image_transport list_transports`；
- 用 `ros2 topic list` 以实际名称为准；
- 当前观察话题固定为 `/perception/tracking_image/compressed`。

### 4.12 Foxglove延迟不断积累

**现象**

画面帧率看似正常，但延迟逐渐增加到十几秒。

**根因**

原始图像/高质量JPEG产生的带宽和编码负载超过无线链路可持续吞吐，Foxglove
继续排队显示历史帧。

**对策**

- 远程只传压缩标注图；
- 限制远程发布帧率、宽度和JPEG质量；
- 使用深度为1、BEST_EFFORT、VOLATILE的最新帧语义；
- 对过期帧设置最大年龄并主动丢弃；
- 同时观察 `topic hz` 和 `topic bw`，不能只看FPS。

### 4.13 2D跟踪ID容易改变

**现象**

- 人体运动、短时遮挡或bbox面积突变时分配新ID；
- 全身突然只露头部时尤其明显。

**根因**

单纯IoU难以处理快速位移和尺度突变；没有ReID时，长时遮挡后的身份恢复能力有
物理上限。

**已实施对策**

- ByteTrack式高低置信度两阶段关联；
- XYAH Kalman预测；
- Mahalanobis门控；
- IoU、中心距离、尺度变化复合代价；
- LOST专用关联门限和延迟创建新ID；
- 类别一致性和相机运动补偿；
- 当前锁定ID允许有限时间预测。

**已知边界**

大面积遮挡、长时间离开画面、多人外观相似交叉仍可能换ID。若需要跨长遮挡保持
身份，应增加行人ReID，但在引入前必须评估Jetson算力、时延和隐私约束。

### 4.14 Orbbec只能以USB 2.0 HighSpeed连接

**现象**

USB查看工具显示：

```text
Device maximum Speed: SuperSpeed
Device Connection Speed: High-Speed (480 Mbit/s)
```

**根因**

设备支持SuperSpeed不代表当前线缆、转接、Hub和物理端口的SuperSpeed通道已经
连通。快充线也不等于USB 3数据线。

**对策**

- 使用 `lsusb -t` 或USB Device Tree Viewer看实际连接速度；
- 避免把两路高带宽相机放在同一个USB 2 Hub；
- USB 2阶段可降分辨率/帧率完成标定和功能验证；
- 以后切到USB 3，只要相机安装、分辨率、焦段和内参模型不变，外参通常无需因
  总线速度本身重标；若运行分辨率改变则必须重新验证参数适配。

### 4.15 双相机标定RMSE过高

**现象**

多次出现：

```text
RMSE 1.830px / 2.734px / 1.290px exceeds 1.000px
```

或图像对时间差超过门限。

**根因**

- 棋盘格姿态分布不足或重复；
- 模糊、反光、棋盘占比太小；
- 两相机采样不同步；
- Sony焦段或对焦状态改变；
- 少量错误图像对污染整体外参。

**对策**

- 使用 `8x6` 内角点、`25 mm` 方格；
- 中央、四边、四角都要覆盖近/中/远和不同倾角；
- 采集时先保持棋盘静止，再触发图像对；
- 保存原始样本，允许离线剔除刚性位姿和极线离群点；
- 最终成功结果：21/28对，RMSE约 `0.6636 px`；
- 标定成功后同时保存Sony、Gemini refined内参和外参。

### 4.16 TF出现两个不连通的树

**现象**

```text
Could not find a connection ... Tf has two or more unconnected trees
```

导致深度融合诊断为 `WAITING_TF`。

**根因**

只发布了Sony到Gemini color的外参，但Gemini color到depth的设备内参外参没有进入
同一TF树，或父子frame方向配置错误。

**对策**

- 发布静态外参：Sony optical frame与Gemini color optical frame；
- 从 `/camera/depth_to_color` 获取Gemini设备深度到彩色外参；
- 确认 `camera_color_optical_frame` 与 `camera_depth_optical_frame` 可互查；
- 最终确认Sony到Gemini depth整条TF连通后再启动融合。

### 4.17 `/perception/targets_3d`没有输出

**现象**

节点存在，但话题无消息；诊断显示 `WAITING_TF` 或 `TIME_UNSYNCED`。

**根因**

节点“已经启动”不代表输入条件满足。融合需要同时具备：

- `/perception/tracks`；
- depth image与depth CameraInfo；
- Sony CameraInfo；
- 完整TF；
- 时间差不超过门限。

**对策**

按输入、TF、时间同步、输出的顺序逐项检查，不要重复启动多个
`depth_fusion_node`。融合成功后Foxglove会显示 `X/Y/Z/D/C`；目标离开后在有限
保持时间结束显示 `Target3D unavailable` 属于正确行为。

### 4.18 PBVS云台方向错误和俯仰抖动

**现象**

- 首次运行直接抬头；
- 左右方向相反；
- 俯仰断续、中心附近抖动。

**根因**

相机光学坐标、云台协议正方向、图像像素方向没有统一；增量位置协议的最低动作
周期与50Hz控制频率不匹配。

**对策**

- 分别做单轴、小速度、悬空测试确认符号；
- 统一Sony optical frame到云台yaw/pitch的映射；
- pitch使用独立增益、低通、死区和加速度限制；
- RS2增量位置目标最多按10Hz发送，不能用50Hz覆盖前一动作。

### 4.19 CAN出现 `No buffer space available`

**现象**

运行一段时间后持续出现：

```text
CAN 发送失败: No buffer space available
```

云台自动跟随和自带摇杆均可能失去响应。

**根因**

- DJI协议一条指令可能拆成多帧；
- 50Hz持续发送使USB-CAN发送队列耗尽；
- 错选板载 `mttcan`；
- USB-CAN掉线后接口名称/设备节点变化；
- CAN物理链路无正常应答。

**对策**

- 云台驱动只发送最新速度并限频到20Hz；
- 增大SocketCAN `txqueuelen`，启用 `restart-ms`；
- 一键脚本按 `gs_usb` 驱动归属识别USB-CAN；
- 无正确USB-CAN时禁止回退到 `mttcan`；
- 启动后必须检查 `/gimbal/status.connected`、`rx_count` 和 `can_error_count`，
  不能只看接口为UP；
- 接口出错时停止旧节点、重载 `gs_usb`、重新配置后只启动一套系统。

### 4.20 同名节点重复

**现象**

`ros2 node list` 显示两个 `/servo_manager` 或 `/depth_fusion_node`。

**根因**

可能是重复启动真实进程，也可能是ROS daemon/DDS发现缓存。

**对策**

同时检查：

```bash
pgrep -af servo_manager_node
ros2 topic info /auto/cmd_vel -v
```

只有一个进程且只有一个发布者时属于图缓存；重启daemon或等待租约过期即可。
若有两个真实进程，停止所有旧bringup后只启动一次。最终控制话题必须只有一个
上游发布者。

### 4.21 手动遥控可用、自动跟随无反应

**现象**

底盘手动遥控正常，但自动模式云台和底盘输出为零。

**根因候选**

- `command_mux` 仍处于manual；
- `servo_manager`重复；
- `/perception/aim_target_2d` 无效；
- `/perception/targets_3d` 虽有消息但超时/质量门控失败；
- 平台状态中云台未连接，PBVS安全抑制全部运动；
- CAN接口存在但无RX。

**对策**

沿以下证据链检查：

```text
/perception/targets_3d
  -> /servo/state
  -> /auto/cmd_vel + /auto/cmd_gimbal
  -> /remote_control/status (mode=auto, active_source=auto)
  -> /cmd_vel + /cmd_gimbal
  -> /platform/state + /gimbal/status
```

不能通过直接向最终 `/cmd_vel` 增加第二个发布者绕过仲裁。

### 4.22 底盘前后运动顿挫

**现象**

底盘可跟随，但前进/后退表现为短促运动和停车反复交替。

**关键证据**

录包 `chassis_stutter_20260730_162612` 共约111秒：

- `/auto/cmd_vel`：约50.1Hz；
- `/auto/cmd_vel.linear.x` 有348次“非零到零”跳变；
- 342次跳变恰好对应 `/servo/state = LOST`；
- `/cmd_vel` 基本复现 `/auto/cmd_vel`，说明问题来自伺服上游，不是底盘串口；
- `/perception/targets_3d` P99到达间隔约402ms、最大约1.70s；
- 原 `target_timeout` 和 `pbvs_max_source_age` 均为250ms，门限过紧；
- 平台连接、控制模式、急停和深度质量不是主要触发因素。

**修复**

1. `target_timeout` 和 `pbvs_max_source_age` 调整为450ms；
2. 自动指令独立租约调整为350ms；
3. 自动零速不再瞬时砍为零，改为有界快速减速；
4. 保留手动松键、急停和真正超时的立即停车语义。

**结果**

实机反馈底盘明显更顺滑，当前参数作为冻结基线。

## 5. 标准排障顺序

### 5.1 节点唯一性

```bash
ros2 node list 2>/dev/null | sort | uniq -c | awk '$1 > 1'
pgrep -af "fcr_bringup|servo_manager_node|depth_fusion_node"
```

### 5.2 感知输入

```bash
timeout 8 ros2 topic hz /sony/image_raw
timeout 8 ros2 topic hz /perception/detections
timeout 8 ros2 topic hz /perception/tracks
timeout 8 ros2 topic hz /perception/targets_3d
```

### 5.3 TF与融合

```bash
timeout 6 ros2 run tf2_ros tf2_echo \
  sony_camera_optical_frame camera_depth_optical_frame

timeout 8 ros2 topic echo /diagnostics |
grep -A 20 -B 5 depth_fusion
```

### 5.4 伺服与仲裁

```bash
ros2 topic echo /servo/state --once
ros2 topic echo /auto/cmd_vel --once
ros2 topic echo /remote_control/status --once
ros2 topic echo /cmd_vel --once
```

### 5.5 驱动状态

```bash
ros2 topic echo /platform/state --once
ros2 topic echo /gimbal/status --once
ip -details -statistics link show can0
```

接口名不固定时，以启动脚本最终选择的接口为准。

### 5.6 Foxglove

```bash
timeout 8 ros2 topic hz /perception/tracking_image/compressed
timeout 8 ros2 topic bw /perception/tracking_image/compressed
```

## 6. rosbag最小采集集合

控制问题不录原始图像，避免录包本身改变系统负载。建议至少包含：

```text
/perception/tracks
/perception/targets_3d
/perception/aim_target_2d
/servo/state
/auto/cmd_vel
/auto/cmd_gimbal
/remote_control/status
/cmd_vel
/cmd_gimbal
/platform/state
/gimbal/status
/chassis/odom_raw
/odom
/diagnostics
/tf
/tf_static
```

录包后同时保存运行参数：

```bash
ros2 param dump /servo_manager
ros2 param dump /command_mux
ros2 param dump /chassis_driver
```

诊断包目录统一放在 `diagnostics/`，该目录已加入 `.gitignore`，不得提交大型
`.db3/.mcap/.tar.gz` 到源码仓库。

## 7. 仍需接受的系统边界

- 没有ReID时，长时间离屏和大面积遮挡不能保证恢复原ID；
- USB 2链路可以完成降级验证，但不适合作为最终双相机高帧率配置；
- `Target3D unavailable` 在目标真正丢失并超过保持时间后是正确安全状态；
- 自动控制的450ms容错不是无限预测，超过门限仍必须停车；
- 软件急停不能替代物理断电急停；
- 每次更换相机安装位置、Sony焦段或标定分辨率后，应重新验证内外参；
- 每次优化只改一层并保存同场景基线，禁止同时调感知、控制和驱动参数。

## 8. 后续变更规则

1. 当前参数作为 `v3.3.27` 实机基线；
2. 修改前先保存参数dump和最小rosbag；
3. 修改后使用相同人物、距离、遮挡和运动路径复测；
4. 记录目标丢失次数、ID切换次数、`LOST`占比、速度零跳变次数和停止距离；
5. 新方案只有在安全性不下降且指标优于基线时才能替代当前默认值；
6. 回归失败时按提交回退，不用继续叠加临时参数掩盖问题。
