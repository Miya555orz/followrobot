# perception_pkg

当前生产链路实现 Sony 主 RGB 图像目标检测与跟踪，并将 Gemini 335 深度
重投影到 Sony 图像完成锁定目标的稳健 3D 定位。避障、建图和 Nav2 不属于
本包当前范围。

## 当前数据流

```text
Sony image
  -> detection_node (YOLO ONNX)
  -> /perception/detections
  -> tracking_node (ByteTrack V2: XYAH Kalman + state-specific Hungarian association)
  -> /perception/tracks
  + Gemini depth + calibrated TF
  -> depth_fusion_node
  -> /perception/targets_3d
```

`Target.bbox` 和 `Target.center` 始终使用 Sony 输入图像的像素坐标。检测和跟踪
结果保留 Sony 原始时间戳及 `frame_id`。

仓库原有 `depth_estimator_node` 和 `PerceptionPipeline` 仅为历史仿真兼容
代码，默认既不构建也不启动。只有显式设置
`-DPERCEPTION_BUILD_LEGACY_FUSION=ON` 才会生成它们；生产环境不得使用该选项。

## depth_fusion_node

- 使用已标定的 Gemini depth → Sony optical TF 重投影深度点，不直接混用两台
  相机的像素坐标。
- 通过有界双队列按最近时间戳配对，过期数据直接丢弃，防止延迟累积。
- 仅从人体框中央上半身 ROI 取深度，使用分位数种子、MAD 离群剔除和中位统计。
- 每个 tracking ID 独立维护 XYZ/速度滤波；XY 保持响应，Z 使用更强平滑。
- 通过位置跳变、横向速度和深度速度门控拒绝异常测量，异常值不会重置滤波器。
- 当前锁定 ID 可在 0.45 秒内发布明确标记的 `PREDICTED` 结果；其他 ID 不预测。
- 融合状态为 `VALID / DEGRADED / PREDICTED / INVALID`。只有真实的 VALID 或
  DEGRADED 深度可以进入底盘平移，DEGRADED 平移速度减半。
- `/diagnostics` 提供拒绝测量数、预测目标数、同步误差、队列长度和输出数据年龄。

## detection_node

- 支持常见 YOLOv8 `[1, 84, 8400]` 和 YOLOv5 `[1, N, 85]` ONNX 输出。
- 启动时严格检查模型文件、输入尺寸、输出形状和类别数量，并执行模型预热。
- 实现 letterbox、归一化、OpenCV DNN 前向、坐标还原和按类别 NMS。
- 推理在后台线程运行，待处理缓存只有一帧；过载时丢弃旧帧，避免延迟累积。
- `labels_path` 为空时使用 COCO 80 类，默认只输出 `person`。
- 默认发布阈值为 `0.10`，为 ByteTrack 保留低分恢复候选；这些候选不能直接
  创建新 ID，新轨迹仍需满足 `new_track_threshold`。
- 模型二进制不进 Git；下载脚本按 SHA-256 校验后，CMake 才将其安装到包内。
- `device=cuda_fp16` 会先检查 OpenCV CUDA DNN target；不可用时明确退出，不静默
  回退。Jetson 生产路径使用 `device=tensorrt` 和本机生成的 `.engine`。
- 每隔 `performance_log_period` 秒输出处理帧率、平均/最大推理耗时和累计丢帧数。

## tracking_node

- 8 维恒速 XYAH Kalman 状态：`[cx, cy, aspect, h, vx, vy, va, vh]`，并使用
  Mahalanobis 门控排除与预测协方差明显冲突的匹配。
- 默认使用原生 C++ ByteTrack V2：CONFIRMED、LOST 和 TENTATIVE 分阶段关联，
  LOST 使用更宽松的恢复门限；代价同时考虑 IoU、中心距离和尺寸变化。
- 低分框只能恢复已有 ID。高分新目标先作为匿名候选连续出现；默认至少满足
  `new_track_delay_frames` 与 `min_confirm_hits` 后才分配 ID。LOST 轨迹附近的新
  候选需要更长确认，减少遮挡后立即换号。
- `TENTATIVE -> CONFIRMED -> LOST -> removed` 生命周期完整映射到现有消息；
  LOST 轨迹按秒而非按帧过期，默认保留 2.5 秒，摄像头帧率变化不会改变恢复窗口。
- 默认不发布 TENTATIVE 轨迹，避免未经确认的 ID 进入目标选择和控制链。
- 可选稀疏光流 + RANSAC 全局运动补偿订阅同一 Sony 图像，在机器人/云台运动时
  先修正预测框；估计失败自动退回纯 Kalman 预测，不中断跟踪。
- `tracker_type=legacy_iou` 可回退到升级前的单阶段 IoU 跟踪器进行 A/B 对照。
- 默认自动选择面积与置信度综合得分最高的 `person`。
- 目标选择服务：`/tracking_node/set_tracking_target`。
- `/diagnostics` 报告输入超时和最近端到端延迟，并周期输出平均、P95、最大延迟。

## 构建与测试

```bash
source /opt/ros/humble/setup.bash
python3 src/perception_pkg/scripts/download_model.py
colcon build --symlink-install --packages-up-to perception_pkg
source install/setup.bash
colcon test --packages-select perception_pkg
colcon test-result --verbose
```

## 启动

默认加载随 `perception_pkg` 安装的 `yolov8n.onnx`：

```bash
ros2 launch perception_pkg perception.launch.py \
  sony_image_topic:=/sony/image_raw
```

也可以覆盖模型路径和推理设备：

```bash
ros2 launch perception_pkg perception.launch.py \
  model_path:=/absolute/path/to/yolov8n.onnx \
  device:=cpu
```

Jetson 上的 TensorRT 10 路径（engine 只能由目标机可信 ONNX 本地生成）：

```bash
/usr/src/tensorrt/bin/trtexec \
  --onnx=$HOME/fcr_models/yolov8n.onnx \
  --saveEngine=$HOME/fcr_models/yolov8n_fp16.engine \
  --fp16 --memPoolSize=workspace:1024 --skipInference

ros2 launch perception_pkg perception.launch.py \
  model_path:=$HOME/fcr_models/yolov8n_fp16.engine \
  device:=tensorrt sony_image_topic:=/sony/image_raw
```

临时回退旧跟踪器：

```bash
ros2 launch perception_pkg perception.launch.py tracker_type:=legacy_iou
```

若实机画面纹理太少或需要单独对比相机运动补偿，可只关闭 GMC，ByteTrack V2
其余逻辑保持不变：

```bash
ros2 launch perception_pkg perception.launch.py \
  enable_camera_motion_compensation:=false
```

TensorRT 支持仅在检测到 `NvInfer.h`、`libnvinfer` 和 CUDA runtime 开发文件时
条件编译；普通 Humble CI 自动只构建 ONNX/OpenCV 路径。

完成 SDK 本地安装后，可在 Jetson 工作空间根目录执行可重复构建与测试：

```bash
bash src/perception_pkg/scripts/jetson_build_perception.sh "$PWD"
```

启动 Sony、TensorRT 检测和跟踪链后，执行五分钟接口采样：

```bash
ACCEPTANCE_DURATION_SECONDS=300 \
  bash src/perception_pkg/scripts/jetson_acceptance.sh \
  "$HOME/fcr_models/yolov8n_fp16.engine"
```

脚本只自动检查 topic 存在性、消息可达性、采样频率以及融合前不得出现的 3D
topic；相机 30 分钟稳定性、诊断状态和人物 ID 切换仍需实机观察确认。

也可以只调试其中一段：

```bash
# 只运行 YOLO
ros2 launch perception_pkg perception.launch.py enable_tracking:=false

# 使用外部/mock detections，只运行跟踪
ros2 launch perception_pkg perception.launch.py enable_detection:=false
```

检查接口：

```bash
ros2 topic hz /perception/detections
ros2 topic hz /perception/tracks
ros2 topic echo /perception/tracks --once
ros2 topic info /perception/detections -v
```

实机接入前先确认 Sony 发布图像的分辨率、编码、时间戳和光学坐标系稳定，再
进行模型速度与跟踪稳定性评估。
