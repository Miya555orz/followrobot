# Sony—DJI RS2 二维高速跟踪链路

## 1. 当前架构

```text
Sony image_raw (源时间戳)
  -> YOLO TensorRT
  -> ByteTrack
  -> face_aim_node (alpha-beta位置/速度估计)
  -> /perception/aim_target_2d
  -> gimbal_visual_servo (50 Hz二维视线角快环)
  -> /auto/cmd_gimbal               [未启用语音路由]
     /autonomy/cmd_gimbal -> /auto  [启用语音路由]
  -> command_mux (50 Hz，最终限幅/模式/急停)
  -> /cmd_gimbal
  -> gimbal_driver (speed模式，默认20 Hz实际CAN下发)
  -> DJI RS2
```

`gimbal_visual_servo`不订阅Gemini深度、`/perception/targets_3d`、TF或底盘状态历史。
原`servo_manager`继续负责PBVS三维距离和底盘回中，但启用二维快环后，其云台输出被隔离到
`/servo/internal/legacy_cmd_gimbal`，不会与二维快环争用候选命令。

## 2. 核心算法

关注点和期望构图点先按Sony内参归一化：

```text
x  = (u  - cx) / fx       y  = (v  - cy) / fy
x* = (u* - cx) / fx       y* = (v* - cy) / fy
```

随后计算真实视线角，而不是直接把像素差线性映射到电机速度：

```text
yaw_error   = atan(x) - atan(x*)
pitch_error = atan2(y, sqrt(1+x^2))
            - atan2(y*, sqrt(1+x*^2))
```

第一版采用位置反馈，yaw和pitch独立增益、速度上限和角加速度上限。增益通过平滑插值连续调度：
小误差降低增益以抑制抖动，大误差提高增益以快速追赶。双轴误差连续进入小阈值后进入静稳保持，
退出阈值大于进入阈值，形成滞回。

## 3. 预测与数据权限

`face_aim_node`只使用一层alpha-beta滤波，联合输出像素位置、像素速度和残差协方差。
旧固定LPF已移除，避免多层低通叠加相位滞后。

控制时预测点：

```text
u_pred = u + velocity_u * T
v_pred = v + velocity_v * T
T = clip(observation_age + rs2_latency, 0, max_prediction)
```

实际`T`还会随置信度下降、协方差增大和观测年龄增大而缩短。权限分层为：

- `age <= 0.08 s`：正常反馈和有限预测；
- `0.08 < age <= 0.18 s`：连续降低增益和速度上限；
- `0.18 < age <= 0.30 s`：仅允许显式`PREDICTED`状态，命令随年龄衰减；
- `age > 0.30 s`：立即hold。

急停、RS2断连、平台状态超时、相机内参缺失和软限位同样会撤销运动权限。

## 4. 最新值策略

- 图像、检测、跟踪、AimTarget2D和控制命令均采用`KEEP_LAST(1)`；
- 控制快环50 Hz，只传播最新30 Hz视觉观测，不伪造新测量；
- `command_mux`是`/cmd_gimbal`唯一发布者；
- 云台驱动的最终命令订阅深度为1；
- 云台驱动只保存一个`latest_cmd`，独立定时器以20 Hz发送，启停边沿立即下发；
- 先验证20 Hz，再单独A/B测试30/40 Hz，不同步修改控制增益。

## 5. 时间戳和诊断

`AimTarget2D.header.stamp`始终是最后一次真实Sony测量时刻；`estimate_stamp`是当前
滤波状态对应的时间坐标。LOST期间二者会分离：前者用于撤权，后者用于避免重复预测。
`GimbalCmd`继续携带：

- `source_stamp`：Sony采集时刻；
- `control_stamp`：二维快环计算时刻；
- `mux_stamp`：command_mux选中时刻。

`/diagnostics`中的`gimbal_visual_servo: fast_2d_loop`提供观测年龄、预测时域、角误差、命令和
capture-to-control延迟。`/gimbal/status`提供source/control/mux到CAN下发尝试的分段延迟。

## 6. 参数入口

- 二维控制参数：`src/servo_control_pkg/config/gimbal_visual_servo.yaml`
- alpha-beta观测参数：`src/perception_pkg/config/face_aim_params.yaml`
- command_mux最终硬限幅：`src/teleop_control_pkg/config/command_mux.yaml`
- RS2发送频率：`src/robot_platform_pkg/config/gimbal_params.yaml`

调参顺序固定为：方向与安全限位 -> 时间戳/年龄 -> yaw响应 -> pitch稳定 -> 预测 -> 发送频率。
一次只改变一层，不同时提高增益、预测时域和CAN发送频率。

## 7. Jetson构建

`AimTarget2D`、`GimbalCmd`和`GimbalStatus`接口已扩展，必须把消息包和所有
直接使用者一起重新构建：

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install \
  --packages-select \
    vision_servo_msgs \
    perception_pkg \
    servo_control_pkg \
    teleop_control_pkg \
    external_control_pkg \
    robot_platform_pkg \
    remote_monitor_pkg \
    bringup_pkg \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source ~/ros2_ws/install/setup.bash
colcon test --packages-select servo_control_pkg
colcon test-result --verbose
```

若旧的消息生成结果造成ABI缓存问题，只删除上述八个包对应的`build/`和
`install/`目录后重建，不要清理整个工作区。

## 8. 实机验收

启动默认的PBVS底盘慢环 + Sony二维云台快环：

```bash
ros2 run bringup_pkg start_fcr.sh --controller pbvs
```

在另一个终端切换自动模式：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic pub --once /teleop/mode std_msgs/msg/String "{data: auto}"
```

先验收架构与话题唯一性：

```bash
ros2 node list | sort | uniq -c | grep -E \
  "gimbal_visual_servo|servo_manager|command_mux|gimbal_driver"
ros2 topic info /cmd_gimbal -v
ros2 topic info /auto/cmd_gimbal -v
ros2 topic hz /perception/aim_target_2d
ros2 topic echo /gimbal/status --once
```

期望：`gimbal_visual_servo`只有一个；`/cmd_gimbal`只有`command_mux`一个
发布者；`GimbalStatus`的三个分段延迟字段不持续增长。

最后验收不依赖深度的二维链路：

```bash
ros2 run bringup_pkg start_fcr.sh \
  --controller pbvs \
  --no-gemini \
  --no-translation \
  --no-voice
```

此时`/perception/targets_3d`可以不存在，底盘应停止平移，但人物进入Sony画面后
`/perception/aim_target_2d`和云台yaw/pitch命令仍应持续更新。

如需A/B回退到旧的统一PBVS云台输出：

```bash
ros2 run bringup_pkg start_fcr.sh \
  --controller pbvs \
  --legacy-gimbal-loop
```
