# 笔记本 ASR 到 Jetson ROS 2 语音控制

## 1. 数据流

```text
Windows 麦克风
  -> Qwen3-ASR-0.6B (CPU)
  -> HTTP POST /voice/text
Jetson voice_text_http_bridge_node
  -> /voice/text (std_msgs/msg/String)
  -> intent_classifier_node
  -> /external/voice_command
  -> voice_command_dispatcher_node
  -> 云台或底盘控制节点
```

Windows 只负责录音和语音转文字，不直接发送运动控制指令。意图分类、指令分发、优先级仲裁和硬件安全保护均保留在 Jetson。

## 2. 接口约定

请求：

```http
POST http://JETSON_IP:8081/voice/text
Content-Type: application/json
```

```json
{
  "text": "西西，云台向右一点",
  "request_id": "d2c4c4fa-54ae-4dc4-b662-08a6ef8bcd20",
  "timestamp": 1780000000.0,
  "source": "windows_qwen3_asr"
}
```

正常接收返回 HTTP 202。相同 `request_id` 在 60 秒内重发时返回 HTTP 200、`duplicate=true`，不会再次发布 ROS 消息。

## 3. Jetson 构建

```bash
cd ~/ros2_ws/src/fcr_ros2_3
git pull --ff-only origin main

source /opt/ros/humble/setup.bash
source ~/venvs/fcr_runtime/bin/activate

colcon build --packages-select \
  voice_intent_pkg external_control_pkg

source install/setup.bash
```

确认新入口存在：

```bash
ros2 pkg executables voice_intent_pkg | grep voice_text_http_bridge_node
```

## 4. 先单独验证 HTTP 到 ROS

Jetson 终端 1：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/fcr_ros2_3/install/setup.bash

ros2 run voice_intent_pkg voice_text_http_bridge_node --ros-args \
  -p bind_address:=0.0.0.0 \
  -p port:=8081 \
  -p text_topic:=/voice/text
```

Jetson 终端 2：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/src/fcr_ros2_3/install/setup.bash
ros2 topic echo /voice/text std_msgs/msg/String
```

在 Jetson 查询局域网地址：

```bash
hostname -I
```

在 Windows PowerShell 测试，替换 `192.168.50.35`：

```powershell
$body = @{
    text = "西西，云台向右一点"
    request_id = [guid]::NewGuid().ToString()
    timestamp = [DateTimeOffset]::UtcNow.ToUnixTimeMilliseconds() / 1000
    source = "windows_manual_test"
} | ConvertTo-Json

Invoke-RestMethod `
  -Uri "http://192.168.50.35:8081/voice/text" `
  -Method Post `
  -ContentType "application/json" `
  -Body $body
```

Jetson `/voice/text` 应只出现一次对应文本。

## 5. 启动 Jetson 分类和控制链路

关闭第 4 节单独启动的桥接节点，然后执行：

```bash
cd ~/ros2_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash
source ~/venvs/fcr_runtime/bin/activate
source install/setup.bash

export PYTHONNOUSERSITE=1
unset PYTHONHOME
export CUDA_VISIBLE_DEVICES=""

ros2 launch external_control_pkg voice_control.launch.py \
  start_wake_up_node:=false \
  start_text_http_bridge:=true \
  start_intent_classifier:=true \
  publish_cloud_intents:=false \
  start_dispatcher:=true \
  classifier_model_root:=$HOME/ros2_ws/models/classifier_v2 \
  embedding_model_dir:=$HOME/ros2_ws/models/bge-base-zh-v1.5 \
  text_http_bind_address:=0.0.0.0 \
  text_http_port:=8081
```

先观察分类链路：

```bash
ros2 topic echo /voice/text std_msgs/msg/String
ros2 topic echo /external/intent_result std_msgs/msg/String
ros2 topic echo /external/voice_command external_control_pkg/msg/VoiceCommand
```

真实硬件驱动和控制路由继续使用已经验收过的云台、底盘启动步骤。首次联调应架空底盘轮子，并确认手动控制和急停仍可覆盖语音控制。

## 6. Windows 麦克风发送

```powershell
conda activate fcr_voice_asr
cd "D:\code\fcr_ros2\voice\server"

python .\mic_test.py `
  --device 1 `
  --duration 5 `
  --server-url "http://192.168.50.35:8081"
```

成功时 Windows 输出类似：

```text
Recognition result: 西西向右转一点。
Send result: accepted=True duplicate=False request_id=...
```

## 7. 可选共享令牌

Jetson 启动参数增加：

```bash
text_http_auth_token:=替换为共享令牌
```

Windows 增加相同参数：

```powershell
--auth-token "替换为共享令牌"
```

不要把真实令牌写进 Git 仓库或终端截图。

## 8. 排查命令

Windows 检查网络和健康接口：

```powershell
Test-NetConnection 192.168.50.35 -Port 8081
Invoke-RestMethod "http://192.168.50.35:8081/health"
```

Jetson 检查端口和节点：

```bash
ss -lntp | grep 8081
ros2 node list | grep -E 'voice_text|intent_classifier|dispatcher'
ros2 topic info /voice/text -v
```

如果 Jetson 使用 UFW：

```bash
sudo ufw allow from 192.168.50.0/24 to any port 8081 proto tcp
```

按实际局域网网段修改规则，不要将端口直接暴露到公网。
