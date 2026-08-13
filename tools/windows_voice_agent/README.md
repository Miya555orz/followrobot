# FCR Windows 常驻语音代理

该程序在 Windows 上持续监听麦克风，完成 Qwen3-ASR、八分类、意图切分与
参数提取，再通过 HTTP `/voice/command` 将结构化候选发送给 Jetson。Windows
只提交候选输入；Jetson 的
系统状态机、语音分发器、控制仲裁和急停链路拥有最终执行权。

## 安全边界

- 不让 Windows 加入 ROS 2 DDS 网络。
- 不从 Windows 直接发布底盘或云台速度。
- ASR/分类失败时不发送；网络恢复后不补发过期动作。
- Jetson 状态不可用或超时后，普通指令全部拒绝；仅急停短语可继续上送。
- `request_id` 在重试期间保持不变，由 Jetson 去重。
- 急停短语绕过本地分类过滤，但仍经过 Jetson HTTP 和安全链路。
- Jetson 不加载 ASR、BERT、BGE 或 Ollama，只做协议、状态、时效和安全裁决。

## 首次安装

在 PowerShell 中执行：

```powershell
cd D:\code\fcr_ros2\fcr_ros2_3\tools\windows_voice_agent
Set-ExecutionPolicy -Scope CurrentUser RemoteSigned
.\install_voice_agent.ps1
Copy-Item config.example.json config.json -Force
.\prepare_models.ps1
```

编辑 `config.json`：

1. `classifier.project_root` 指向 `classfier/fcr_speech_interpreter` 八分类工程；
2. `asr.module_root` 指向 `voice/server`，语音代理从其 `listener` 包加载 Qwen3-ASR；
3. `audio.device` 设为输入设备编号，或保留 `null` 使用默认麦克风；
4. `asr.device` 推荐保持 `auto`：CUDA 可用时使用 GPU，否则自动回退 CPU；
5. 执行 `prepare_models.ps1`，将八分类 BERT 和 BGE 下载到本工具的
   `models/` 目录；日常启动保持 `asr.offline=true`。Qwen ASR 若尚未缓存，
   再临时改为 `false` 启动一次完成下载，随后恢复 `true`。
6. 默认使用确定性数值解析，避免 LLM 故障时生成错误运动幅度。若实验性启用
   `extractor.enabled`，再安装 Ollama 并执行 `ollama pull qwen2.5:1.5b`；
   明确数值仍以原始语句的规则解析结果为准。

查询麦克风编号：

```powershell
.\.venv\Scripts\python.exe .\fcr_voice_agent.py --list-devices
```

只检查 Jetson 网络、鉴权和状态接口（不加载模型）：

```powershell
.\.venv\Scripts\python.exe .\fcr_voice_agent.py --check-jetson
```

配置与 Jetson 一致的令牌：

```powershell
[Environment]::SetEnvironmentVariable(
  "FCR_VOICE_AUTH_TOKEN",
  "替换成至少32位的ASCII随机令牌",
  "User"
)
```

不要把中文占位文字本身当作令牌。可在 PowerShell 中生成并同时写入当前进程与
用户环境变量：

```powershell
$rng = [Security.Cryptography.RandomNumberGenerator]::Create()
$bytes = New-Object byte[] 32
$rng.GetBytes($bytes)
$token = [BitConverter]::ToString($bytes).Replace('-', '').ToLower()
$rng.Dispose()
$env:FCR_VOICE_AUTH_TOKEN = $token
[Environment]::SetEnvironmentVariable("FCR_VOICE_AUTH_TOKEN", $token, "User")
"Token ready, length=$($token.Length)"
```

重新打开 PowerShell 后启动：

```powershell
.\run_voice_agent.ps1
```

## Jetson 启动条件

综合启动必须开启语音控制，且 `voice_http_auth_token` 与 Windows 环境变量一致。
为防止同网段注入控制，综合脚本在令牌为空时会拒绝启动；暂时不用语音请使用
`ros2 run bringup_pkg start_fcr.sh --controller pbvs --no-voice`。
Jetson 端验证：

```bash
curl http://127.0.0.1:8081/health
ros2 node list | grep -E "voice_text_http_bridge|voice_command_dispatcher|system_mode"
ros2 topic echo /system/state --once
ros2 topic echo /voice/dispatch_status
```

Jetson 只启动 HTTP 候选入口与权威分发器，不需要部署分类模型：

```bash
export FCR_VOICE_AUTH_TOKEN="与Windows一致的随机长令牌"
ros2 run bringup_pkg start_fcr.sh --controller pbvs
```

候选协议版本为 `1`。Jetson 将其转换为 `/external/voice_command`，随后由
`voice_command_dispatcher_node` 按当前 STANDBY/FOLLOW/CINEMATIC 状态决定接受、
拒绝、降级或急停。

Windows 验证：

```powershell
Invoke-RestMethod http://192.168.50.35:8081/health
$headers = @{Authorization="Bearer $env:FCR_VOICE_AUTH_TOKEN"}
Invoke-RestMethod http://192.168.50.35:8081/voice/state -Headers $headers
```

不加载麦克风或模型，发送一条无运动副作用的状态查询候选：

```powershell
.\send_status_query.ps1
```

HTTP `202` 仅表示候选已进入 Jetson 队列，不表示已经获准执行；最终结果必须以
Jetson 的 `/voice/dispatch_status` 为准。

Windows 和 Jetson 都应开启系统时间自动同步。网关会拒绝超过 10 秒的旧请求，
也会拒绝时间戳比 Jetson 快 5 秒以上的请求，防止断网恢复后执行陈旧动作。

### 首轮联调指令

先架空底盘并确保旁边有人持有物理急停，再按状态逐项测试：

- STANDBY：`云台向左五度`、`底盘向前十厘米`；
- FOLLOW：`开始跟随`、`跟近二十厘米`、`停止跟随`；
- CINEMATIC：`进入运镜模式`、`向左横移半米`、`向右环绕三十度`、
  `退出运镜模式`；
- 任意时刻：`紧急停车`。

Jetson 另开终端观察每条指令最终是接受还是拒绝：

```bash
ros2 topic echo /voice/dispatch_status
```

## 注册登录自启动

麦克风通常不能从 Windows Session 0 服务可靠访问，因此采用“用户登录时”任务，
而不是传统 Windows Service：

```powershell
.\register_startup_task.ps1
Start-ScheduledTask -TaskName "FCR Windows Voice Agent"
```

日志位于 `logs/voice_agent.log`，单文件 20 MiB，保留 10 份。

## VAD 调参

- 环境声也会触发：提高 `energy_threshold`，例如从 `0.015` 调到 `0.025`。
- 说话开头被截断：增加 `pre_roll_ms`。
- 一句话被切成两段：增加 `silence_timeout_ms`。
- 响应太慢：适当降低 `silence_timeout_ms`，但不要低于约 450 ms。

P0 使用持续 VAD。正式开放场景仍建议增加唤醒词，以减少环境谈话误触发。
