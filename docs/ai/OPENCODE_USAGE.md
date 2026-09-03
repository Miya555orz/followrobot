# OpenCode 使用指南

这份文档给当前项目的后续 Codex / OpenCode / 人类协作者使用。目标是：当 Codex token 不够、或者想让另一个 AI 先做读代码/写文档/小改动时，可以安全地把任务交给 OpenCode。

当前项目根目录：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
```

当前 OpenCode 可执行文件：

```bash
/home/miya/.opencode/bin/opencode
```

如果普通 `opencode` 命令不可用，优先使用上面的完整路径。

## 1. 每次开始前先检查

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
git status --short --branch --untracked-files=all
/home/miya/.opencode/bin/opencode --version
/home/miya/.opencode/bin/opencode models
```

期望：

- `git status` 能看到当前是否有未提交文件。
- `opencode --version` 当前验证为 `1.18.27`。
- `opencode models` 能列出可用模型。

如果 `opencode models` 偶发提示 database locked，先等几秒，确认没有另一个 OpenCode 进程卡住：

```bash
ps -ef | rg 'opencode|ollama|ros2|gazebo|robot_hw_node'
```

## 2. DeepSeek V4 Flash：只登录一次 API key

当前推荐模型：

```text
deepseek/deepseek-v4-flash
```

第一次配置时运行：

```bash
/home/miya/.opencode/bin/opencode auth login --provider deepseek
```

按提示粘贴 DeepSeek API key。OpenCode 会把凭据存到本机 credential store；不要把 API key 写进 README、脚本、`opencode.json` 或 git commit。

登录后检查：

```bash
/home/miya/.opencode/bin/opencode auth list
/home/miya/.opencode/bin/opencode models deepseek
```

期望看到：

```text
DeepSeek api
deepseek/deepseek-v4-flash
```

## 3. 最推荐的安全入口：fallback 脚本

优先用这个脚本，不要直接随手让 OpenCode 改项目。脚本会自动把项目上下文和安全边界塞进 prompt。

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./scripts/opencode-fallback.sh "Read README.md and docs/ai/*.md, summarize current project status in Chinese, and do not modify any files."
```

常用模板：

```bash
./scripts/opencode-fallback.sh "只阅读代码，找出 TRON1 cmd_vel 适配器应该放在哪个包里，不要修改文件。"
```

```bash
./scripts/opencode-fallback.sh "更新 docs/ai/CURRENT_STATUS.md，补充今晚验证结果。不要启动 ROS，不要接触真机运动。"
```

```bash
./scripts/opencode-fallback.sh "检查 src 下面所有 package.xml 和 CMakeLists.txt，报告可能影响 colcon build 的问题。只读，不修改。"
```

## 4. 直接运行 OpenCode

打开交互界面：

```bash
/home/miya/.opencode/bin/opencode /home/miya/follow_ws/src/fcr_ros2_3
```

在 OpenCode TUI 里，先让它读：

```text
Read AGENTS.md and docs/ai/*.md first. Summarize the project state and safety rules before editing anything.
```

再给任务。涉及机器人、TRON1、速度、底盘运动、CAN、网络、udev、systemd、sudo 的任务，必须先让它只出计划，不要执行。

## 5. Harness 用法

本项目已在 `opencode.json` 配置 Harness 插件：

```json
"plugin": [
  "github:JEF1056/harness#main"
]
```

进入 TUI 后可以尝试：

```text
/harness 分析 TRON1 cmd_vel adapter 的实现计划，只读，不修改
/plan 给出 Sony + RS2 + Orbbec + TRON1 分阶段测试计划
/map
/debug 粘贴第一个真正的 error
```

注意：Harness 插件与路由配置已经准备好；完整多代理写入流程还没有在真机/项目里做最终验证。真正要改代码时，先从文档或小配置改动开始。

路由说明在：

```text
docs/ai/HARNESS_ROUTING.md
```

## 6. 切换模型

查看可用模型：

```bash
/home/miya/.opencode/bin/opencode models
```

通过环境变量切换：

```bash
OPENCODE_MODEL=deepseek/deepseek-v4-flash ./scripts/opencode-fallback.sh "TASK"
```

因为脚本已经默认 `deepseek/deepseek-v4-flash`，日常可以简写为：

```bash
./scripts/opencode-fallback.sh "TASK"
```

直接命令切换：

```bash
/home/miya/.opencode/bin/opencode run --dir /home/miya/follow_ws/src/fcr_ros2_3 --model deepseek/deepseek-v4-flash "TASK"
```

当前本机没有检测到 Ollama，也没有检测到 NVIDIA GPU。以后如果安装 Ollama，先用小模型：

```bash
ollama list
ollama pull qwen2.5-coder:7b-instruct
OPENCODE_MODEL=ollama/qwen2.5-coder:7b-instruct ./scripts/opencode-fallback.sh "TASK"
```

不要盲目拉 30B+ 大模型；这台机器大约 16 GiB RAM，没有可见 NVIDIA GPU。

## 7. 哪些任务可以交给 OpenCode

比较安全：

- 读代码、读文档、总结架构。
- 修改 `docs/ai/*.md`、README、交接文档。
- 检查 `package.xml`、`CMakeLists.txt`、launch/config 文件。
- 写不接真机的 adapter 草稿。
- 跑只读命令，例如 `git status`、`rg`、`ls`、`python3 -m json.tool`。

必须人工确认：

- `ros2 launch`。
- `ros2 topic pub`。
- 任何 `/cmd_vel`、速度、加速度、底盘运动、关节、电机相关操作。
- TRON1 SDK、以太网、CAN、Jetson 网络配置。
- `sudo`、`/etc`、udev、systemd、kernel module。
- 删除、重置、覆盖大量文件。

严禁自动做：

- 直接向 TRON1 裸 `/cmd_vel` 发运动命令。
- 未验证 watchdog / timeout / e-stop 就让真机运动。
- 为了省事修改系统网络、udev、CAN、kernel 配置。
- 把 API key 或 token 写进仓库。

## 8. Codex token 快用完时的最短工作流

复制这个：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./scripts/opencode-fallback.sh "这里写要交给 OpenCode 的任务。先读 AGENTS.md 和 docs/ai/*.md。不要动真机，不要 sudo，不要 ROS launch，除非我明确批准。"
```

做完后检查：

```bash
git status --short --untracked-files=all
git diff --stat
```

如果 OpenCode 改了文件，先让 Codex 或人类 review，再 commit。

## 9. 当前验证状态

- OpenCode 安装路径：`/home/miya/.opencode/bin/opencode`
- OpenCode 版本：`1.18.27`
- DeepSeek 凭据：`opencode auth list` 已能看到 `DeepSeek api`
- 推荐模型：`deepseek/deepseek-v4-flash`，并且 `scripts/opencode-fallback.sh` 已默认使用它
- 已验证：fallback 脚本可以让 OpenCode 读取项目上下文并输出 ROS2 包总结。
- 未完成：Harness 多代理写入任务尚未完整验证。
- 未启用：Ollama / 本地模型。
