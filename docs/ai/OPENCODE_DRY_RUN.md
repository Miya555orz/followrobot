# OpenCode Dry Run Record

日期：2026-09-03

## 已通过

使用命令：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
OPENCODE_MODEL=opencode/mimo-v2.5-free ./scripts/opencode-fallback.sh "Read README.md and all src/*/package.xml files, summarize the ROS2 packages in 5 bullets, and do not modify any files."
```

结果：

- OpenCode 能从当前项目根目录运行。
- OpenCode 能读取 `AGENTS.md` 和 `docs/ai/*.md` 上下文。
- OpenCode 能读取 `README.md` 和 `src/*/package.xml`。
- OpenCode 能输出 5 条 ROS2 包总结。
- 没有要求或执行真机运动。

## 未完整通过

尝试过直接使用：

```bash
/home/miya/.opencode/bin/opencode run --dir /home/miya/follow_ws/src/fcr_ros2_3 --agent coder --auto "..."
```

观察到：

- OpenCode 提示 `coder` 是 subagent，不是 primary agent，并回退到默认 agent。
- 该写入任务卡住后被人工中断。
- 因此不要把 `--agent coder` 当成稳定入口。

推荐继续使用：

```bash
OPENCODE_MODEL=opencode/mimo-v2.5-free ./scripts/opencode-fallback.sh "TASK"
```

Harness 多代理流程请优先在 OpenCode TUI 内用 `/harness`、`/plan`、`/debug` 小任务验证。

