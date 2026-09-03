# OpenCode Workflow

OpenCode is installed for this machine at:

```text
/home/miya/.opencode/bin/opencode
```

Use it directly if the current shell has not loaded `.bashrc`.

## Fallback Command

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
./scripts/opencode-fallback.sh "Summarize README.md and package.xml files. Do not modify anything."
```

With a chosen model:

```bash
OPENCODE_MODEL=provider/model ./scripts/opencode-fallback.sh "TASK"
```

## Model Switching

OpenCode accepts models as `provider/model`.

Use one of:

- `OPENCODE_MODEL=provider/model ./scripts/opencode-fallback.sh "TASK"`
- `opencode run --model provider/model "TASK"`
- OpenCode TUI `/models`
- Project `opencode.json` if a stable provider is later configured

No API keys should be committed. Use `opencode providers login`, environment variables, or OpenCode's credential store.

## Current Local Model Status

Ollama was not installed during setup, and no local Ollama model was detected. The project is prepared for local model use, but local routing is not active until Ollama or another local OpenAI-compatible server is installed and a model is pulled.

Because this laptop showed no NVIDIA GPU through `nvidia-smi` and has about 16 GiB RAM, prefer small/medium coding models if local inference is added later. Do not pull very large models blindly.

Suggested future local choices:

- `qwen2.5-coder:7b-instruct` for lightweight docs, config, and small code edits.
- `qwen2.5-coder:14b-instruct` only if latency and memory are acceptable.
- Avoid 30B+ local models on this machine unless external GPU/VRAM is available.

