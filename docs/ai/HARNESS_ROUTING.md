# Harness Routing

This file documents the intended Codex + OpenCode + Harness workflow for followrobot.

`harness.json` is intentionally kept compatible with `github:JEF1056/harness#main`. It currently contains only an empty `models` map because no local Ollama model or authenticated cloud model was verified during setup.

## Router

```text
Task
  -> Harness Router
      -> Planner / Explorer
      -> Coder
      -> Reviewer
      -> Challenger
      -> Tests
      -> Report
```

## Workflows

Simple task:

```text
Coder
  -> Tests
  -> Report
```

Complex task:

```text
Planner
  -> Coder
  -> Reviewer
  -> Challenger
  -> Tests
  -> Report
```

Safety-critical robot task:

```text
Planner
  -> Coder
  -> Reviewer
  -> Challenger
  -> SIMULATION ONLY
  -> Human approval
  -> REAL ROBOT
```

## Intended Model Routing

Current active state:

- Local model: unavailable, because Ollama is not installed.
- Cloud fallback: available only after user authentication in OpenCode.
- Harness model overrides: disabled for now to avoid hardcoding a model that is not installed.

Future model override format:

```json
{
  "models": {
    "Explorer": "provider/model",
    "Coder": "provider/model",
    "Reviewer": "provider/model",
    "Challenger": "provider/model"
  }
}
```

Recommended policy after models are available:

- Planner or Explorer: strongest available model for architecture and official-doc interpretation.
- Coder: fast local coding model for low-risk edits.
- Reviewer: different model from Coder when possible.
- Challenger: independent model or provider when possible to reduce self-review bias.
- Researcher: cloud/search-capable model when official docs or external sources are required.

Do not set these in `harness.json` until `opencode models` confirms the exact model IDs.

