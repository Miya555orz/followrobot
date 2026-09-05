# OpenCode Review Prompt - Network Evidence and Route Setup

请审查 `/home/miya/follow_ws/src/fcr_ros2_3` 当前工作区改动，重点确认 13:30-14:45 两个目标是否完成且没有安全语义倒退。

目标 1：证据归档 + OpenCode 审查准备

- README / `docs/ai/CURRENT_STATUS.md` / `docs/HANDOFF.md` 应明确写清楚：
  - PC -> Jetson USB SSH 可用；
  - Jetson -> TRON1 Ethernet network-only PASS；
  - 真实运动仍暂停。
- `docs/ai/TRON1_JETSON_NETWORK_PREFLIGHT_2026-09-05.md` 应保存 route/ping/preflight 证据，并说明不是实机运动许可。

目标 2：网络设置可复现流程

- 新增 `tools/tron1_bringup/jetson_tron1_route_setup.sh`，用途只限 Jetson 上配置 TRON1 Ethernet 的 link/address/route。
- 默认应是 dry-run；只有 `--apply` 才执行 `sudo ip link/ip addr/ip route`。
- 不应 source ROS，不应 `ros2 launch`，不应 `ros2 run`，不应 `ros2 topic pub`，不应启动 `robot_hw`，不应激活 controller，不应发布速度。
- `docs/ai/TRON1_JETSON_NETWORK_SETUP_REPEATABLE_2026-09-05.md` 应同时记录临时 `ip route replace` 流程和可选 NetworkManager profile；NetworkManager profile 必须是 route-only/manual，不能引入 controller autostart。

请重点检查：

```bash
git status --short
git diff --check
bash -n tools/tron1_bringup/jetson_tron1_route_setup.sh \
  tools/tron1_bringup/jetson_tron1_network_preflight.sh \
  tools/tron1_bringup/tron1_real_motion_path_preflight.sh \
  tools/tron1_bringup/tron1_safety_acceptance_check.sh
rg -n "ros2 (launch|run|topic pub)|enable_motion:=true|robot_hw|controller|cmd_vel|L1 \\+ Y|triangle" \
  tools/tron1_bringup/jetson_tron1_route_setup.sh \
  docs/ai/TRON1_JETSON_NETWORK_SETUP_REPEATABLE_2026-09-05.md \
  docs/ai/TRON1_JETSON_NETWORK_PREFLIGHT_2026-09-05.md \
  README.md docs/HANDOFF.md docs/ai/CURRENT_STATUS.md
```

注意：文档里的“禁止命令/安全边界”会命中上述 `rg`，请按上下文判断。只有把这些动作作为可执行下一步，或把 network-only PASS 写成运动许可，才算问题。

请按 P0/P1/P2/P3 输出问题；若无 P0/P1，请说明是否可以合入。不要建议任何会让 TRON1 运动的测试。
