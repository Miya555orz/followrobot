# OpenCode Review Prompt - Jetson to TRON1 Network-Only Preflight

请审查 `/home/miya/follow_ws/src/fcr_ros2_3` 当前提交相对上一版的改动，重点确认它是否仍然满足 TRON1 实机前安全边界。

本轮目标：

1. 保存 2026-09-05 PC 直连 TRON1 的只读 preflight 结果。
2. 增加 Jetson 侧连接 TRON1 的 network-only 预检脚本与文档。
3. 为下一步拓扑做准备：`PC --USB/SSH--> Jetson`，`Jetson --Ethernet--> TRON1`。

必须区分：

- 这是 network-only 连通性测试准备，不是 controller bringup。
- 不应启动 ROS graph、`robot_hw`、官方 controller。
- 不应发布 `/cmd_vel`、`/fcr_tron/cmd_vel` 或任何速度命令。
- 不应加入任何“接上网线即可地面运动”的文字。
- real motion 必须继续 paused，直到 A-10 人工项、Gazebo 零漂 blocker、controller/SDK/hardware stop 后果确认、架空/支架分步 checklist 都完成。

请重点检查：

- `tools/tron1_bringup/jetson_tron1_network_preflight.sh`
  - 是否只运行 `ip`/`ping` 这类只读网络检查。
  - 是否没有 `ros2 launch`、`ros2 run`、`ros2 topic pub`、controller 激活或速度发布。
  - 是否正确 BLOCK 代理/TUN/container/policy-table 路由。
  - 是否正确支持 `TRON_LINK_IFACE=<Jetson接TRON1的有线接口名>` 白名单。
  - 退出码语义是否清晰：`0=PASS, 1=FAIL, 2=WARN-only, 3=BLOCK`。
- `docs/TRON1_REAL_TEST_STEP_CHECKLIST.md`
  - Step 2 是否明确从 Jetson network-only 开始。
  - 是否明确禁止 `robot_hw`、controller 激活、速度发布、`enable_motion=true`。
  - 是否没有把 network-only PASS 说成实机运动许可。
- `README.md`、`docs/HANDOFF.md`、`docs/ai/CURRENT_STATUS.md`
  - 是否正确记录 PC 直连 preflight 已 PASS，但小办公室内未做运动且已断开。
  - 是否明确下一步只允许 Jetson 上的 route/ping/preflight。
- `docs/ai/TRON1_PC_DIRECT_PREFLIGHT_2026-09-05.md`
  - 是否准确保存用户本次 PC 直连证据。

建议验证：

```bash
git status --short
git diff --check
bash -n tools/tron1_bringup/jetson_tron1_network_preflight.sh \
  tools/tron1_bringup/tron1_real_motion_path_preflight.sh \
  tools/tron1_bringup/tron1_safety_acceptance_check.sh \
  tools/tron1_bringup/pc_jetson_network_preflight.sh
rg -n "ros2 (launch|run|topic pub)|enable_motion:=true|L1 \\+ 三角|L1 \\+ Y" tools/tron1_bringup/jetson_tron1_network_preflight.sh docs README.md
```

注意：文档中的“禁止命令/禁止动作”清单会命中上面的 `rg`，请按上下文判断；只有把这些动作作为可执行下一步才算问题。

请按 P0/P1/P2/P3 输出问题；若无 P0/P1，请说明是否可以合入。不要建议任何会让 TRON1 运动的测试。
