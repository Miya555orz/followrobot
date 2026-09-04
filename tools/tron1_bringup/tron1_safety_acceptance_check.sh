#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/miya/follow_ws/src/fcr_ros2_3"
FOLLOW_WS="/home/miya/follow_ws"
LIMX_WS="/home/miya/limx_ws"
export ROBOT_TYPE="${ROBOT_TYPE:-WF_TRON1A}"
export RL_TYPE="${RL_TYPE:-isaacgym}"

fail_count=0
warn_count=0
block_count=0

pass() { echo "[PASS] $*"; }
warn() { echo "[WARN] $*"; warn_count=$((warn_count + 1)); }
fail() { echo "[FAIL] $*"; fail_count=$((fail_count + 1)); }
block() { echo "[BLOCK] $*"; block_count=$((block_count + 1)); }

has_ros_graph_topic() {
  timeout 3s ros2 topic info "$1" -v >/tmp/tron1_topic_info.txt 2>&1
}

launch_arg_default() {
  local file="$1"
  local arg="$2"
  awk -v arg="'$arg':" '
    $0 ~ arg {in_arg=1; next}
    in_arg && /^    '\''[^'\'']+'\'':/ {exit}
    in_arg && /\(default:/ {print; exit}
  ' "$file"
}

echo "[TRON1 真机前安全闸门 / read-only]"
echo "本脚本不启动 ROS、不发布速度、不连接真实 TRON1；只检查当前环境和已运行 graph。"
echo

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
else
  fail "/opt/ros/humble/setup.bash 不存在"
  exit 2
fi

if [ -f "$FOLLOW_WS/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source "$FOLLOW_WS/install/setup.bash"
  set -u
else
  warn "$FOLLOW_WS/install/setup.bash 不存在；FCR 包可能不可见"
fi

if [ -f "$LIMX_WS/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source "$LIMX_WS/install/setup.bash"
  set -u
else
  warn "$LIMX_WS/install/setup.bash 不存在；LimX 官方包可能不可见"
fi

echo "[1/7] Workspace"
echo "PROJECT_ROOT=$PROJECT_ROOT"
echo "ROBOT_TYPE=$ROBOT_TYPE"
echo "RL_TYPE=$RL_TYPE"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-未设置；建议仿真验收使用独立 domain，例如 83}"
git -C "$PROJECT_ROOT" status --short --branch || true
echo

echo "[2/7] 残留进程快照"
process_snapshot="$(pgrep -af 'gazebo|gzserver|gzclient|pointfoot_node|robot_hw_node|tron1_mode_manager_node|tron1_safety_limiter_node|rqt_robot_steering|ros2 topic pub' || true)"
real_tron_processes=""
if [ -z "$process_snapshot" ]; then
  pass "未发现 Gazebo/TRON/limiter/裸 ros2 topic pub 残留进程"
else
  echo "$process_snapshot"
  if echo "$process_snapshot" | grep -Ev 'tron1_safety_acceptance_check|pgrep -af' >/tmp/tron1_relevant_processes.txt; then
    real_tron_processes="$(cat /tmp/tron1_relevant_processes.txt)"
    block "发现可能影响真机测试的残留进程；真机前请先人工确认并关闭"
  else
    pass "进程快照只包含本检查命令自身"
  fi
fi
echo

echo "[3/7] ROS package 可见性"
ros2 pkg prefix robot_platform_pkg >/dev/null 2>&1 \
  && pass "robot_platform_pkg 可见" \
  || fail "robot_platform_pkg 不可见"
ros2 pkg prefix robot_hw >/dev/null 2>&1 \
  && pass "LimX robot_hw 可见" \
  || warn "LimX robot_hw 不可见；无法做官方 launch/订阅检查"
echo

echo "[4/7] 默认 launch/config 安全值"
if ros2 launch bringup_pkg fcr_tron_safe_mode_sim.launch.py --show-args >/tmp/tron1_safe_sim_args.txt 2>&1; then
  grep -A3 "'enable_motion':" /tmp/tron1_safe_sim_args.txt || true
  grep -A3 "'allow_tron_follow_motion':" /tmp/tron1_safe_sim_args.txt || true
  launch_arg_default /tmp/tron1_safe_sim_args.txt enable_motion | grep -q "(default: 'false')" \
    && pass "safe_mode_sim launch 默认 enable_motion=false" \
    || fail "safe_mode_sim launch 默认 enable_motion 不是 false"
  launch_arg_default /tmp/tron1_safe_sim_args.txt allow_tron_follow_motion | grep -q "(default: 'false')" \
    && pass "safe_mode_sim launch 默认 allow_tron_follow_motion=false" \
    || fail "safe_mode_sim launch 默认 allow_tron_follow_motion 不是 false"
else
  fail "无法 inspect fcr_tron_safe_mode_sim.launch.py"
fi

if ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py --show-args >/tmp/tron1_limiter_args.txt 2>&1; then
  grep -E "enable_motion|motion_authorized_timeout_sec|max_linear_x|max_linear_y|max_angular_z" /tmp/tron1_limiter_args.txt || true
  grep -A3 "'enable_motion':" /tmp/tron1_limiter_args.txt | grep -q "(default: 'false')" \
    && pass "limiter launch 默认 enable_motion=false" \
    || fail "limiter launch 默认 enable_motion 不是 false"
  grep -A3 "'max_linear_y':" /tmp/tron1_limiter_args.txt | grep -q "(default: '0.0')" \
    && pass "limiter launch 默认 max_linear_y=0.0" \
    || fail "limiter launch 默认 max_linear_y 不是 0.0"
else
  fail "无法 inspect tron1_safety_limiter.launch.py"
fi

if ros2 launch robot_hw pointfoot_hw_sim.launch.py --show-args >/tmp/tron1_hw_sim_args.txt 2>&1; then
  grep -E "fcr_cmd_vel_topic|start_steering_gui|gui|server" /tmp/tron1_hw_sim_args.txt || true
  grep -A3 "'start_steering_gui':" /tmp/tron1_hw_sim_args.txt | grep -q "(default: 'false')" \
    && pass "官方 sim launch 默认不启动 rqt_robot_steering" \
    || fail "官方 sim launch 默认可能启动 steering GUI"
else
  warn "无法 inspect pointfoot_hw_sim.launch.py"
fi
echo

echo "[5/7] Live ROS graph：/fcr_tron/cmd_vel"
if has_ros_graph_topic /fcr_tron/cmd_vel; then
  cat /tmp/tron1_topic_info.txt
  publisher_names="$(awk '
    /^Publisher count:/ {in_publishers=1; next}
    /^Subscription count:/ {in_publishers=0}
    in_publishers && /Node name:/ {print $3}
  ' /tmp/tron1_topic_info.txt)"
  publisher_count="$(printf "%s\n" "$publisher_names" | sed '/^$/d' | wc -l)"
  if printf "%s\n" "$publisher_names" | grep -qx "tron1_safety_limiter" && [ "$publisher_count" -eq 1 ]; then
    pass "/fcr_tron/cmd_vel 当前只有 tron1_safety_limiter 一个发布者"
  else
    block "/fcr_tron/cmd_vel 当前没有唯一 limiter 发布者；publishers=$publisher_names；若准备真机运动，必须启动 limiter 并复查"
  fi

  subscriber_names="$(awk '
    /^Subscription count:/ {in_subscribers=1; next}
    in_subscribers && /Node name:/ {print $3}
  ' /tmp/tron1_topic_info.txt)"
  if printf "%s\n" "$subscriber_names" | grep -Eq "^(robot_hw_node|cmd_vel_node|pointfoot_node)$"; then
    pass "官方控制器已订阅 /fcr_tron/cmd_vel"
  else
    block "当前 graph 未看到官方控制器订阅 /fcr_tron/cmd_vel；subscribers=$subscriber_names；若准备真机运动，必须先启动官方控制器并复查"
  fi
else
  block "当前没有可检查的 /fcr_tron/cmd_vel graph；这在未启动时正常，但不能算真机前 PASS"
fi
echo

echo "[6/7] 裸 /cmd_vel 风险检查"
if has_ros_graph_topic /cmd_vel; then
  cat /tmp/tron1_topic_info.txt
  if grep -Eq "Node name: (robot_hw_node|cmd_vel_node|pointfoot_node)" /tmp/tron1_topic_info.txt; then
    if [ -z "$real_tron_processes" ]; then
      block "ROS graph 里有官方 TRON 裸 /cmd_vel endpoint，但未发现对应进程；可能是 stale graph。请执行：ros2 daemon stop; ros2 daemon start"
    else
      fail "官方 TRON 控制相关节点出现在裸 /cmd_vel；禁止真机运动"
    fi
  else
    warn "存在 /cmd_vel graph，请确认它不属于 TRON 官方控制器"
  fi
else
  pass "当前未发现裸 /cmd_vel graph"
fi
echo

echo "[7/7] 自动 topic/Gazebo 验收与真机人工门"
if [ -f "$PROJECT_ROOT/docs/TRON1_SAFE_MODE_ACCEPTANCE_2026-09-04.md" ] &&
  grep -q "PASS：47/47" "$PROJECT_ROOT/docs/TRON1_SAFE_MODE_ACCEPTANCE_2026-09-04.md"; then
  pass "已记录 47/47 topic + Gazebo/robot_hw_sim 安全验收"
else
  block "未找到 47/47 topic + Gazebo/robot_hw_sim 安全验收记录"
fi

if [ "${A10_CONFIRMED:-}" = "yes" ]; then
  pass "A-10 人工确认已由环境变量 A10_CONFIRMED=yes 显式给出"
else
  block "A-10 仍需真机前人工确认：物理急停/阻尼可触达，且官方 controller watchdog/进程死亡后果已理解。若已完成，显式设置 A10_CONFIRMED=yes 后复查"
fi
if [ -f "$PROJECT_ROOT/docs/TRON1_OFFICIAL_CONTROLLER_SEMANTICS.md" ]; then
  pass "阶段 B 只读摸底已记录：zero cmd 是 RL policy 零期望速度，不是急停/泄力/阻尼"
else
  block "阶段 B 仍需只读摸底：零 cmd 是平衡/刹停/阻尼/仅零期望值，damping/lock API 是否存在"
fi

echo
echo "Summary: FAIL=$fail_count WARN=$warn_count BLOCK=$block_count"
if [ "$fail_count" -gt 0 ]; then
  echo "结论：FAIL。禁止真机运动。"
  exit 1
fi
if [ "$block_count" -gt 0 ]; then
  echo "结论：BLOCK。代码基础安全验收有进展，但还不是地面真机运动许可。"
  exit 3
fi
echo "结论：PASS。仍建议从架空/支架 enable_motion=false 开始。"
exit 0
