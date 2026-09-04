#!/usr/bin/env bash
# TRON1 Gazebo 练习脚本（仅仿真，不连接真机）
#
# 一键启动：
#   1. 官方 Gazebo 仿真控制器 (pointfoot_hw_sim)
#   2. FCR 安全栈 (mode_manager + safety limiter)
#   3. /safety/estop_state=false 心跳
#   4. 把状态机推进到 TRON_FOLLOW
#   5. 打开中文控制台，留给你练键位
#
# 用法：bash tools/tron1_bringup/tron1_practice_gazebo.sh
# 退出：在控制台输入“退出”，或 Ctrl-C

set -euo pipefail

FOLLOW_WS="/home/miya/follow_ws"
LIMX_WS="/home/miya/limx_ws"
LOG_DIR="/tmp/tron1_practice"
TRON_IP="${TRON_IP:-10.192.1.2}"

export ROBOT_TYPE="${ROBOT_TYPE:-WF_TRON1A}"
export RL_TYPE="${RL_TYPE:-isaacgym}"
export FCR_TRON_CMD_VEL_TOPIC="${FCR_TRON_CMD_VEL_TOPIC:-/fcr_tron/cmd_vel}"
export FCR_TRON_CMD_VEL_TIMEOUT_SEC="${FCR_TRON_CMD_VEL_TIMEOUT_SEC:-0.25}"
# 练习专用 domain（90），避免和自动验收(83)/真机默认(0)混跑
if [ "${ROS_DOMAIN_ID+x}" != "x" ]; then
  export ROS_DOMAIN_ID="90"
else
  export ROS_DOMAIN_ID
fi

die() {
  echo "[练习脚本][FAIL] $*" >&2
  exit 4
}

validate_ros_domain_id() {
  if [[ ! "$ROS_DOMAIN_ID" =~ ^[0-9]+$ ]]; then
    die "ROS_DOMAIN_ID 必须是 1..232 的十进制整数，当前值：$ROS_DOMAIN_ID"
  fi
  if [[ "$ROS_DOMAIN_ID" =~ ^0[0-9]+$ ]]; then
    die "ROS_DOMAIN_ID 不允许前导零，当前值：$ROS_DOMAIN_ID"
  fi
  local domain_num
  domain_num=$((10#$ROS_DOMAIN_ID))
  if (( domain_num < 1 || domain_num > 232 )); then
    die "ROS_DOMAIN_ID 必须是 1..232；拒绝空值、0 或默认真机 domain。当前值：$ROS_DOMAIN_ID"
  fi
}

real_robot_guard() {
  local risky
  risky="$(pgrep -af 'pointfoot_node|/robot_hw_node|robot_hw_node|pointfoot_hw.launch.py' \
    | grep -Ev 'tron1_practice_gazebo|pgrep -af' || true)"
  if [ -n "$risky" ]; then
    echo "$risky" >&2
    die "发现可能连接真机的 TRON1/robot_hw 进程；拒绝启动仿真练习"
  fi
  if ping -c 1 -W 1 "$TRON_IP" >/dev/null 2>&1; then
    die "检测到 TRON1 地址 $TRON_IP 可达；拒绝启动 enable_motion=true 的练习脚本"
  fi
}

ros_daemon_is_running() {
  command -v ros2 >/dev/null 2>&1 &&
    ros2 daemon status 2>/dev/null | grep -q "^The daemon is running$"
}

prelaunch_graph_guard() {
  local info_file="$LOG_DIR/prelaunch_cmd_vel_topic_info.txt"
  if timeout 3s ros2 topic info /fcr_tron/cmd_vel -v >"$info_file" 2>&1; then
    if grep -q "Node name:" "$info_file"; then
      cat "$info_file" >&2
      die "启动前 /fcr_tron/cmd_vel 已有 graph endpoint；请清理残留 graph 后重试"
    fi
  elif ! grep -q "Unknown topic" "$info_file"; then
    cat "$info_file" >&2
    die "启动前 ROS graph 预扫描失败；不能默认为安全"
  fi
}

gazebo_graph_guard() {
  local nodes_file="$LOG_DIR/ros_nodes.txt"
  local gazebo_ready=0
  for _ in $(seq 1 60); do
    if timeout 3s ros2 node list >"$nodes_file" 2>&1 &&
       grep -Eq '^/gazebo$|^gazebo$' "$nodes_file"; then
      gazebo_ready=1
      break
    fi
    sleep 1
  done
  if [ "$gazebo_ready" -ne 1 ]; then
    cat "$nodes_file" >&2
    die "60 秒内未看到 /gazebo 节点；拒绝把 robot_hw_node 当作仿真证据"
  fi

  local info_file="$LOG_DIR/cmd_vel_topic_info.txt"
  timeout 3s ros2 topic info /fcr_tron/cmd_vel -v >"$info_file" 2>&1 || {
    cat "$info_file" >&2
    die "无法检查 /fcr_tron/cmd_vel graph"
  }
  if grep -Eq "Node name: (pointfoot_node|cmd_vel_node)" "$info_file"; then
    cat "$info_file" >&2
    die "发现非本 Gazebo 练习允许的 /fcr_tron/cmd_vel 订阅者"
  fi
}

wait_for_mode_state() {
  local expected="$1"
  local state_file="$LOG_DIR/mode_state.txt"
  for _ in $(seq 1 10); do
    if timeout 3s ros2 topic echo --once /tron1/mode_state >"$state_file" 2>&1 &&
       grep -q "data: $expected" "$state_file"; then
      return 0
    fi
    sleep 1
  done
  cat "$state_file" >&2
  return 1
}

validate_ros_domain_id

set +u
source /opt/ros/humble/setup.bash
set -u
if [ -f "$FOLLOW_WS/install/setup.bash" ]; then
  set +u
  source "$FOLLOW_WS/install/setup.bash"
  set -u
fi
if [ -f "$LIMX_WS/install/setup.bash" ]; then
  set +u
  source "$LIMX_WS/install/setup.bash"
  set -u
fi

mkdir -p "$LOG_DIR"
ROS_DAEMON_WAS_RUNNING=0
if ros_daemon_is_running; then
  ROS_DAEMON_WAS_RUNNING=1
fi
real_robot_guard
prelaunch_graph_guard

echo "== TRON1 Gazebo 练习（仿真） =="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "日志目录：$LOG_DIR"
echo

CLEANED_UP=0
PIDS=()
start_background() {
  local log_file="$1"
  shift
  setsid "$@" >"$log_file" 2>&1 </dev/null &
  PIDS+=($!)
}

cleanup() {
  if [ "$CLEANED_UP" -eq 1 ]; then
    return
  fi
  CLEANED_UP=1
  echo
  echo "[练习脚本] 正在关闭后台进程..."
  for pid in "${PIDS[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${PIDS[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
  done
  if [ "$ROS_DAEMON_WAS_RUNNING" -eq 0 ] && command -v ros2 >/dev/null 2>&1; then
    ROS_DOMAIN_ID="$ROS_DOMAIN_ID" ros2 daemon stop >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT
trap 'cleanup; exit 130' INT
trap 'cleanup; exit 143' TERM

# 1. 官方 Gazebo 仿真控制器
start_background "$LOG_DIR/gazebo.log" \
  ros2 launch robot_hw pointfoot_hw_sim.launch.py \
  use_gazebo:=true \
  fcr_cmd_vel_topic:=/fcr_tron/cmd_vel \
  start_steering_gui:=false

# 2. FCR 安全栈
start_background "$LOG_DIR/safe_stack.log" \
  ros2 launch bringup_pkg fcr_tron_safe_mode_sim.launch.py \
  enable_motion:=true \
  allow_tron_follow_motion:=true \
  max_linear_x:=0.03 \
  max_angular_z:=0.10

# 3. 等 graph 就绪
echo "[练习脚本] 等待 /fcr_tron/cmd_vel 与 /tron1/mode_state 就绪..."
ready=0
for _ in $(seq 1 90); do
  if timeout 3s ros2 topic info /fcr_tron/cmd_vel >/dev/null 2>&1 && \
     timeout 3s ros2 topic info /tron1/mode_state >/dev/null 2>&1; then
    ready=1
    break
  fi
  sleep 1
done
if [ "$ready" -ne 1 ]; then
  echo "[练习脚本] 启动超时。请查看日志：$LOG_DIR/gazebo.log 和 $LOG_DIR/safe_stack.log"
  exit 1
fi
echo "[练习脚本] graph 就绪。"
gazebo_graph_guard

# 4. estop=false 心跳（后台持续，别关）
ros2 topic pub --rate 50 /safety/estop_state std_msgs/msg/Bool "{data: false}" \
  >"$LOG_DIR/estop_heartbeat.log" 2>&1 &
PIDS+=($!)
sleep 1

# 5. 推进状态机到 TRON_FOLLOW
echo "[练习脚本] 推进状态机到 TRON_FOLLOW..."
for req in developer_mode developer_self_check_pass stand_ready walk_ready \
           device_self_check_pass gimbal_follow tron_follow; do
  ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: $req}" \
    >/dev/null 2>&1
  sleep 0.2
done
if wait_for_mode_state "TRON_FOLLOW"; then
  echo "[练习脚本] 已确认进入 TRON_FOLLOW。"
else
  die "推进状态机后未确认进入 TRON_FOLLOW；请重试或查看日志"
fi
echo

echo "=============================================="
echo " 中文控制台已就绪。可输入："
echo "   直走 / 后退 / 左转 / 右转"
echo "   停        -> 持续发 0 速度"
echo "   急停      -> 输出立即归零"
echo "   解除急停  -> 清除急停锁存"
echo "   帮助 / 退出"
echo "=============================================="
echo

# 6. 中文控制台（前台交互；退出后自动清理后台进程）
ros2 run teleop_control_pkg tron1_chinese_teleop
