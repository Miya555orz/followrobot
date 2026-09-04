#!/usr/bin/env bash
# TRON1 Gazebo 键盘控制台练习脚本（仅仿真，不连接真机）
#
# 一键启动与真机完全一致的链路：
#   fcr_mode_console -> command_mux -> /fcr/cmd_vel_stamped
#                     -> safety limiter -> /fcr_tron/cmd_vel -> Gazebo
#
# 用法：bash tools/tron1_bringup/tron1_practice_keyboard_gazebo.sh
# 退出：按 Ctrl-C（或先按 1 待机再 Ctrl-C）

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
  if (( ROS_DOMAIN_ID < 1 || ROS_DOMAIN_ID > 232 )); then
    die "ROS_DOMAIN_ID 必须是 1..232；拒绝空值、0 或默认真机 domain。当前值：$ROS_DOMAIN_ID"
  fi
}

real_robot_guard() {
  local risky
  risky="$(pgrep -af 'pointfoot_node|/robot_hw_node|robot_hw_node|pointfoot_hw.launch.py' \
    | grep -Ev 'tron1_practice_keyboard_gazebo|pgrep -af' || true)"
  if [ -n "$risky" ]; then
    echo "$risky" >&2
    die "发现可能连接真机的 TRON1/robot_hw 进程；拒绝启动仿真练习"
  fi
  if ping -c 1 -W 1 "$TRON_IP" >/dev/null 2>&1; then
    die "检测到 TRON1 地址 $TRON_IP 可达；拒绝启动 enable_motion=true 的练习脚本"
  fi
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
  timeout 3s ros2 node list >"$nodes_file" 2>&1 || die "无法读取 ROS node list"
  grep -Eq '^/gazebo$|^gazebo$' "$nodes_file" || {
    cat "$nodes_file" >&2
    die "未看到 /gazebo 节点；拒绝把 robot_hw_node 当作仿真证据"
  }

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
real_robot_guard
prelaunch_graph_guard

echo "== TRON1 Gazebo 键盘控制台练习（仿真） =="
echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "日志目录：$LOG_DIR"
echo

PIDS=()
cleanup() {
  echo
  echo "[练习脚本] 正在关闭后台进程..."
  for pid in "${PIDS[@]:-}"; do
    kill "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# 1. 官方 Gazebo 仿真控制器
ros2 launch robot_hw pointfoot_hw_sim.launch.py \
  use_gazebo:=true \
  fcr_cmd_vel_topic:=/fcr_tron/cmd_vel \
  start_steering_gui:=false >"$LOG_DIR/gazebo.log" 2>&1 </dev/null &
PIDS+=($!)

# 2. FCR 完整链路（command_mux + mode_manager + limiter），关掉相机/深度
ros2 launch bringup_pkg fcr_tron_full_follow.launch.py \
  enable_motion:=true \
  allow_tron_follow_motion:=true \
  start_gemini:=false \
  enable_depth_fusion:=false \
  max_linear_x:=0.15 \
  max_angular_z:=0.30 >"$LOG_DIR/fcr_chain.log" 2>&1 </dev/null &
PIDS+=($!)

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
  echo "[练习脚本] 启动超时。请查看日志：$LOG_DIR/gazebo.log 和 $LOG_DIR/fcr_chain.log"
  exit 1
fi
echo "[练习脚本] graph 就绪。"
gazebo_graph_guard

# 4. 推进状态机到 TRON_FOLLOW
echo "[练习脚本] 推进状态机到 TRON_FOLLOW..."
for req in developer_mode developer_self_check_pass stand_ready walk_ready \
           device_self_check_pass gimbal_follow tron_follow; do
  ros2 topic pub --once /tron1/mode_request std_msgs/msg/String "{data: $req}" \
    >/dev/null 2>&1
  sleep 0.2
done
echo "[练习脚本] 已进入 TRON_FOLLOW。"
echo

echo "=============================================="
echo " 键盘控制台已就绪，按键如下："
echo "   5       进入连续遥控（先按这个）"
echo "   W / S   前进 / 后退"
echo "   Q / E   左转 / 右转"
echo "   空格    停车（释放按键也会自动停）"
echo "   X       软件急停（会锁死三层，见下）"
echo "   C       清除 command_mux 急停"
echo "   1       待机 / 停止"
echo "   H       帮助"
echo "   Ctrl-C  退出整套"
echo ""
echo " 注意：按 X 急停后，状态机被锁进 ESTOP，"
echo " 只按 C 不能恢复运动；恢复需重新推进状态机，"
echo " 最简单：Ctrl-C 退出后重新运行本脚本。"
echo "=============================================="
echo

# 6. 键盘控制台（前台交互；退出后自动清理后台进程）
ros2 run teleop_control_pkg fcr_mode_console --ros-args -p linear_speed:=0.12 -p yaw_rate:=0.40
