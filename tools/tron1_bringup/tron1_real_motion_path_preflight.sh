#!/usr/bin/env bash
# Read-only preflight for the TRON1 real-motion path.
# This script never publishes velocity commands.
# Exit codes: 0=PASS, 1=FAIL, 2=WARN-only, 3=BLOCK.

set -u

TRON_IP="${TRON_IP:-10.192.1.2}"
JETSON_IP="${JETSON_IP:-172.31.178.242}"
FCR_WS="${FCR_WS:-$HOME/follow_ws}"
LIMX_WS="${LIMX_WS:-$HOME/limx_ws}"
TRON_LINK_IFACE="${TRON_LINK_IFACE:-}"

PASS_COUNT=0
WARN_COUNT=0
BLOCK_COUNT=0
FAIL_COUNT=0

section() {
  printf '\n===== %s =====\n' "$1"
}

run_info() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

mark_pass() {
  PASS_COUNT=$((PASS_COUNT + 1))
  printf '[PASS] %s\n' "$1"
}

mark_warn() {
  WARN_COUNT=$((WARN_COUNT + 1))
  printf '[WARN] %s\n' "$1"
}

mark_block() {
  BLOCK_COUNT=$((BLOCK_COUNT + 1))
  printf '[BLOCK] %s\n' "$1"
}

mark_fail() {
  FAIL_COUNT=$((FAIL_COUNT + 1))
  printf '[FAIL] %s\n' "$1"
}

print_command_output() {
  printf '\n$ %s\n' "$1"
  printf '%s\n' "$2"
}

proxy_or_virtual_route() {
  grep -Eq '(^|[[:space:]])dev[[:space:]](Mihomo|mihomo|Meta|meta|TUN|tun[0-9]*|utun[0-9]*|utun4|tap[0-9]*|wg[0-9]*|tailscale[0-9]*|zt[a-zA-Z0-9]*|docker[0-9]*|br-[a-fA-F0-9]+|veth[a-zA-Z0-9]*)($|[[:space:]])|(^|[[:space:]])table[[:space:]]2022($|[[:space:]])'
}

route_dev_from() {
  awk '
    {
      for (i = 1; i < NF; i++) {
        if ($i == "dev") {
          print $(i + 1)
          exit
        }
      }
    }
  '
}

direct_wired_iface() {
  grep -Eq '^(en|eth|eno|ens|enp|enx|usb)[[:alnum:]_.:-]*$'
}

show_args_default_false() {
  local arg_name="$1"
  awk -v arg="'${arg_name}':" '
    $0 ~ arg { in_arg = 1; next }
    in_arg && /^\ \ \ \ '\''[^'\'']+'\'':/ { exit 1 }
    in_arg && /\(default: '\''false'\''\)/ { found = 1; exit 0 }
    END { if (!found) exit 1 }
  '
}

section "Inputs"
echo "TRON_IP=$TRON_IP"
echo "JETSON_IP=$JETSON_IP"
echo "FCR_WS=$FCR_WS"
echo "LIMX_WS=$LIMX_WS"
if [ -n "$TRON_LINK_IFACE" ]; then
  echo "TRON_LINK_IFACE=$TRON_LINK_IFACE"
else
  echo "TRON_LINK_IFACE=<unset; accepting only wired-looking en*/eth*/eno*/ens*/enp*/enx*/usb* route devices>"
fi

section "Network route checks"
run_info ip -brief addr

route_output="$(ip route get "$TRON_IP" 2>&1)"
route_rc=$?
route_dev="$(printf '%s\n' "$route_output" | route_dev_from)"
print_command_output "ip route get $TRON_IP" "$route_output"
if [ "$route_rc" -ne 0 ]; then
  mark_block "No OS route to TRON_IP; fix wired TRON1/JETSON network before hardware prep."
elif printf '%s\n' "$route_output" | proxy_or_virtual_route; then
  mark_block "TRON_IP route uses a proxy, TUN, container, or policy table path; require direct wired route before hardware prep."
elif [ -z "$route_dev" ]; then
  mark_warn "TRON_IP route exists, but the output does not name an interface; inspect manually before hardware prep."
elif [ -n "$TRON_LINK_IFACE" ] && [ "$route_dev" != "$TRON_LINK_IFACE" ]; then
  mark_block "TRON_IP route uses dev $route_dev, not required TRON_LINK_IFACE=$TRON_LINK_IFACE."
elif [ -z "$TRON_LINK_IFACE" ] && ! printf '%s\n' "$route_dev" | direct_wired_iface; then
  mark_block "TRON_IP route uses dev $route_dev, which is not classified as a direct wired interface; set TRON_LINK_IFACE only after manual verification."
else
  mark_pass "TRON_IP route uses direct wired-looking dev $route_dev."
fi

ping_output="$(ping -c 1 -W 1 "$TRON_IP" 2>&1)"
ping_rc=$?
print_command_output "ping -c 1 -W 1 $TRON_IP" "$ping_output"
if [ "$ping_rc" -eq 0 ]; then
  mark_pass "TRON_IP responds to one read-only ping."
else
  mark_block "TRON_IP did not respond to a read-only ping; do not continue toward hardware prep."
fi

section "ROS environment"
set +u
if [ -f /opt/ros/humble/setup.bash ]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
fi
if [ -f "$FCR_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$FCR_WS/install/setup.bash"
fi
if [ -f "$LIMX_WS/install/setup.bash" ]; then
  # shellcheck source=/dev/null
  source "$LIMX_WS/install/setup.bash"
fi
set -u

run_info ros2 pkg executables robot_platform_pkg
run_info ros2 pkg executables robot_hw

section "Safe launch defaults"
export ROBOT_TYPE="${ROBOT_TYPE:-WF_TRON1A}"
export RL_TYPE="${RL_TYPE:-isaacgym}"
fcr_launch_output="$(ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py --show-args 2>&1)"
fcr_launch_rc=$?
print_command_output "ros2 launch bringup_pkg fcr_tron_jetson_comm.launch.py --show-args" "$fcr_launch_output"
if [ "$fcr_launch_rc" -ne 0 ]; then
  mark_block "Cannot inspect FCR/TRON launch defaults; keep real motion paused."
else
  printf '%s\n' "$fcr_launch_output" | show_args_default_false "start_tron_hw" \
    && mark_pass "FCR/TRON launch default keeps start_tron_hw=false." \
    || mark_fail "FCR/TRON launch default does not clearly keep start_tron_hw=false."
  printf '%s\n' "$fcr_launch_output" | show_args_default_false "enable_motion" \
    && mark_pass "FCR/TRON launch default keeps enable_motion=false." \
    || mark_fail "FCR/TRON launch default does not clearly keep enable_motion=false."
fi

robot_hw_output="$(ros2 launch robot_hw pointfoot_hw.launch.py --show-args 2>&1)"
robot_hw_rc=$?
print_command_output "ros2 launch robot_hw pointfoot_hw.launch.py --show-args" "$robot_hw_output"
if [ "$robot_hw_rc" -ne 0 ]; then
  mark_warn "Cannot inspect official robot_hw launch defaults; rely on FCR launch override and manual checklist."
elif printf '%s\n' "$robot_hw_output" | grep -Eq '/cmd_vel'; then
  mark_warn "Official robot_hw launch exposes /cmd_vel as a default; real bringup must use the FCR launch override to /fcr_tron/cmd_vel."
else
  mark_pass "Official robot_hw launch defaults do not visibly advertise /cmd_vel."
fi

section "If graph is already running"
run_info ros2 topic info -v /fcr_tron/cmd_vel
run_info ros2 topic info -v /fcr/cmd_vel_stamped
run_info ros2 topic info -v /cmd_vel

section "Result guide"
echo "PASS for low-speed prep requires:"
echo "  1) TRON_IP route uses a direct wired interface; set TRON_LINK_IFACE=<iface> to require an exact device."
echo "  2) non-motion network checks stop at route/ping; do not press L1+Y/triangle or activate the official controller."
echo "  3) fcr_tron_jetson_comm.launch.py keeps start_tron_hw=false and enable_motion=false by default."
echo "  4) /fcr_tron/cmd_vel has exactly one publisher: tron1_safety_limiter."
echo "  5) /cmd_vel has no TRON controller subscriber during FCR tests."
echo
echo "Exit codes: 0=PASS, 1=FAIL, 2=WARN-only, 3=BLOCK."
echo
echo "This preflight is read-only: no real motion command was sent."
echo
echo "Summary: PASS=$PASS_COUNT WARN=$WARN_COUNT BLOCK=$BLOCK_COUNT FAIL=$FAIL_COUNT"

if [ "$FAIL_COUNT" -gt 0 ]; then
  exit 1
fi
if [ "$BLOCK_COUNT" -gt 0 ]; then
  exit 3
fi
if [ "$WARN_COUNT" -gt 0 ]; then
  exit 2
fi
exit 0
