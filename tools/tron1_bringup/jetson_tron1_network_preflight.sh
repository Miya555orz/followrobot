#!/usr/bin/env bash
# Jetson-side read-only network preflight for TRON1.
# This script never starts ROS, never launches robot_hw, and never publishes velocity.
# Exit codes: 0=PASS, 1=FAIL, 2=WARN-only, 3=BLOCK.

set -u

TRON_IP="${TRON_IP:-10.192.1.2}"
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
  grep -Eq '(^|[[:space:]])dev[[:space:]](Mihomo|mihomo|Meta|meta|TUN|tun[[:alnum:]_.:-]*|utun[[:alnum:]_.:-]*|tap[[:alnum:]_.:-]*|wg[[:alnum:]_.:-]*|tailscale[[:alnum:]_.:-]*|zt[a-zA-Z0-9_.:-]*|docker[[:alnum:]_.:-]*|br-[a-fA-F0-9]+|veth[a-zA-Z0-9_.:-]*)($|[[:space:]])|(^|[[:space:]])table[[:space:]]2022($|[[:space:]])'
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
  grep -Eq '^(en|eth|eno|ens|enp|enx|end|usb)[[:alnum:]_.:-]*$'
}

section "Inputs"
echo "Run this on the Jetson host wired to TRON1; do not run it from the PC unless the PC is intentionally the wired TRON1 endpoint."
echo "TRON_IP=$TRON_IP"
if [ -n "$TRON_LINK_IFACE" ]; then
  echo "TRON_LINK_IFACE=$TRON_LINK_IFACE"
else
  echo "TRON_LINK_IFACE=<unset; accepting only wired-looking en*/eth*/eno*/ens*/enp*/enx*/end*/usb* route devices>"
fi

section "Network devices"
run_info ip -brief addr
run_info ip -brief link

section "TRON1 route"
route_output="$(ip route get "$TRON_IP" 2>&1)"
route_rc=$?
route_dev="$(printf '%s\n' "$route_output" | route_dev_from)"
print_command_output "ip route get $TRON_IP" "$route_output"

if [ "$route_rc" -ne 0 ]; then
  mark_block "No OS route to TRON_IP; configure the Jetson wired interface before continuing."
elif printf '%s\n' "$route_output" | proxy_or_virtual_route; then
  mark_block "TRON_IP route uses a proxy, TUN, container, or policy table path; require direct Jetson-to-TRON1 Ethernet."
elif [ -z "$route_dev" ]; then
  mark_warn "TRON_IP route exists, but the output does not name an interface; inspect manually before continuing."
elif [ -n "$TRON_LINK_IFACE" ] && [ "$route_dev" != "$TRON_LINK_IFACE" ]; then
  mark_block "TRON_IP route uses dev $route_dev, not required TRON_LINK_IFACE=$TRON_LINK_IFACE."
elif [ -z "$TRON_LINK_IFACE" ] && ! printf '%s\n' "$route_dev" | direct_wired_iface; then
  mark_block "TRON_IP route uses dev $route_dev, which is not classified as a direct wired interface."
else
  mark_pass "TRON_IP route uses direct wired-looking dev $route_dev."
fi

section "TRON1 ping"
ping_output="$(ping -c 1 -W 1 "$TRON_IP" 2>&1)"
ping_rc=$?
print_command_output "ping -c 1 -W 1 $TRON_IP" "$ping_output"

if [ "$ping_rc" -eq 0 ]; then
  mark_pass "TRON_IP responds to one read-only ping."
else
  mark_block "TRON_IP did not respond to a read-only ping; do not continue toward controller bringup."
fi

section "Safety reminder"
echo "This preflight is network-only: no ROS graph, no robot_hw, no controller activation, no velocity command."
echo "Do not press L1+Y/triangle during this check."
echo "Exit codes: 0=PASS, 1=FAIL, 2=WARN-only, 3=BLOCK."
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
