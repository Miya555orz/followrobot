#!/usr/bin/env bash
# Jetson-side TRON1 Ethernet route setup helper.
# Default mode is dry-run. With --apply, this script only configures IP link,
# address, and route; it never starts ROS, robot_hw, controllers, or publishers.

set -u

TRON_IFACE="${TRON_IFACE:-${TRON_LINK_IFACE:-}}"
TRON_IP="${TRON_IP:-10.192.1.2}"
TRON_NET="${TRON_NET:-10.192.1.0/24}"
JETSON_TRON_ADDR="${JETSON_TRON_ADDR:-10.192.1.200}"
CONFIRM_TRON1_ROUTE_SETUP="${CONFIRM_TRON1_ROUTE_SETUP:-no}"
APPLY="false"
PING_AFTER="${PING_AFTER:-true}"

usage() {
  cat <<EOF
Usage:
  TRON_IFACE=<Jetson TRON1 Ethernet iface> $0 [--dry-run|--apply]

Environment:
  TRON_IFACE          Required unless TRON_LINK_IFACE is set.
  TRON_LINK_IFACE     Alias for TRON_IFACE.
  TRON_IP             Default: 10.192.1.2
  TRON_NET            Default: 10.192.1.0/24
  JETSON_TRON_ADDR    Default: 10.192.1.200
  CONFIRM_TRON1_ROUTE_SETUP
                      Required as "yes" with --apply.
  PING_AFTER          Default: true

This helper configures only network link/address/route state. It does not source
ROS, launch robot_hw, activate a controller, or publish velocity commands.
EOF
}

for arg in "$@"; do
  case "$arg" in
    --apply)
      APPLY="true"
      ;;
    --dry-run)
      APPLY="false"
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $arg"
      usage
      exit 2
      ;;
  esac
done

section() {
  printf '\n===== %s =====\n' "$1"
}

run_or_print() {
  if [ "$APPLY" = "true" ]; then
    printf '\n$ %s\n' "$*"
    "$@"
  else
    printf '%q' "$1"
    shift
    for arg in "$@"; do
      printf ' %q' "$arg"
    done
    printf '\n'
  fi
}

run_apply_or_fail() {
  printf '\n$ %s\n' "$*"
  if "$@"; then
    return 0
  fi
  echo "[FAIL] Command failed: $*"
  exit 1
}

addr_present() {
  ip -4 addr show dev "$TRON_IFACE" | grep -Eq "(^|[[:space:]])${JETSON_TRON_ADDR}/24([[:space:]]|$)"
}

addr_owner_iface() {
  ip -o -4 addr show | awk -v cidr="${JETSON_TRON_ADDR}/24" '$4 == cidr { print $2; exit }'
}

route_field_after() {
  awk -v key="$1" '
    {
      for (i = 1; i < NF; i++) {
        if ($i == key) {
          print $(i + 1)
          exit
        }
      }
    }
  '
}

proxy_or_virtual_iface() {
  grep -Eiq '^(lo|l4tbr[0-9]*|Mihomo|Meta|TUN|tun[[:alnum:]_.:-]*|utun[[:alnum:]_.:-]*|tap[[:alnum:]_.:-]*|wg[[:alnum:]_.:-]*|tailscale[[:alnum:]_.:-]*|zt[a-zA-Z0-9_.:-]*|docker[[:alnum:]_.:-]*|br-[a-fA-F0-9]+|veth[a-zA-Z0-9_.:-]*)$'
}

direct_wired_iface() {
  grep -Eq '^(en|eth|eno|ens|enp|enx|end)[[:alnum:]_.:-]*$'
}

section "Inputs"
echo "Run this on the Jetson host whose Ethernet cable is connected to TRON1."
echo "APPLY=$APPLY"
echo "TRON_IFACE=${TRON_IFACE:-<unset>}"
echo "TRON_IP=$TRON_IP"
echo "TRON_NET=$TRON_NET"
echo "JETSON_TRON_ADDR=$JETSON_TRON_ADDR"
echo "CONFIRM_TRON1_ROUTE_SETUP=$CONFIRM_TRON1_ROUTE_SETUP"

if [ -z "$TRON_IFACE" ]; then
  echo "[BLOCK] TRON_IFACE is required. Example: TRON_IFACE=enP8p1s0 $0 --apply"
  exit 3
fi

if printf '%s\n' "$TRON_IFACE" | proxy_or_virtual_iface; then
  echo "[BLOCK] Refusing virtual/proxy/bridge interface: $TRON_IFACE"
  exit 3
fi

if ! printf '%s\n' "$TRON_IFACE" | direct_wired_iface; then
  echo "[BLOCK] Refusing non-wired-looking interface: $TRON_IFACE"
  exit 3
fi

if ! ip link show "$TRON_IFACE" >/dev/null 2>&1; then
  echo "[FAIL] Interface does not exist on this host: $TRON_IFACE"
  exit 1
fi

section "Current devices"
ip -brief addr
ip -brief link

if [ "$APPLY" != "true" ]; then
  section "Dry-run commands"
  echo "No changes were made. Re-run with --apply to execute:"
elif [ "$CONFIRM_TRON1_ROUTE_SETUP" != "yes" ]; then
  echo "[BLOCK] --apply requires CONFIRM_TRON1_ROUTE_SETUP=yes after reviewing --dry-run output."
  exit 3
fi

if [ "$APPLY" = "true" ]; then
  run_apply_or_fail sudo ip link set "$TRON_IFACE" up
else
  run_or_print sudo ip link set "$TRON_IFACE" up
fi

addr_owner="$(addr_owner_iface)"
if [ -n "$addr_owner" ] && [ "$addr_owner" != "$TRON_IFACE" ]; then
  if [ "$APPLY" = "true" ]; then
    echo "[FAIL] ${JETSON_TRON_ADDR}/24 already exists on $addr_owner, not $TRON_IFACE."
    exit 1
  else
    echo
    echo "[INFO] ${JETSON_TRON_ADDR}/24 already exists on $addr_owner; --apply will fail until the address conflict is resolved."
  fi
elif addr_present; then
  echo
  if [ "$APPLY" = "true" ]; then
    echo "[INFO] ${JETSON_TRON_ADDR}/24 already exists on $TRON_IFACE; keeping it."
  else
    echo "[INFO] ${JETSON_TRON_ADDR}/24 already exists on $TRON_IFACE; --apply will skip the addr add command."
  fi
else
  if [ "$APPLY" = "true" ]; then
    run_apply_or_fail sudo ip addr add "${JETSON_TRON_ADDR}/24" dev "$TRON_IFACE"
  else
    run_or_print sudo ip addr add "${JETSON_TRON_ADDR}/24" dev "$TRON_IFACE"
  fi
fi

if [ "$APPLY" = "true" ]; then
  run_apply_or_fail sudo ip route replace "$TRON_NET" dev "$TRON_IFACE" src "$JETSON_TRON_ADDR" metric 10
else
  run_or_print sudo ip route replace "$TRON_NET" dev "$TRON_IFACE" src "$JETSON_TRON_ADDR" metric 10
fi

if [ "$APPLY" != "true" ]; then
  section "Next"
  echo "After reviewing the commands, run:"
  echo "  CONFIRM_TRON1_ROUTE_SETUP=yes TRON_IFACE=$TRON_IFACE $0 --apply"
  echo "Then validate:"
  echo "  TRON_LINK_IFACE=$TRON_IFACE ./tools/tron1_bringup/jetson_tron1_network_preflight.sh"
  exit 0
fi

section "Route check"
route_output="$(ip route get "$TRON_IP" 2>&1)"
route_rc=$?
printf '%s\n' "$route_output"
route_dev="$(printf '%s\n' "$route_output" | route_field_after dev)"
route_src="$(printf '%s\n' "$route_output" | route_field_after src)"

if [ "$route_rc" -ne 0 ]; then
  echo "[BLOCK] No OS route to TRON_IP after setup."
  exit 3
fi
if [ "$route_dev" != "$TRON_IFACE" ]; then
  echo "[BLOCK] Route uses dev ${route_dev:-<unknown>}, expected $TRON_IFACE."
  exit 3
fi
if [ "$route_src" != "$JETSON_TRON_ADDR" ]; then
  echo "[BLOCK] Route uses src ${route_src:-<unknown>}, expected $JETSON_TRON_ADDR."
  exit 3
fi
echo "[PASS] Route uses dev $TRON_IFACE src $JETSON_TRON_ADDR."

if [ "$PING_AFTER" = "true" ]; then
  section "Ping check"
  if ping -c 1 -W 1 "$TRON_IP"; then
    echo "[PASS] TRON_IP responded to one read-only ping."
  else
    echo "[BLOCK] TRON_IP did not respond to one read-only ping."
    exit 3
  fi
fi

section "Done"
echo "Network route configured. This was network-only; no ROS/controller/velocity command was used."
