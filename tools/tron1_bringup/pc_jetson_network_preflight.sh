#!/usr/bin/env bash
# PC-side preflight for reaching the Jetson/TRON1 bring-up network.
#
# Default mode is read-only. Use --fix-route only after the Ethernet link has
# carrier and the main routing table can reach the Jetson.

set -u

JETSON_IP="${JETSON_IP:-172.31.178.242}"
ETH_IF="${ETH_IF:-enp0s31f6}"
FIX_ROUTE="false"

if [ "${1:-}" = "--fix-route" ]; then
  FIX_ROUTE="true"
fi

section() {
  printf '\n===== %s =====\n' "$1"
}

run() {
  printf '\n$ %s\n' "$*"
  "$@" 2>&1 || true
}

section "Inputs"
echo "JETSON_IP=$JETSON_IP"
echo "ETH_IF=$ETH_IF"
echo "FIX_ROUTE=$FIX_ROUTE"

section "Network devices"
run ip -brief addr
run nmcli device status

section "Ethernet carrier"
if [ -e "/sys/class/net/$ETH_IF/operstate" ]; then
  echo "$ETH_IF operstate: $(cat "/sys/class/net/$ETH_IF/operstate" 2>/dev/null || echo unknown)"
fi
if [ -e "/sys/class/net/$ETH_IF/carrier" ]; then
  echo "$ETH_IF carrier: $(cat "/sys/class/net/$ETH_IF/carrier" 2>/dev/null || echo unknown)"
fi

section "Routes"
run ip route get "$JETSON_IP"
run ip rule show
run ip route show table main
run ip route show table 2022

section "SSH probe"
run ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new \
  "miya@$JETSON_IP" 'hostname; date; echo SSH_OK'

if [ "$FIX_ROUTE" != "true" ]; then
  section "Read-only result"
  echo "No changes were made."
  echo "If Ethernet has carrier but ip route get $JETSON_IP still goes through Mihomo/TUN,"
  echo "run this temporary route bypass:"
  echo "  JETSON_IP=$JETSON_IP ETH_IF=$ETH_IF $0 --fix-route"
  exit 0
fi

section "Temporary route bypass"
if ! ip link show "$ETH_IF" >/dev/null 2>&1; then
  echo "Cannot fix route: interface does not exist: $ETH_IF"
  exit 1
fi

if [ -e "/sys/class/net/$ETH_IF/carrier" ] && [ "$(cat "/sys/class/net/$ETH_IF/carrier" 2>/dev/null || echo 0)" != "1" ]; then
  echo "Cannot fix route safely: $ETH_IF has no carrier. Check cable/switch/Jetson power first."
  exit 1
fi

echo "Adding temporary high-priority policy rule so $JETSON_IP uses the main table before Mihomo."
echo "This does not edit NetworkManager config and will disappear after reboot."
sudo ip rule add pref 100 to "$JETSON_IP/32" lookup main 2>/dev/null || true
run ip route get "$JETSON_IP"

section "SSH probe after route bypass"
run ssh -o BatchMode=yes -o ConnectTimeout=5 -o StrictHostKeyChecking=accept-new \
  "miya@$JETSON_IP" 'hostname; date; echo SSH_OK'

