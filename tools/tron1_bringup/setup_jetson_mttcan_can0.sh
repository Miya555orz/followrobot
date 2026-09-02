#!/usr/bin/env bash
# Configure Jetson Orin Nano on-board mttcan can0 for DJI RS2 bring-up.
# This script only brings the CAN interface up. It does not send any gimbal
# command and does not touch TRON1 motion.

set -euo pipefail

CAN_IF="${JETSON_CAN_INTERFACE:-can0}"
BITRATE="${JETSON_CAN_BITRATE:-1000000}"
RESTART_MS="${JETSON_CAN_RESTART_MS:-100}"
TX_QUEUE_LEN="${JETSON_CAN_TX_QUEUE_LEN:-100}"

usage() {
  cat <<'EOF'
Usage: ./setup_jetson_mttcan_can0.sh [--interface canX]

Defaults:
  interface   can0
  bitrate     1000000
  restart-ms  100
  txqueuelen  100

Environment overrides:
  JETSON_CAN_INTERFACE
  JETSON_CAN_BITRATE
  JETSON_CAN_RESTART_MS
  JETSON_CAN_TX_QUEUE_LEN
EOF
}

while (($# > 0)); do
  case "$1" in
    --interface)
      [[ $# -ge 2 ]] || { echo "ERROR: --interface requires a value" >&2; exit 2; }
      CAN_IF="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ${EUID} -ne 0 ]]; then
  exec sudo --preserve-env=JETSON_CAN_INTERFACE,JETSON_CAN_BITRATE,JETSON_CAN_RESTART_MS,JETSON_CAN_TX_QUEUE_LEN bash \
    "$0" --interface "$CAN_IF"
fi

for command_name in ip modprobe readlink basename; do
  command -v "$command_name" >/dev/null 2>&1 || {
    echo "ERROR: required command not found: $command_name" >&2
    exit 1
  }
done

[[ -d "/sys/class/net/${CAN_IF}" ]] || {
  echo "ERROR: interface does not exist: ${CAN_IF}" >&2
  echo "Check available CAN interfaces with: ip -br link | grep can" >&2
  exit 1
}

driver_path="$(readlink -f "/sys/class/net/${CAN_IF}/device/driver" 2>/dev/null || true)"
driver_name=""
if [[ -n "$driver_path" ]]; then
  driver_name="$(basename "$driver_path")"
fi

if [[ "$driver_name" != "mttcan" ]]; then
  echo "WARN: ${CAN_IF} driver is '${driver_name:-unknown}', not 'mttcan'." >&2
  echo "WARN: continuing because the interface was explicitly selected." >&2
fi

modprobe can
modprobe can_raw
if ! modprobe mttcan 2>/dev/null; then
  echo "INFO: mttcan module not loaded by modprobe; it may be built into the Jetson kernel."
fi

echo "Configuring ${CAN_IF}: bitrate=${BITRATE}, restart-ms=${RESTART_MS}, txqueuelen=${TX_QUEUE_LEN}"
ip link set "$CAN_IF" down || true
ip link set "$CAN_IF" type can bitrate "$BITRATE" restart-ms "$RESTART_MS"
ip link set "$CAN_IF" txqueuelen "$TX_QUEUE_LEN"
ip link set "$CAN_IF" up

echo
ip -details -statistics link show "$CAN_IF"
echo
echo "JETSON_CAN_INTERFACE=${CAN_IF}"
echo "Use this launch argument when starting the RS2 driver:"
echo "  can_interface:=${CAN_IF}"
