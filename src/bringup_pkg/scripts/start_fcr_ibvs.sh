#!/usr/bin/env bash
set -Eeuo pipefail

# One-command real-hardware bringup for the FCR IBVS follow stack.
#
# The script deliberately leaves command_mux in its configured default
# "manual" mode. The perception and servo nodes may produce /auto commands,
# but the operator must explicitly publish "auto" before actuators follow them.

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WORKSPACE="${FCR_ROS2_WS:-$HOME/ros2_ws}"
MODEL_PATH="${FCR_MODEL_PATH:-$HOME/fcr_models/yolov8n_fp16.engine}"
CAN_BITRATE="${FCR_GIMBAL_CAN_BITRATE:-1000000}"
CAN_INTERFACE="${FCR_GIMBAL_CAN_INTERFACE:-}"
FOXGLOVE_PORT="${FCR_FOXGLOVE_PORT:-8765}"
START_GEMINI="${FCR_START_GEMINI:-true}"
ENABLE_CHASSIS="${FCR_ENABLE_CHASSIS:-true}"
SERVO_TRANSLATION="${FCR_SERVO_TRANSLATION:-true}"

usage() {
  cat <<'EOF'
Usage: ros2 run bringup_pkg start_fcr_ibvs.sh [options]

Options:
  --can-interface IFACE  Use an explicit SocketCAN interface.
  --can-bitrate RATE     Configure the gimbal CAN bitrate (default: 1000000).
  --model PATH           TensorRT engine path.
  --workspace PATH       ROS 2 workspace (default: ~/ros2_ws).
  --foxglove-port PORT   Foxglove WebSocket port (default: 8765).
  --no-gemini            Do not start Gemini/depth fusion.
  --no-chassis           Do not start the chassis driver.
  --no-translation       Disable automatic chassis translation.
  -h, --help             Show this help.

Environment equivalents:
  FCR_GIMBAL_CAN_INTERFACE, FCR_GIMBAL_CAN_BITRATE, FCR_MODEL_PATH,
  FCR_ROS2_WS, FCR_FOXGLOVE_PORT, FCR_START_GEMINI,
  FCR_ENABLE_CHASSIS, FCR_SERVO_TRANSLATION.
EOF
}

while (($# > 0)); do
  case "$1" in
    --can-interface)
      CAN_INTERFACE="${2:?missing value for --can-interface}"
      shift 2
      ;;
    --can-bitrate)
      CAN_BITRATE="${2:?missing value for --can-bitrate}"
      shift 2
      ;;
    --model)
      MODEL_PATH="${2:?missing value for --model}"
      shift 2
      ;;
    --workspace)
      WORKSPACE="${2:?missing value for --workspace}"
      shift 2
      ;;
    --foxglove-port)
      FOXGLOVE_PORT="${2:?missing value for --foxglove-port}"
      shift 2
      ;;
    --no-gemini)
      START_GEMINI=false
      shift
      ;;
    --no-chassis)
      ENABLE_CHASSIS=false
      shift
      ;;
    --no-translation)
      SERVO_TRANSLATION=false
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

require_command() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "ERROR: required command '$1' is not installed." >&2
    exit 1
  fi
}

require_command ip

if [[ ! "$CAN_BITRATE" =~ ^[0-9]+$ ]] || ((CAN_BITRATE <= 0)); then
  echo "ERROR: invalid CAN bitrate: $CAN_BITRATE" >&2
  exit 2
fi
if [[ ! "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]] ||
   ((FOXGLOVE_PORT < 1 || FOXGLOVE_PORT > 65535)); then
  echo "ERROR: invalid Foxglove port: $FOXGLOVE_PORT" >&2
  exit 2
fi

detect_can_interface() {
  local -a all_can=()
  local -a gs_usb_can=()
  local path iface driver

  shopt -s nullglob
  for path in /sys/class/net/can*; do
    iface="${path##*/}"
    [[ "$iface" =~ ^can[0-9]+$ ]] || continue
    all_can+=("$iface")
    driver="$(basename "$(readlink -f "$path/device/driver" 2>/dev/null || true)")"
    if [[ "$driver" == "gs_usb" ]]; then
      gs_usb_can+=("$iface")
    fi
  done
  shopt -u nullglob

  if ((${#gs_usb_can[@]} == 1)); then
    printf '%s\n' "${gs_usb_can[0]}"
    return
  fi
  if ((${#gs_usb_can[@]} > 1)); then
    echo "ERROR: multiple gs_usb CAN interfaces found: ${gs_usb_can[*]}" >&2
    echo "Specify the RS2 adapter with --can-interface or FCR_GIMBAL_CAN_INTERFACE." >&2
    exit 1
  fi
  if ((${#all_can[@]} == 1)); then
    echo "WARN: ${all_can[0]} is not identified as gs_usb; using the only CAN interface." >&2
    printf '%s\n' "${all_can[0]}"
    return
  fi
  if ((${#all_can[@]} == 0)); then
    echo "ERROR: no SocketCAN interface found under /sys/class/net/can*." >&2
  else
    echo "ERROR: multiple CAN interfaces found: ${all_can[*]}" >&2
    echo "Specify the RS2 adapter with --can-interface or FCR_GIMBAL_CAN_INTERFACE." >&2
  fi
  exit 1
}

if [[ -z "$CAN_INTERFACE" ]]; then
  CAN_INTERFACE="$(detect_can_interface)"
fi
if [[ ! "$CAN_INTERFACE" =~ ^[A-Za-z0-9_.:-]+$ ]] ||
   [[ ! -e "/sys/class/net/$CAN_INTERFACE" ]]; then
  echo "ERROR: SocketCAN interface '$CAN_INTERFACE' does not exist." >&2
  exit 1
fi

echo "Configuring gimbal CAN: interface=$CAN_INTERFACE bitrate=$CAN_BITRATE"
sudo ip link set dev "$CAN_INTERFACE" down 2>/dev/null || true
sudo ip link set dev "$CAN_INTERFACE" type can bitrate "$CAN_BITRATE" restart-ms 100
sudo ip link set dev "$CAN_INTERFACE" up

if ! ip link show dev "$CAN_INTERFACE" | grep -q "UP"; then
  echo "ERROR: failed to bring $CAN_INTERFACE UP." >&2
  exit 1
fi
ip -details link show dev "$CAN_INTERFACE"

if [[ ! -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]]; then
  echo "ERROR: /opt/ros/$ROS_DISTRO_NAME/setup.bash does not exist." >&2
  exit 1
fi
if [[ ! -f "$WORKSPACE/install/setup.bash" ]]; then
  echo "ERROR: workspace setup file not found: $WORKSPACE/install/setup.bash" >&2
  exit 1
fi
if [[ ! -f "$MODEL_PATH" ]]; then
  echo "ERROR: TensorRT engine not found: $MODEL_PATH" >&2
  exit 1
fi

# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"
require_command ros2

echo
echo "Starting FCR:"
echo "  controller     : IBVS"
echo "  model          : $MODEL_PATH"
echo "  gimbal CAN     : $CAN_INTERFACE"
echo "  Gemini/fusion  : $START_GEMINI"
echo "  chassis        : $ENABLE_CHASSIS"
echo "  translation    : $SERVO_TRANSLATION"
echo "  Foxglove       : ws://0.0.0.0:$FOXGLOVE_PORT"
echo "  command mux    : starts in MANUAL; switch to AUTO explicitly"
echo

exec ros2 launch bringup_pkg fcr_bringup.launch.py \
  use_sim:=false \
  use_rviz:=false \
  use_foxglove:=true \
  foxglove_port:="$FOXGLOVE_PORT" \
  enable_chassis:="$ENABLE_CHASSIS" \
  can_interface:="$CAN_INTERFACE" \
  detection_device:=tensorrt \
  model_path:="$MODEL_PATH" \
  enable_depth_fusion:="$START_GEMINI" \
  start_gemini:="$START_GEMINI" \
  enable_servo:=true \
  controller_plugin:=servo_control_pkg::IBVSController \
  servo_auto_start:=true \
  servo_target_topic:=/perception/targets_3d \
  servo_allow_chassis_translation:="$SERVO_TRANSLATION"
