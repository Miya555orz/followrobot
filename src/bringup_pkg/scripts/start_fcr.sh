#!/usr/bin/env bash
set -Eeuo pipefail

# One-command real-hardware bringup for the FCR visual follow stack.
#
# The script deliberately leaves command_mux in its configured default
# "manual" mode. The perception and servo nodes may produce /auto commands,
# but the operator must explicitly publish "auto" before actuators follow them.

ROS_DISTRO_NAME="${ROS_DISTRO_NAME:-humble}"
WORKSPACE="${FCR_ROS2_WS:-$HOME/ros2_ws}"
MODEL_PATH="${FCR_MODEL_PATH:-$HOME/fcr_models/yolov8n_fp16.engine}"
CAN_BITRATE="${FCR_GIMBAL_CAN_BITRATE:-1000000}"
CAN_INTERFACE="${FCR_GIMBAL_CAN_INTERFACE:-}"
CAN_USB_PATH="${FCR_GIMBAL_CAN_USB_PATH:-}"
CAN_WAIT_TIMEOUT="${FCR_GIMBAL_CAN_WAIT_TIMEOUT:-12}"
CAN_CONFIG_RETRIES="${FCR_GIMBAL_CAN_CONFIG_RETRIES:-3}"
CAN_RESTART_MS="${FCR_GIMBAL_CAN_RESTART_MS:-100}"
# Keep the queue bounded. A very large queue hides missing CAN ACKs and turns
# current gimbal commands into a long stale backlog.
CAN_TX_QUEUE="${FCR_GIMBAL_CAN_TX_QUEUE:-128}"
FOXGLOVE_PORT="${FCR_FOXGLOVE_PORT:-8765}"
START_GEMINI="${FCR_START_GEMINI:-true}"
ENABLE_CHASSIS="${FCR_ENABLE_CHASSIS:-true}"
SERVO_TRANSLATION="${FCR_SERVO_TRANSLATION:-true}"
CONTROLLER="${FCR_CONTROLLER:-pbvs}"
ENABLE_SERVO_MANAGER="${FCR_ENABLE_SERVO_MANAGER:-true}"
ENABLE_GIMBAL_VISUAL_SERVO="${FCR_ENABLE_GIMBAL_VISUAL_SERVO:-true}"
ENABLE_CINEMATIC_MOTION="${FCR_ENABLE_CINEMATIC_MOTION:-true}"
ENABLE_VOICE="${FCR_ENABLE_VOICE:-true}"

usage() {
  cat <<'EOF'
Usage: ros2 run bringup_pkg start_fcr.sh [options]

Options:
  --can-interface IFACE  Use an explicit SocketCAN interface.
  --can-usb-path PATH    Select the gs_usb device by USB interface path
                        (example: 1-2.3:1.0; stable across can0/can1 renames).
  --can-bitrate RATE     Configure the gimbal CAN bitrate (default: 1000000).
  --model PATH           TensorRT engine path.
  --workspace PATH       ROS 2 workspace (default: ~/ros2_ws).
  --foxglove-port PORT   Foxglove WebSocket port (default: 8765).
  --controller MODE      Chassis/depth controller: ibvs or pbvs (default: pbvs).
  --gimbal-only          Start Sony 2D perception and the RS2 gimbal fast loop only.
  --legacy-gimbal-loop   Disable the independent Sony 2D gimbal fast loop.
  --no-cinematic         Disable STATIC/DOLLY/TRUCK/ORBIT task support.
  --no-gemini            Do not start Gemini/depth fusion.
  --no-chassis           Do not start the chassis driver.
  --no-translation       Disable automatic chassis translation.
  --no-voice             Disable the Jetson-side candidate-intent gate and routing.
  -h, --help             Show this help.

Environment equivalents:
  FCR_GIMBAL_CAN_INTERFACE, FCR_GIMBAL_CAN_USB_PATH,
  FCR_GIMBAL_CAN_BITRATE, FCR_GIMBAL_CAN_WAIT_TIMEOUT,
  FCR_GIMBAL_CAN_CONFIG_RETRIES, FCR_GIMBAL_CAN_RESTART_MS,
  FCR_GIMBAL_CAN_TX_QUEUE, FCR_MODEL_PATH,
  FCR_ROS2_WS, FCR_FOXGLOVE_PORT, FCR_START_GEMINI,
  FCR_ENABLE_CHASSIS, FCR_SERVO_TRANSLATION, FCR_CONTROLLER,
  FCR_ENABLE_SERVO_MANAGER,
  FCR_ENABLE_GIMBAL_VISUAL_SERVO, FCR_ENABLE_CINEMATIC_MOTION,
  FCR_ENABLE_VOICE.
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
    --can-usb-path)
      CAN_USB_PATH="${2:?missing value for --can-usb-path}"
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
    --controller)
      CONTROLLER="${2:?missing value for --controller}"
      shift 2
      ;;
    --gimbal-only)
      START_GEMINI=false
      ENABLE_CHASSIS=false
      SERVO_TRANSLATION=false
      ENABLE_SERVO_MANAGER=false
      ENABLE_GIMBAL_VISUAL_SERVO=true
      ENABLE_CINEMATIC_MOTION=false
      ENABLE_VOICE=false
      shift
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
    --legacy-gimbal-loop)
      ENABLE_GIMBAL_VISUAL_SERVO=false
      shift
      ;;
    --no-cinematic)
      ENABLE_CINEMATIC_MOTION=false
      shift
      ;;
    --no-voice)
      ENABLE_VOICE=false
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
require_command readlink
require_command sudo

for numeric_setting in \
  CAN_BITRATE CAN_WAIT_TIMEOUT CAN_CONFIG_RETRIES CAN_RESTART_MS CAN_TX_QUEUE
do
  value="${!numeric_setting}"
  if [[ ! "$value" =~ ^[0-9]+$ ]] || ((value <= 0)); then
    echo "ERROR: invalid $numeric_setting: $value" >&2
    exit 2
  fi
done
if [[ ! "$FOXGLOVE_PORT" =~ ^[0-9]+$ ]] ||
   ((FOXGLOVE_PORT < 1 || FOXGLOVE_PORT > 65535)); then
  echo "ERROR: invalid Foxglove port: $FOXGLOVE_PORT" >&2
  exit 2
fi
case "${CONTROLLER,,}" in
  ibvs)
    CONTROLLER=ibvs
    CONTROLLER_PLUGIN="servo_control_pkg::IBVSController"
    ALLOCATION_RATIO="0.5"
    ;;
  pbvs)
    CONTROLLER=pbvs
    CONTROLLER_PLUGIN="servo_control_pkg::PBVSController"
    # PBVS uses a cascade: gimbal handles the fast bearing loop while the
    # chassis yaw only recentres the gimbal through the allocator unwind term.
    ALLOCATION_RATIO="0.0"
    ;;
  *)
    echo "ERROR: --controller must be 'ibvs' or 'pbvs', got: $CONTROLLER" >&2
    exit 2
    ;;
esac

if [[ "$ENABLE_CINEMATIC_MOTION" == true && "$CONTROLLER" != pbvs ]]; then
  echo "WARNING: cinematic motion requires PBVS; disabling cinematic task support." >&2
  ENABLE_CINEMATIC_MOTION=false
fi
if [[ "$ENABLE_CINEMATIC_MOTION" == true && "$ENABLE_SERVO_MANAGER" != true ]]; then
  echo "WARNING: cinematic motion requires servo_manager; disabling it." >&2
  ENABLE_CINEMATIC_MOTION=false
fi

can_driver() {
  local iface="$1"
  basename "$(
    readlink -f "/sys/class/net/$iface/device/driver" 2>/dev/null || true
  )"
}

can_usb_path() {
  local iface="$1"
  basename "$(
    readlink -f "/sys/class/net/$iface/device" 2>/dev/null || true
  )"
}

list_gs_usb_interfaces() {
  local -a gs_usb_can=()
  local path iface driver

  shopt -s nullglob
  for path in /sys/class/net/can*; do
    iface="${path##*/}"
    [[ "$iface" =~ ^can[0-9]+$ ]] || continue
    driver="$(can_driver "$iface")"
    if [[ "$driver" == "gs_usb" ]]; then
      if [[ -n "$CAN_USB_PATH" ]] &&
         [[ "$(can_usb_path "$iface")" != "$CAN_USB_PATH" ]]; then
        continue
      fi
      gs_usb_can+=("$iface")
    fi
  done
  shopt -u nullglob
  if ((${#gs_usb_can[@]} > 0)); then
    printf '%s\n' "${gs_usb_can[@]}"
  fi
}

detect_can_interface_once() {
  local -a candidates=()
  mapfile -t candidates < <(list_gs_usb_interfaces)
  if ((${#candidates[@]} == 1)); then
    printf '%s\n' "${candidates[0]}"
    return 0
  fi
  if ((${#candidates[@]} > 1)); then
    echo "ERROR: multiple matching gs_usb CAN interfaces found: ${candidates[*]}" >&2
    echo "Use --can-usb-path or FCR_GIMBAL_CAN_USB_PATH to select the RS2 adapter." >&2
    return 2
  fi
  return 1
}

wait_for_can_interface() {
  local deadline=$((SECONDS + CAN_WAIT_TIMEOUT))
  local detected=""
  local status=0
  local path iface
  while ((SECONDS <= deadline)); do
    if detected="$(detect_can_interface_once)"; then
      printf '%s\n' "$detected"
      return 0
    else
      status=$?
      ((status == 2)) && return 2
    fi
    sudo modprobe gs_usb >/dev/null 2>&1 || true
    sleep 1
  done
  echo "ERROR: gs_usb RS2 adapter did not enumerate within ${CAN_WAIT_TIMEOUT}s." >&2
  echo "Available CAN devices:" >&2
  for path in /sys/class/net/can*; do
    [[ -e "$path" ]] || continue
    iface="${path##*/}"
    echo "  $iface driver=$(can_driver "$iface") usb=$(can_usb_path "$iface")" >&2
  done
  echo "USB error -71/-110 cannot be repaired in software; check adapter power, cable and hub." >&2
  return 1
}

validate_can_interface() {
  local iface="$1"
  if [[ ! "$iface" =~ ^[A-Za-z0-9_.:-]+$ ]] ||
     [[ ! -e "/sys/class/net/$iface" ]]; then
    echo "ERROR: SocketCAN interface '$iface' does not exist." >&2
    return 1
  fi
  if [[ "$(can_driver "$iface")" != "gs_usb" ]]; then
    echo "ERROR: '$iface' uses driver '$(can_driver "$iface")', not gs_usb." >&2
    echo "The Jetson mttcan interface must never be used for the RS2 USB-CAN adapter." >&2
    return 1
  fi
  if [[ -n "$CAN_USB_PATH" ]] &&
     [[ "$(can_usb_path "$iface")" != "$CAN_USB_PATH" ]]; then
    echo "ERROR: '$iface' belongs to USB path '$(can_usb_path "$iface")'," >&2
    echo "       expected '$CAN_USB_PATH'." >&2
    return 1
  fi
}

# Prevent two bringup processes from reconfiguring the same CAN bus and starting
# duplicate gimbal/servo nodes.
if command -v flock >/dev/null 2>&1; then
  exec 9>"/tmp/fcr_gimbal_can.lock"
  if ! flock -n 9; then
    echo "ERROR: another FCR bringup process owns /tmp/fcr_gimbal_can.lock." >&2
    exit 1
  fi
fi
# Ask for sudo once. Repeated prompts in the middle of hardware recovery can
# leave the interface half-configured.
sudo -v

if [[ -z "$CAN_INTERFACE" ]]; then
  CAN_INTERFACE="$(wait_for_can_interface)"
else
  validate_can_interface "$CAN_INTERFACE"
  if [[ -z "$CAN_USB_PATH" ]]; then
    CAN_USB_PATH="$(can_usb_path "$CAN_INTERFACE")"
  fi
fi

validate_can_interface "$CAN_INTERFACE"
CAN_USB_PATH="${CAN_USB_PATH:-$(can_usb_path "$CAN_INTERFACE")}"

echo "Configuring gimbal CAN: interface=$CAN_INTERFACE usb=$CAN_USB_PATH bitrate=$CAN_BITRATE"

configure_can_once() {
  local iface="$1"
  validate_can_interface "$iface" || return 1
  sudo ip link set dev "$iface" down >/dev/null 2>&1 || true
  sudo ip link set dev "$iface" type can \
    bitrate "$CAN_BITRATE" restart-ms "$CAN_RESTART_MS" || return 1
  sudo ip link set dev "$iface" txqueuelen "$CAN_TX_QUEUE" || return 1
  sudo ip link set dev "$iface" up || return 1
  sleep 0.2
  can_is_healthy "$iface"
}

can_is_healthy() {
  local iface="$1"
  local details
  details="$(ip -details link show dev "$iface" 2>/dev/null)" || return 1
  grep -qE '<[^>]*UP[^>]*>' <<<"$details" || return 1
  grep -q "bitrate $CAN_BITRATE" <<<"$details" || return 1
  # A freshly configured, correctly terminated bus must start ERROR-ACTIVE.
  # ERROR-WARNING/PASSIVE/BUS-OFF usually means missing ACK, bad termination,
  # wrong bitrate or damaged wiring; starting motion in that state is unsafe.
  grep -q 'can state ERROR-ACTIVE' <<<"$details" || return 1
}

rebind_gs_usb_interface() {
  local iface="$1"
  local usb_interface
  usb_interface="$(can_usb_path "$iface")"
  [[ -n "$usb_interface" ]] || return 1
  [[ -e "/sys/bus/usb/drivers/gs_usb/$usb_interface" ]] || return 1

  echo "WARN: rebinding only gs_usb device $usb_interface (not the global driver)." >&2
  printf '%s' "$usb_interface" |
    sudo tee /sys/bus/usb/drivers/gs_usb/unbind >/dev/null || return 1
  sleep 1
  printf '%s' "$usb_interface" |
    sudo tee /sys/bus/usb/drivers/gs_usb/bind >/dev/null || return 1
  sleep 2
}

configured=false
for ((attempt = 1; attempt <= CAN_CONFIG_RETRIES; attempt++)); do
  if configure_can_once "$CAN_INTERFACE"; then
    configured=true
    break
  fi
  echo "WARN: CAN configuration attempt $attempt/$CAN_CONFIG_RETRIES failed." >&2
  if ((attempt == 1)) && [[ -e "/sys/class/net/$CAN_INTERFACE" ]]; then
    rebind_gs_usb_interface "$CAN_INTERFACE" || true
  fi
  CAN_INTERFACE="$(wait_for_can_interface)" || break
done

if [[ "$configured" != true ]]; then
  echo "ERROR: CAN startup recovery exhausted; FCR will not start with an unhealthy bus." >&2
  echo "Replug the USB-CAN adapter and inspect: sudo dmesg | tail -n 60" >&2
  exit 1
fi

ip -details link show dev "$CAN_INTERFACE"
printf 'interface=%s\nusb_path=%s\nbitrate=%s\n' \
  "$CAN_INTERFACE" "$CAN_USB_PATH" "$CAN_BITRATE" \
  >"/tmp/fcr_gimbal_can.env"

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
# ROS 2 generated setup files legitimately probe optional variables before
# defining them, which is incompatible with Bash nounset. Keep strict mode for
# this script, but suspend nounset only while the generated environments load.
set +u
# shellcheck disable=SC1090
source "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
# shellcheck disable=SC1090
source "$WORKSPACE/install/setup.bash"
set -u
require_command ros2

echo
echo "Starting FCR:"
echo "  controller     : ${CONTROLLER^^}"
echo "  servo manager  : $ENABLE_SERVO_MANAGER"
echo "  2D gimbal loop : $ENABLE_GIMBAL_VISUAL_SERVO"
echo "  cinematic task : $ENABLE_CINEMATIC_MOTION"
echo "  model          : $MODEL_PATH"
echo "  gimbal CAN     : $CAN_INTERFACE"
echo "  Gemini/fusion  : $START_GEMINI"
echo "  chassis        : $ENABLE_CHASSIS"
echo "  translation    : $SERVO_TRANSLATION"
echo "  Foxglove       : ws://0.0.0.0:$FOXGLOVE_PORT"
echo "  voice gate      : $ENABLE_VOICE (external candidate intents only)"
echo "  command mux    : starts in MANUAL; switch to AUTO explicitly"
echo

# PBVS uses the fused 3D target for distance control and the 2D aim target for
# the fast angular loop. Keep comments outside the continued launch command:
# a shell comment inside a backslash continuation truncates all later args.
exec ros2 launch bringup_pkg fcr_bringup.launch.py \
  use_sim:=false \
  use_rviz:=false \
  use_foxglove:=true \
  foxglove_port:="$FOXGLOVE_PORT" \
  enable_chassis:="$ENABLE_CHASSIS" \
  can_interface:="$CAN_INTERFACE" \
  gimbal_control_mode:=speed \
  detection_device:=tensorrt \
  model_path:="$MODEL_PATH" \
  enable_depth_fusion:="$START_GEMINI" \
  start_gemini:="$START_GEMINI" \
  enable_monitor_future_inputs:=true \
  enable_servo:=true \
  controller_plugin:="$CONTROLLER_PLUGIN" \
  servo_auto_start:=true \
  servo_target_topic:=/perception/targets_3d \
  servo_aim_target_topic:=/perception/aim_target_2d \
  enable_servo_manager:="$ENABLE_SERVO_MANAGER" \
  enable_gimbal_visual_servo:="$ENABLE_GIMBAL_VISUAL_SERVO" \
  enable_cinematic_motion:="$ENABLE_CINEMATIC_MOTION" \
  servo_allocation_ratio:="$ALLOCATION_RATIO" \
  servo_allow_chassis_translation:="$SERVO_TRANSLATION" \
  enable_voice_control:="$ENABLE_VOICE"
