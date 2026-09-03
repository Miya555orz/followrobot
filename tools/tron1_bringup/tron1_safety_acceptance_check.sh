#!/usr/bin/env bash
set -eo pipefail

PROJECT_ROOT="/home/miya/follow_ws/src/fcr_ros2_3"
FOLLOW_WS="/home/miya/follow_ws"
LIMX_WS="/home/miya/limx_ws"
export ROBOT_TYPE="${ROBOT_TYPE:-WF_TRON1A}"
export RL_TYPE="${RL_TYPE:-isaacgym}"

echo "[TRON1 safety acceptance check]"
echo "This script is read-only. It does not launch ROS, publish velocity, or connect to real TRON1."
echo

if [ -f /opt/ros/humble/setup.bash ]; then
  set +u
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  set -u
else
  echo "[FAIL] /opt/ros/humble/setup.bash not found"
  exit 2
fi

if [ -f "$FOLLOW_WS/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source "$FOLLOW_WS/install/setup.bash"
  set -u
else
  echo "[WARN] $FOLLOW_WS/install/setup.bash not found; FCR packages may be unavailable"
fi

if [ -f "$LIMX_WS/install/setup.bash" ]; then
  set +u
  # shellcheck source=/dev/null
  source "$LIMX_WS/install/setup.bash"
  set -u
else
  echo "[WARN] $LIMX_WS/install/setup.bash not found; TRON official packages may be unavailable"
fi

echo "[1/6] Workspace"
echo "PROJECT_ROOT=$PROJECT_ROOT"
echo "ROBOT_TYPE=$ROBOT_TYPE"
echo "RL_TYPE=$RL_TYPE"
git -C "$PROJECT_ROOT" status --short --branch || true
echo

echo "[2/6] Process snapshot"
pgrep -af 'gazebo|gzserver|gzclient|pointfoot_node|robot_hw_node|tron1_safety_limiter|rqt_robot_steering|ros2 topic pub' || true
echo

echo "[3/6] Package availability"
ros2 pkg prefix robot_platform_pkg >/dev/null 2>&1 \
  && echo "[PASS] robot_platform_pkg visible" \
  || echo "[FAIL] robot_platform_pkg not visible"
ros2 pkg prefix robot_hw >/dev/null 2>&1 \
  && echo "[PASS] LimX robot_hw visible" \
  || echo "[WARN] LimX robot_hw not visible"
echo

echo "[4/6] Launch arguments"
if ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py --show-args >/tmp/tron1_limiter_args.txt 2>&1; then
  grep -E "input_topic|output_topic|enable_motion|max_linear_x|max_linear_y|max_angular_z" /tmp/tron1_limiter_args.txt || true
else
  echo "[WARN] Could not inspect tron1_safety_limiter launch arguments"
fi

if ros2 launch robot_hw pointfoot_hw_sim.launch.py --show-args >/tmp/tron1_hw_sim_args.txt 2>&1; then
  grep -E "fcr_cmd_vel_topic|start_steering_gui|gui|server" /tmp/tron1_hw_sim_args.txt || true
else
  echo "[WARN] Could not inspect pointfoot_hw_sim launch arguments"
fi
echo

echo "[5/6] Live ROS graph snapshot, if graph is running"
if timeout 3s ros2 topic info /fcr_tron/cmd_vel -v >/tmp/tron1_cmd_vel_info.txt 2>&1; then
  cat /tmp/tron1_cmd_vel_info.txt
else
  echo "[INFO] /fcr_tron/cmd_vel is not currently inspectable. This is OK before launch."
  echo "[INFO] If a graph is running and this repeats, restart the ROS 2 daemon: ros2 daemon stop; ros2 daemon start"
fi
echo

echo "[6/6] Required acceptance gates"
cat <<'EOF'
[ ] /fcr_tron/cmd_vel has exactly one publisher: tron1_safety_limiter
[ ] official robot_hw_node subscribes /fcr_tron/cmd_vel
[ ] no FCR/TRON process consumes bare /cmd_vel
[ ] enable_motion=false keeps /fcr_tron/cmd_vel zero under nonzero upstream input
[ ] enable_motion=true clamps to x<=0.03, y=0, yaw<=0.10 for first real tests
[ ] lost upstream command returns /fcr_tron/cmd_vel to zero
[ ] /safety/estop_state=true returns /fcr_tron/cmd_vel to zero
[ ] limiter SIGINT/SIGTERM sends zero burst
[ ] limiter/publisher crash behavior is documented before real motion
[ ] physical motor switch / hardware damping path is reachable
EOF

echo
echo "Next: follow docs/TRON1_SAFETY_ACCEPTANCE_CHECKLIST.md. Do not run real motion until all gates are understood."
