# TRON1 Real Test Runbook

Date: 2026-09-05

Status: future-use runbook. Do not execute motion steps until the listed prerequisites are met on site.

Global rule:

```text
No isolated chat command is a real-motion permit.
Only commands in this runbook may be considered, and any line marked [UNVERIFIED - DO NOT EXECUTE] must not be run.
```

## Step 1. Pre-flight Inspection

Command:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
source /home/miya/limx_ws/install/setup.bash
./tools/tron1_bringup/tron1_safety_acceptance_check.sh
```

Purpose: confirm the workspace, defaults, graph guards, A-10 gates, and known acceptance evidence before any hardware prep.

Expected result: `FAIL=0`. `BLOCK` is expected if A-10 manual gates or live graph checks are not complete.

PASS: no `FAIL`, and every `BLOCK` is understood as an unmet real-motion prerequisite rather than ignored.

FAIL: any unexplained `FAIL`, unexpected live graph, Gazebo/steering leftovers, or bare `/cmd_vel` hazard.

Abort condition: unexpected publisher, controller, `ros2 topic pub`, Gazebo process, or any text suggesting immediate real motion.

Recovery: stop all ROS/Gazebo processes, rerun the read-only gate, and return to documentation review.

## Step 2. Network Verification

Command:

```bash
# PC -> Jetson USB SSH, on the PC
JETSON_IP=192.168.55.1
ip -brief addr
ip route get $JETSON_IP
ping -c 1 -W 1 $JETSON_IP

# Jetson -> TRON1 Ethernet, on the Jetson
cd /home/miya/follow_ws/src/fcr_ros2_3
TRON_LINK_IFACE=<Jetson wired interface to TRON1> ./tools/tron1_bringup/jetson_tron1_network_preflight.sh
```

Purpose: prove routing and ping only. This is not controller bringup.

Expected result: PC route to Jetson uses the USB gadget NIC, not Mihomo/TUN. Jetson route to `10.192.1.2` uses the wired TRON1 interface and ping succeeds.

PASS: `jetson_tron1_network_preflight.sh` reports `FAIL=0 BLOCK=0`.

FAIL: route goes through `Mihomo`, `tun*`, `utun*`, `tap*`, `wg*`, `tailscale*`, `docker*`, bridge, policy table, or ping fails after cabling is expected to be ready.

Abort condition: any proposal to solve routing by launching ROS, `robot_hw`, controller, or publishing velocity.

Recovery: fix only IP/link/route. Use `docs/ai/JETSON_USB_SSH_QUICKSTART_2026-09-05.md` and `docs/ai/TRON1_JETSON_NETWORK_SETUP_REPEATABLE_2026-09-05.md`.

## Step 3. SDK Communication

Command:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
ros2 run limxsdk_lowlevel pf_sensorjoy_monitor 10.192.1.2
```

Purpose: read remote-controller `SensorJoy` data without sending motor commands.

Expected result: button and axis values change when the operator moves the controller.

PASS: `SensorJoy` data is visible, and no `robot_hw`, controller activation, or velocity publisher is started.

FAIL: no SDK data, wrong IP route, or the command unexpectedly starts a control process.

Abort condition: any motor/controller activation, unexpected TRON1 posture change, or operator discomfort.

Recovery: Ctrl-C the monitor, disconnect the TRON1 link if needed, and return to network verification.

## Step 4. Robot State Verification

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
No verified non-motion command is currently documented that proves TRON1 controller state while FCR is active.
```

Purpose: establish a reliable state source before interpreting remote e-stop or controller readiness.

Expected result: a future verified SDK/ROS state command reports developer/idle/estop/controller state without commanding motion.

PASS: state source is read-only, repeatable, and recorded.

FAIL: state must be inferred from robot movement, controller activation, or unverified logs.

Abort condition: any requirement to stand or walk the robot to learn state.

Recovery: keep real motion paused and add the verified state command to this runbook after review.

## Step 5. Physical E-stop Verification

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Human-only physical stop rehearsal. No shell command.
```

Purpose: verify the operator can immediately reach and trigger the physical motor switch / hardware stop.

Expected result: the operator can identify the action and explain recovery before any controller activation.

PASS: the action is rehearsed in place, with one hand reachable and a second observer if available.

FAIL: unclear switch/action, blocked access, crowded room, or no stable support/stand.

Abort condition: small office, unstable robot, no physical access, or uncertain recovery.

Recovery: do not proceed; move to a safe area or support frame and repeat.

## Step 6. Software Stop Verification

Command:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh
```

Isolation: run this with TRON1 disconnected or unreachable, or in a confirmed isolated simulation network/domain. Do not add `--allow-robot-network` during on-site support-frame/lawn prep just to bypass the guard.

Purpose: verify FCR software estop, limiter latch, and command zeroing in simulation/isolated ROS graph. Relevant acceptance cases include external estop/clear checks and software mode-request estop cases.

Expected result: acceptance completes without `FAIL`.

PASS: software estop zeros `/fcr_tron/cmd_vel`, latch/clear behavior is understood, and results are recorded.

FAIL: estop does not zero output, clear path is unsafe, or unexpected graph appears.

Abort condition: script attempts real hardware, source route points to TRON1, or starts official `robot_hw`.

Recovery: stop the script, inspect graph, and rerun only in isolated simulation/test domain.

## Step 7. Command Timeout Verification

Command:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash
./tools/tron1_bringup/run_tron1_safe_mode_acceptance.sh
```

Isolation: run this with TRON1 disconnected or unreachable, or in a confirmed isolated simulation network/domain. Do not add `--allow-robot-network` during on-site support-frame/lawn prep just to bypass the guard.

Purpose: verify stale input, stale authorization, and stale estop samples fail closed in the FCR safety layer. Relevant acceptance cases include input timeout, limiter state timeout, and estop sample timeout checks.

Expected result: timeout cases publish zero and report the expected blocked state.

PASS: stale command source, stale auth, and stale estop sample are all blocked or zeroed.

FAIL: any stale condition allows nonzero `/fcr_tron/cmd_vel`.

Abort condition: any test requires a real TRON1 controller or real velocity command.

Recovery: fix the software gate in simulation before returning to hardware prep.

## Step 8. Support-Frame / Stand Test

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Future command must come from a reviewed support-frame test entry after Steps 1-7 pass on site.
```

Purpose: first real controller/FCR interaction with wheels/feet unloaded or constrained.

Expected result: startup remains at `enable_motion=false`, official controller subscription and limiter publisher are verified, and no unexpected motion occurs.

PASS: robot is physically supported, stop actions are reachable, graph is correct, and all outputs stay zero until a later reviewed pulse.

FAIL: support is unstable, controller subscribes to bare `/cmd_vel`, or FCR output is not uniquely owned by `tron1_safety_limiter`.

Abort condition: any ground contact movement, unexpected posture change, or missing stop operator.

Recovery: stop controller through the documented physical/software path, power down if needed, and archive logs.

## Step 9. Lawn Lowest-Risk Test

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
No lawn command is authorized in this freeze.
```

Purpose: move from support-frame evidence to open-area, low-consequence surface testing.

Expected result: all support-frame criteria already passed, robot has clear space, and physical stop is immediately reachable.

PASS: only after documented support-frame PASS plus reviewed lawn setup.

FAIL: no support-frame PASS, small indoor space, or unverified remote e-stop.

Abort condition: bystanders, obstacles, slope, unstable posture, or communication uncertainty.

Recovery: return to support-frame test and reduce risk.

## Step 10. Single-Axis Command

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Do not invent or run a real velocity command here. Add the reviewed command only after support-frame PASS.
```

Purpose: test one translational or yaw axis at the lowest practical pulse.

Expected result: measurable, bounded response followed by immediate zero output.

PASS: direction, duration, stop, and recovery are all recorded.

FAIL: wrong direction, drift, no stop, unexpected controller state, or operator cannot abort.

Abort condition: any uncommanded motion or inability to stop.

Recovery: stop physically, stop software, disable motion, archive logs.

## Step 11. Stop

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Use the stop method verified in Steps 5-7 and support-frame testing; do not add a new stop command in the field.
```

Purpose: prove the exact stop path used after each pulse.

Expected result: command output returns to zero and robot state is stable.

PASS: stop is immediate enough for the test area and does not require guessing.

FAIL: stop depends on an unverified key, stale graph, or physical movement continuing.

Abort condition: delayed stop or unclear state.

Recovery: physical stop, then postmortem before any next command.

## Step 12. Turn

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Yaw/turn command must be added only after single-axis support-frame PASS.
```

Purpose: verify low yaw behavior without mixing translation.

Expected result: bounded yaw response and zero lateral/forward surprise within the support-frame or open-area plan.

PASS: turn direction and stop are documented.

FAIL: yaw causes unexpected translation, oscillation, or drift beyond plan.

Abort condition: unstable body or unexpected displacement.

Recovery: stop, reduce limits, and return to simulation/design review.

## Step 13. Stop

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Repeat only the already verified stop path.
```

Purpose: prove repeatability after a yaw command.

Expected result: robot returns to controlled zero-output state.

PASS: repeat stop behavior matches Step 11.

FAIL: stop behavior changes after yaw.

Abort condition: any delayed or uncertain stop.

Recovery: physical stop and end the test session.

## Step 14. Failure Injection

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
Do not kill live real-motion processes until support-frame failure-injection procedure is separately reviewed.
```

Purpose: verify stale command, node death, and communication loss behavior under controlled conditions.

Expected result: fail-closed behavior without ground risk.

PASS: injected failure produces the expected zero/block/log and no unsafe movement.

FAIL: failure allows movement or leaves state unclear.

Abort condition: any failure mode lacks a known recovery path.

Recovery: power/physical stop first, then logs; do not retry in the same session without review.

## Step 15. FCR Integration

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
No full FCR TRON1 integration command is authorized in this freeze.
```

Purpose: add perception/mux/limiter/controller interaction after standalone TRON1 safety is proven.

Expected result: FCR remains a slow correction source, not a direct uncontrolled base driver.

PASS: graph ownership, limiter state, target loss, estop, and timeout all stay correct.

FAIL: direct `/cmd_vel`, competing command sources, or unexpected nonzero output.

Abort condition: target loss keeps moving, or FCR next-cycle command overrides a stop.

Recovery: disable motion, stop nodes, return to simulation.

## Step 16. Person-Following

Command:

```text
[UNVERIFIED - DO NOT EXECUTE]
No person-following base-motion command is authorized in this freeze.
```

Purpose: final integrated behavior after all prior gates pass.

Expected result: RS2 handles fast visual centering; TRON1 only performs slow yaw/distance correction.

PASS: tracking, target loss, stop, and operator override all behave as documented.

FAIL: base chases detection noise, loses target while moving, or stop is not dominant.

Abort condition: any unsafe tracking behavior or unclear command source.

Recovery: software stop, physical stop, archive logs, and freeze before further work.
