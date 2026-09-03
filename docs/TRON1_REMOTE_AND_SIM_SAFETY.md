# TRON1 Remote Controller and Simulation-First Safety Notes

Date: 2026-09-03

Scope: TRON1 EDU / WF_TRON1A, local PC `/home/miya/follow_ws` + `/home/miya/limx_ws`.

This note records the real-robot observations from the first developer-mode bring-up and defines the next safe path: use simulation and remote-controller monitoring first; do not continue free-running real motion.

## Current conclusion

Real TRON1 bring-up was stronger and faster than expected. The project should pause real motion tests and return to simulation plus remote-controller familiarization.

Current status:

```text
[✓] PC <-> TRON1 Ethernet link works.
[✓] TRON1 default IP 10.192.1.2 is reachable through enp0s31f6.
[✓] Official robot_hw can connect to TRON1.
[✓] Remote controller axes and buttons are visible through LimX SDK SensorJoy.
[✓] L1 + triangle/Y starts WheelfootController.
[✓] Physical motor switch / hardware action can put motors into damping mode.
[!] L1 + X is software stopController + abort, not motor damping / torque release.
[!] Real motion felt too aggressive for this stage.
[ ] Continue in Gazebo/simulation before more real movement.
```

## Remote controller mapping observed

Official config file:

```text
/home/miya/limx_ws/src/tron1-rl-deploy-ros2/robot_hw/config/joystick.yaml
```

Observed/official mapping:

```text
A      = button 0
B      = button 1
X      = button 2
Y/△    = button 3
L1     = button 4
R2     = button 5
L2     = button 6
R1     = button 7
SELECT = button 8
START  = button 9
Up     = button 12
Down   = button 13
Left   = button 14
Right  = button 15
MENU   = button 16
```

Axes:

```text
left stick horizontal  = axes[0]
left stick vertical    = axes[1]
right stick horizontal = axes[2]
right stick vertical   = axes[3]
```

Official ROS behavior:

```text
L1 + Y/△ = startController(WheelfootController)
L1 + X   = stopController(WheelfootController), then abort()
```

Important: `L1 + X` is not a damping/zero-torque command. It stops the official controller process path. Hardware damping was observed through the physical motor switch/hardware action, not through `L1 + X`.

## Read-only remote monitor

A local SDK helper was added in:

```text
/home/miya/limx_ws/src/limxsdk-lowlevel/examples/pf_sensorjoy_monitor.cpp
```

It only subscribes to `SensorJoy` and prints axes/buttons. It does not publish `RobotCmd` and does not send motor commands.

Build:

```bash
source /opt/ros/humble/setup.bash
cd /home/miya/limx_ws
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 --packages-select limxsdk_lowlevel
```

Run:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
ros2 run limxsdk_lowlevel pf_sensorjoy_monitor 10.192.1.2
```

Expected output while pressing buttons:

```text
buttons[..., 4:1, ...]       # L1
buttons[..., 3:1, ...]       # Y / triangle
buttons[..., 2:1, ...]       # X
buttons[..., 4:1, 3:1, ...]  # L1 + Y/triangle
```

## Real-robot log evidence from first activation

During first activation, logs showed the controller was already active:

```text
Controller 'WheelfootController' is already active. Skipping start.
```

After the physical switch/hardware action, logs showed:

```text
Ethercat code: -1, msg: Motor in damping mode
Controller 'WheelfootController' stopped.
pointfoot_node exited with code -6
```

Interpretation: the physical switch/hardware action successfully moved the motor side into damping mode. The official ROS node then treated this as fatal, stopped the controller, and exited.

## Simulation-first next path

Do not continue real motion until the operator is comfortable with remote-controller start/stop behavior and the simulation path has been re-tested.

Recommended next step:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/limx_ws/install/setup.bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
export FCR_TRON_CMD_VEL_TOPIC=/fcr_tron/cmd_vel
export FCR_TRON_CMD_VEL_TIMEOUT_SEC=0.25

ros2 launch robot_hw pointfoot_hw_sim.launch.py \
  use_gazebo:=true \
  fcr_cmd_vel_topic:=/fcr_tron/cmd_vel \
  start_steering_gui:=false
```

Then test the FCR limiter separately:

```bash
source /opt/ros/humble/setup.bash
source /home/miya/follow_ws/install/setup.bash

ros2 launch robot_platform_pkg tron1_safety_limiter.launch.py \
  enable_motion:=false \
  max_linear_x:=0.01 \
  max_angular_z:=0.03 \
  input_timeout_sec:=0.25
```

Only after simulation behavior is understood should real tests resume.

## Real motion pause rule

Until further notice:

```text
Do not run real TRON1 with enable_motion=true.
Do not rely on L1+X as damping or torque release.
Do not start full person-following on TRON1.
Use the physical motor switch/hardware stop as the primary emergency action.
Use simulation to learn controller behavior first.
```
