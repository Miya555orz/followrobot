# Migration Plan

## Current Assumption

The first migration target is not to rewrite the follow algorithm. It is to preserve the FCR perception and servo chain, then replace the LEKIWI base execution path with a TRON1 adapter.

## Step Plan

### Step 1: Verify ROS 2 Humble on Ubuntu

Purpose: prove the base ROS environment works before involving project code.

Run on: Ubuntu.

Commands:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

Second terminal:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

Expected result: listener prints messages.

Acceptance: two ROS 2 demo nodes communicate.

### Step 2: Build FCR Without Hardware

Purpose: establish the original project baseline.

Run on: Ubuntu.

Commands:

```bash
cd ~/fcr_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17
```

Expected result: build succeeds or produces a dependency error we can fix.

Acceptance: `colcon build` reaches 100 percent with no failed packages.

### Step 3: Run FCR Mock Control Loop

Purpose: validate perception/control message flow without robot hardware.

Run on: Ubuntu.

Operation: use the project's mock or MVP launch from `simulation_pkg` / bringup docs.

Expected result: target messages produce bounded command messages.

Acceptance: `/auto/cmd_vel` and muxed command output can be echoed and stop on target loss.

### Step 4: Build TRON1 Official Stack

Purpose: decide Humble-only versus split Humble/Iron workflow.

Run on: Ubuntu.

Operation:

```bash
cd ~/tron1_ws/src/tron1-rl-deploy-ros2
git fetch origin
git checkout feature/humble
cd ~/tron1_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble
colcon build --symlink-install
```

Expected result: either Humble branch builds, or errors clearly show Iron-only dependencies.

Acceptance: choose one of:

```text
Humble-only path accepted
Split FCR Humble + TRON1 Iron path required
```

### Step 5: Verify TRON1 `/cmd_vel` in Simulation

Purpose: confirm command type, direction, stop behavior, and scaling before adapter work.

Run on: Ubuntu.

Operation: launch TRON1 simulation, then publish small Twist commands.

Expected result: robot responds to `linear.x` and `angular.z`; `linear.y` remains disabled for early tests.

Acceptance: zero command reliably stops the simulated robot.

### Step 6: Create Adapter Package

Purpose: bridge FCR `TwistStamped` output to TRON1 `Twist` input safely.

Run on: Windows for initial code review, Ubuntu for build/test.

Responsibilities:

```text
subscribe FCR safe TwistStamped
check timestamp freshness
scale physical command to TRON normalized command
clamp and slew-rate limit
force linear.y = 0 initially
publish zero on timeout
publish geometry_msgs/Twist for TRON1
```

Acceptance: adapter passes unit/smoke tests and simulation stop tests.

## Do Not Do Yet

- Do not edit TRON1 official controller code before testing remapping and bridge options.
- Do not drive the real robot from visual servo output before proving zero-watchdog behavior.
- Do not enable lateral motion until TRON1 model, policy, and real stability are confirmed.
