# Follow Robot TRON1 Migration Workspace

This directory is the migration control room for moving the original FCR ROS 2 follow-robot project from the LEKIWI three-omni chassis to a TRON1 platform.

## Directory Map

```text
D:/followrobot
  upstream/                  Original upstream code snapshots or clones; ignored by this repo.
    cuiangA/fcr_ros2_3/       Original FCR project snapshot.
    limxdynamics/             TRON1 official repositories; fetch on Linux if Windows git is slow.
  ros2_ws/
    src/
      our_code/               New packages owned by this migration.
        tron1_adapter/        Adapter design placeholder; no ROS package code yet.
  docs/                       Architecture, environment, migration, and workflow notes.
  scripts/                    Setup/fetch helper scripts.
  tests/                      Future integration and smoke tests.
  notes/                      Manual run logs and hardware observations.
```

## Current Rule

Do not edit upstream code directly. Copy or wrap behavior in `ros2_ws/src/our_code/` first, then promote the smallest verified change.

## First Linux Command

On Ubuntu 22.04, start by checking the ROS underlay:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

Use a second terminal to run:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```
