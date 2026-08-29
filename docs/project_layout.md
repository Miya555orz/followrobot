# Project Layout

## Goal

Keep upstream code, our migration code, documentation, and Linux build outputs separate.

## Chosen Structure

```text
D:/followrobot
  upstream/
    cuiangA/
      fcr_ros2_3/
    limxdynamics/
      limxsdk-lowlevel/
      tron1-rl-deploy-ros2/
      tron1-gazebo-ros2/
      tron1-robot-description/
      robot-visualization/
      ros2-bridger/
      robot-joystick/
  ros2_ws/
    src/
      our_code/
        tron1_adapter/
  docs/
  scripts/
  tests/
  notes/
```

## Policy

- `upstream/` is read-only reference material. Keep it out of this workspace Git history.
- `ros2_ws/src/our_code/` is where new migration packages will live.
- Do not put modified copies directly inside `upstream/`.
- If an upstream patch becomes necessary, record it in `docs/migration_plan.md` first, then apply it as a small patch branch or wrapper package.
- `ros2_ws/build`, `ros2_ws/install`, and `ros2_ws/log` are generated on Linux and ignored.

## Why Not Submodules Yet

Submodules are useful after the upstream revisions are chosen, but they add friction while the project is still exploring ROS 2 Humble versus Iron and TRON1 model variants. For now, normal clones or snapshots under `upstream/` are easier to inspect and safer to discard.

## Current Local Notes

- `upstream/cuiangA/fcr_ros2_3/` contains the FCR snapshot used for static analysis.
- `fcr_ros2_3/` and `upstream/limxdynamics/limxsdk-lowlevel/` may appear if a Windows `git clone` was interrupted. They are ignored and should not be treated as source until recloned successfully.
