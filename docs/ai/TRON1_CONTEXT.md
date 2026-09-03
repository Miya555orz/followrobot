# TRON1 Context

Official repositories used by this project:

- `https://github.com/limxdynamics/limxsdk-lowlevel`
- `https://github.com/limxdynamics/tron1-robot-description`
- `https://github.com/limxdynamics/tron1-gazebo-ros2`
- `https://github.com/limxdynamics/tron1-rl-deploy-ros2`
- `https://github.com/limxdynamics/tron1-mujoco-sim`
- `https://github.com/limxdynamics/tron1-rl-deploy-python`
- `https://github.com/limxdynamics/tron1-agent`

Local TRON1 workspace:

```text
/home/miya/limx_ws
```

Observed local TRON1 packages:

- `limxsdk_lowlevel`
- `limxsdk_sim`
- `mrosbridger`
- `pointfoot_gazebo`
- `robot_controllers`
- `robot_description`
- `robot_hw`
- `robot_visualization`

Recommended robot type for current wheeled-foot work:

```bash
export ROBOT_TYPE=WF_TRON1A
export RL_TYPE=isaacgym
```

Confirmed high-level control layer:

```text
geometry_msgs/msg/Twist
  linear.x
  linear.y
  angular.z
```

The official controller internally maps this high-level command into the RL locomotion controller and then to SDK/hardware commands. followrobot should integrate at the high-level Twist layer, after safety limiting.

Current safe topic chain:

```text
/fcr/cmd_vel_stamped
  -> robot_platform_pkg/tron1_safety_limiter_node
  -> /fcr_tron/cmd_vel
  -> TRON1 official robot_hw_node
```

Do not use bare `/cmd_vel` for TRON1 hardware.

