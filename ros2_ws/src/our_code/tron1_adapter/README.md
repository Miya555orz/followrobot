# tron1_adapter

Placeholder for the future adapter package.

The first real node should be small:

```text
FCR safe cmd_vel: geometry_msgs/TwistStamped
  -> freshness check
  -> physical-to-normalized scaling
  -> clamp and slew-rate limit
  -> force linear.y = 0 for early tests
  -> TRON1 cmd_vel: geometry_msgs/Twist
```

Do not implement hardware behavior here until the TRON1 official simulation confirms `/cmd_vel` type, direction, scaling, and stop behavior.
