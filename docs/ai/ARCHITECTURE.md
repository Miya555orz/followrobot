# Architecture

Current verified chain:

```text
Sony camera
  -> Jetson
  -> perception
  -> tracking
  -> follow / visual servo controller
  -> DJI RS2 gimbal
```

TRON1 target chain:

```text
Sony / Depth Camera
  -> Jetson
  -> Perception / Tracking
  -> Follow / Servo Controller
  -> Unified Base Interface
  -> TRON1 Adapter / Safety Limiter
  -> TRON1 ROS2 / SDK
  -> TRON1 EDU
```

Full actuator split:

```text
Target Tracking
  -> Follow Controller
      -> Gimbal Command -> RS2 Adapter -> DJI RS2
      -> Base Command   -> Base Interface -> TRON1 Adapter -> TRON1 EDU
```

Keep base-specific logic behind adapters, launch files, and config. Do not mix TRON1 SDK or motor details into perception or tracking.

