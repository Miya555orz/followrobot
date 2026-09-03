# Base Interface + TRON1 Adapter 设计

目标：让 FCR perception / tracking / follow 逻辑与物理底盘解耦。TRON1 专用行为只能存在于 adapter / limiter / bringup 层。

## 目标架构

```text
Perception / Tracking / Follow
          |
          v
统一底盘接口
          |
   +------+------+
   |             |
Omni Adapter   TRON1 Adapter
   |             |
旧底盘         tron1_safety_limiter
                 |
                 v
              /fcr_tron/cmd_vel
                 |
                 v
              TRON1 官方控制器
```

## 当前实际接口

第一版实现继续使用已经验证过的命令类型：

```text
/fcr/cmd_vel_stamped
geometry_msgs/msg/TwistStamped
```

原因：

- FCR 已经通过 command mux 使用 `TwistStamped`。
- TRON1 官方控制器消费的是 `Twist`。
- `tron1_safety_limiter` 已经负责把 FCR stamped 命令桥接到 `/fcr_tron/cmd_vel`。
- 在硬件安全路径证明前，不引入新的消息类型，风险更低。

之后可以再扩展为更丰富的 `vision_servo_msgs/BaseCommand`，但不要让新消息设计阻塞 TRON1 安全 bringup。

## Topic 归属

| 层级 | Topic | 类型 | 归属 |
| --- | --- | --- | --- |
| 上层跟拍输出 | `/fcr/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | command mux / base interface |
| TRON limiter 输入 | `/fcr/cmd_vel_stamped` | `geometry_msgs/TwistStamped` | `tron1_safety_limiter` 订阅 |
| TRON limiter 输出 | `/fcr_tron/cmd_vel` | `geometry_msgs/Twist` | 只能由 `tron1_safety_limiter` 发布 |
| TRON 官方控制 | `/fcr_tron/cmd_vel` | `geometry_msgs/Twist` | 官方 `robot_hw_node` 订阅 |
| 禁止直连路径 | `/cmd_vel` | `geometry_msgs/Twist` | FCR/TRON 集成中不允许使用 |

## Package 边界

近期推荐布局：

```text
src/servo_control_pkg/
  跟随控制器、云台/底盘误差拆分

src/robot_platform_pkg/
  tron1_safety_limiter_node
  tron_cmd_adapter_node
  platform status 和硬件侧 adapter

src/bringup_pkg/
  launch 组合和模式选择
```

不要把 TRON1 专用代码放进：

- `src/perception_pkg`
- 目标检测 / tracking 逻辑
- Sony / Orbbec 相机节点
- 通用 visual servo 数学逻辑

## Adapter 职责

`tron1_adapter` 可以知道：

- 早期测试中 TRON1 不允许使用横向速度。
- TRON1 yaw 应该是低速、长期、慢时间尺度补偿。
- TRON1 需要 `/fcr_tron/cmd_vel`，不能使用裸 `/cmd_vel`。
- 真机默认必须是 `enable_motion=false`。
- 目标丢失必须产生零底盘命令。

`tron1_adapter` 不能：

- 绕过 `tron1_safety_limiter`；
- 直接发布到 `/cmd_vel`；
- 决定视觉目标是谁；
- 命令高速跟随；
- 向上层模式管理器隐藏安全失败。

## 跟拍控制分工

当前最安全的跟拍分工：

```text
RS2 云台：
  负责短时间尺度、小角度目标居中

TRON1 底盘：
  只负责慢速、长期 yaw 回正
  真实低速验收通过后，才加入慢速距离补偿
```

避免云台和底盘两个独立控制器追同一个像素误差。底盘应该消费经过滤波、低频的残余误差，而且要发生在云台完成快速稳定之后。

## Launch 策略

底盘后端应该通过 launch 参数选择，而不是修改 perception / follow 代码。

近期模式：

```text
base_backend:=none
  只启动 perception + gimbal

base_backend:=omni
  旧底盘路径

base_backend:=tron1_sim
  TRON1 仿真 topic 安全路径

base_backend:=tron1_real
  真实 TRON1 路径，默认 enable_motion=false
```

`tron1_real` 建议参数：

```text
enable_motion=false
max_linear_x=0.02~0.03
max_linear_y=0.0
max_angular_z=0.05~0.10
```

## 实现前验收

把跟拍逻辑接入 TRON1 前，至少确认：

- [ ] `/fcr_tron/cmd_vel` 只有一个发布者：`tron1_safety_limiter`。
- [ ] 官方控制器订阅 `/fcr_tron/cmd_vel`。
- [ ] `enable_motion=false` 时，所有上游命令都会输出 0。
- [ ] timeout 后输出回到 0。
- [ ] estop 后输出回到 0。
- [ ] limiter 关闭时发布零速度 burst。
- [ ] 命令发布者消失 / limiter 崩溃后的行为已记录。
- [ ] 硬件阻尼 / 停止路径已理解。

## Gazebo 注意事项

`WF_TRON1A + isaacgym` Gazebo 当前存在零命令漂移，纯 yaw 命令也会产生横向位移。`RL_TYPE=isaaclab` 已测试，漂移更明显。轻量控制器和 URDF 摩擦实验已撤回。

因此：

- 使用 Gazebo 验证 launch、topic 接线、limiter、timeout、estop 和命令方向；
- 不把 Gazebo pose hold 作为真实机器人安全通过/失败标准；
- 不围绕 Gazebo 漂移去调 FCR 跟拍参数。

## 下一步实现

先实现最小可用接口：

1. 保持 `/fcr/cmd_vel_stamped` 作为统一命令 topic。
2. 让 `bringup_pkg` 选择该 topic 是不接底盘、接旧 omni 底盘，还是接 TRON1 limiter。
3. 添加 `base_backend` launch 参数。
4. 每一种 TRON1 模式都必须强制经过 `tron1_safety_limiter`。
5. 添加验收脚本，在任何实机运动前证明 topic 归属和安全输出。
