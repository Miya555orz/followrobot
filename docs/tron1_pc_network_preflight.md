# PC -> Jetson/TRON1 Network Preflight

日期：2026-09-03

目标：在继续 TRON1 实机通信前，先确认开发电脑是否真的能走物理网络到 Jetson，而不是被 Mihomo/TUN 或错误路由带偏。

## 当前发现

本机检查时看到：

```text
enp0s31f6: NO-CARRIER / DOWN
172.31.178.242 via 198.18.0.2 dev Mihomo table 2022
```

含义：

- `enp0s31f6` 没有检测到网线链路；这通常是网线未插、对端未开、交换机/转接器没连上。
- 去 Jetson `172.31.178.242` 的路由被 Mihomo/TUN 接管，所以 SSH 不会优先走物理以太网。
- 这不是 Jetson 密码问题。

## 只读预检

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
tools/tron1_bringup/pc_jetson_network_preflight.sh
```

期望：

```text
enp0s31f6 carrier: 1
ip route get 172.31.178.242 -> dev enp0s31f6 或其它真实局域网接口
SSH_OK
```

如果看到：

```text
carrier: 0
```

先处理物理连接，不要改 ROS，也不要改 TRON1 SDK。

## 临时绕过 Mihomo 路由

只有当以太网已经有 carrier、但路由仍然走 Mihomo/TUN 时，再运行：

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3
JETSON_IP=172.31.178.242 ETH_IF=enp0s31f6 tools/tron1_bringup/pc_jetson_network_preflight.sh --fix-route
```

这个操作只添加临时 `ip rule`，不会写 NetworkManager 配置，重启会失效。

## 成功后再做 Jetson 侧只读检查

```bash
ssh miya@172.31.178.242 '
hostname
date
ip -brief addr
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 pkg list | grep -E "^(vision_servo_msgs|robot_platform_pkg|teleop_control_pkg|orbbec_camera|orbbec_camera_msgs|sony_camera_pkg)$" | sort
'
```

不要在 SSH 未确认走物理网络前调 TRON1 实机、CAN、udev、systemd 或 SDK 运动命令。

