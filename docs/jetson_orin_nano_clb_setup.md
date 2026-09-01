# Jetson Orin Nano CLB 开发者套件环境配置流程

本文面向“不是 microSD 直接烧录，而是通过 Type-C USB 连接 Ubuntu 主机电脑刷写系统”的 Jetson Orin Nano CLB 开发者套件。

当前项目建议目标环境：

```text
JetPack: 6.x，优先 JetPack 6.2.1 或 SDK Manager 中可选的稳定 6.x
Jetson Linux: L4T 36.x
Ubuntu: 22.04
ROS 2: Humble
项目: followrobot / fcr_ros2_3
```

参考官方文档：

- NVIDIA Jetson Orin Nano Developer Kit BSP setup
- NVIDIA SDK Manager
- NVIDIA JetPack SDK

> 注意：CLB 套件可能使用非 NVIDIA 原厂载板。刷机前最好保留卖家/厂商给的资料、BSP 或说明书。如果 SDK Manager 不能识别或刷写失败，优先确认 CLB 载板是否要求专用 BSP。

## 总流程

```text
主机电脑 Ubuntu 22.04
  ↓ 安装 NVIDIA SDK Manager
Jetson 进入 Force Recovery Mode
  ↓ Type-C 连接主机，lsusb 看到 NVIDIA APX
SDK Manager 刷 JetPack / Jetson Linux
  ↓ Jetson 首次开机配置用户、网络
Jetson 上运行 post-flash setup 脚本
  ↓ 安装 ROS2、colcon、can-utils、基础依赖
clone followrobot
  ↓ 编译核心包，测试 RS2 CAN
```

## A0：主机电脑准备

在你当前 Ubuntu 主机电脑上执行：

```bash
lsb_release -a
uname -m
df -h
```

要求：

- Ubuntu 20.04 或 22.04 x86_64，推荐 22.04。
- 剩余磁盘最好大于 50GB。
- 网络稳定。
- 准备 NVIDIA Developer 账号。

## A1：安装 SDK Manager

Ubuntu 22.04 主机：

```bash
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install sdkmanager
```

启动：

```bash
sdkmanager
```

第一次启动会要求登录 NVIDIA Developer 账号。

如果图形界面一打开就崩溃，但命令行可用，可以改走 CLI 路线。先检查：

```bash
sdkmanager --ver
sdkmanager --help
```

如果这两个命令能输出版本和帮助，说明 SDK Manager 主程序可用，只是 GUI 崩溃。此时不用继续和图形界面纠缠，后续用：

```bash
sdkmanager --cli --login-type devzone
```

CLI 会显示一个 NVIDIA 登录链接和 user code。用浏览器打开它并登录 NVIDIA Developer 账号，登录完成后终端会继续。

> 2026-09-01 本机记录：`sdkmanager` 版本 `2.4.1.13536`，`sdkmanager-gui` 会段错误，但 `sdkmanager --help` 和 CLI 登录流程可用。

## A2：Jetson 进入 Recovery Mode

这一步以 CLB 开发者套件说明书为准。通常流程是：

```text
1. Jetson 断电
2. 用 Type-C 数据线连接 Jetson 的刷机 Type-C 口和 Ubuntu 主机
3. 按住/短接 Recovery/FC REC
4. 上电
5. 松开 Recovery/FC REC
```

然后在主机电脑上检查：

```bash
lsusb | grep -i nvidia
```

期望看到类似：

```text
NVIDIA Corp. APX
```

如果看不到：

- 换 Type-C 数据线，很多充电线不能传数据。
- 确认接的是刷机 Type-C 口，不是普通供电口。
- 确认 Recovery 进入方式和 CLB 说明书一致。
- 断电重来，不要只重插 USB。

## A3：SDK Manager 刷机选择

SDK Manager 中建议：

```text
Product Category: Jetson
Target Hardware: Jetson Orin Nano / Orin Nano Developer Kit
JetPack: 6.x 稳定版
Host Machine: 可以先不选
Target Components:
  - Jetson OS: 选
  - Jetson SDK Components: 可以选；如果空间/网络不稳，也可以刷完系统后在 Jetson 上 apt install nvidia-jetpack
```

存储位置：

- 如果 CLB 套件带 NVMe SSD，优先刷 NVMe。
- 如果是 eMMC 模组，选择对应 internal/eMMC。
- 如果不确定，暂停，把 SDK Manager 的 storage 选项截图给我。

刷写完成后，断电重启 Jetson，接显示器/键鼠，完成首次 Ubuntu 用户配置。

## A3-CLI：GUI 崩溃时的命令行刷机路线

先让 Jetson 进入 Recovery Mode，并确认主机能看到 NVIDIA APX：

```bash
lsusb | grep -i nvidia
```

然后查询 SDK Manager 识别到的 Jetson 设备：

```bash
sdkmanager --list-connected Jetson
```

查询可用 Jetson 安装选项：

```bash
sdkmanager --query interactive --product Jetson --login-type devzone
```

这个命令会交互式询问产品、版本、目标硬件、刷机选项，并最终生成对应的安装命令。不要在不确定 Target Hardware / Storage Device 时直接确认；把终端输出贴给我，我再判断。

## B0：Jetson 首次开机检查

在 Jetson 上执行：

```bash
hostname
uname -a
cat /etc/nv_tegra_release
lsb_release -a
df -h
free -h
ip -brief address
```

## B1：Jetson 安装项目基础环境

在 Jetson 上运行：

```bash
cd ~
git clone -b main https://github.com/Miya555orz/followrobot.git
cd ~/followrobot
bash tools/tron1_bringup/jetson_post_flash_setup.sh
```

脚本做的事：

- 安装 ROS 2 Humble apt 源；
- 安装 `ros-humble-desktop`、`colcon`、`rosdep`；
- 安装 `can-utils`、`git`、`python3-pip` 等基础工具；
- 初始化 `rosdep`；
- 配置 `~/.bashrc` 自动 source ROS 2；
- 给出下一步编译命令。

## B2：clone 到 ROS 工作区

如果你希望保持和笔记本同样目录结构：

```bash
mkdir -p ~/follow_ws/src
cd ~/follow_ws/src
git clone -b main https://github.com/Miya555orz/followrobot.git fcr_ros2_3
```

然后：

```bash
source /opt/ros/humble/setup.bash
cd ~/follow_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
  --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg
```

## B3：RS2 CAN 识别

插入 USB-CAN 和 RS2 后，在 Jetson 上执行：

```bash
ip -details link show
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg --ctime | tail -120
```

判断：

```text
看到 can0        → SocketCAN 路线
看到 ttyUSB0     → 串口型 USB-CAN，需要确认型号和驱动
都没有           → 线、供电、驱动或设备识别问题
```

如果是 `can0`：

```bash
sudo apt install -y can-utils
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
ip -details link show can0
candump can0
```

`candump can0` 会一直等待。按 `Ctrl+C` 退出。

## B4：FCR RS2 节点

只有在 CAN 设备已经明确后再跑：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 launch robot_platform_pkg gimbal_bringup.launch.py
```

另开终端：

```bash
source /opt/ros/humble/setup.bash
source ~/follow_ws/install/setup.bash
ros2 topic list | rg 'gimbal|cmd_gimbal|platform'
ros2 node list | rg 'gimbal|platform'
```

## 常见坑

### 1. 主机 `lsusb` 看不到 NVIDIA APX

大概率是：

- Type-C 线不是数据线；
- Recovery Mode 没进；
- 接错 Type-C 口；
- Jetson 没有正确上电；
- CLB 载板 Recovery 按键/跳线方式和原厂不同。

### 2. SDK Manager 识别不到板子

先确认：

```bash
lsusb | grep -i nvidia
```

如果没有 APX，SDK Manager 也不会识别。

### 3. Jetson 空间不足

优先用 NVMe SSD。JetPack + ROS2 + 项目依赖会占不少空间。

### 4. ROS2 能装但 FCR 编译失败

先只编译核心包：

```bash
MAKEFLAGS="-j1 -l1" colcon build --symlink-install --parallel-workers 1 \
  --packages-select robot_platform_pkg teleop_control_pkg bringup_pkg
```

Sony 相关失败暂时可以接受；我们当前硬件没有 Sony。

### 5. can-utils 没有安装

```bash
sudo apt update
sudo apt install -y can-utils
```

## 你需要贴给我的输出

每走完一个节点，把输出贴回来：

```bash
lsusb | grep -i nvidia
```

Jetson 首次开机后：

```bash
cat /etc/nv_tegra_release
lsb_release -a
ip -brief address
```

CAN 检查：

```bash
ip -details link show
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
dmesg --ctime | tail -120
```
