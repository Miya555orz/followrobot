# Jetson USB SSH Quickstart - 2026-09-05

Purpose: connect from the PC to the Jetson over the Jetson USB gadget network, while avoiding Mihomo/Clash/TUN route capture. This is a PC-to-Jetson login procedure only; it does not start ROS, does not touch TRON1, and does not publish any motion command.

## Known Good Evidence

Observed on 2026-09-05:

```text
Jetson USB IP: 192.168.55.1
PC USB gadget interface: enx964a2d124484
PC USB gadget address: 192.168.55.100/24
Bad captured route: 192.168.55.1 via 198.18.0.2 dev Mihomo table 2022
Good direct route: 192.168.55.1 dev enx964a2d124484 src 192.168.55.100
```

If the interface name changes, use the actual `enx...` or `usb...` device shown by `ip -brief addr`.

## 0. Physical Setup

1. Power on the Jetson and wait for boot.
2. Connect the PC to the Jetson with a USB data cable.
3. Do not connect this step to any TRON1 motion test.

## 1. Diagnose Whether Mihomo Captured the Route

Run on the PC:

```bash
JETSON_IP=192.168.55.1

ip -brief addr
ip route get $JETSON_IP
ping -c 1 -W 1 $JETSON_IP
```

Bad result, stop and fix the route:

```text
192.168.55.1 via 198.18.0.2 dev Mihomo table 2022
```

Good result:

```text
192.168.55.1 dev enx... src 192.168.55.100
```

## 2. Find the Jetson USB NIC

Run on the PC:

```bash
ip -brief link
ip -brief addr | grep -E 'usb|enx|192\.168\.55|eth|enp|eno|ens'
nmcli device status
```

Known good example:

```text
enx964a2d124484  UNKNOWN  192.168.55.100/24
```

If no `enx...` or `usb...` interface appears, replug the USB data cable and wait for the Jetson to finish booting.

## 3. Force the Jetson IP Through the USB NIC

Use the actual interface from the previous step.

```bash
JETSON_IP=192.168.55.1
JETSON_USB_IFACE=enx964a2d124484

sudo ip link set $JETSON_USB_IFACE up
sudo ip addr add 192.168.55.100/24 dev $JETSON_USB_IFACE 2>/dev/null || true
sudo ip route replace ${JETSON_IP}/32 dev $JETSON_USB_IFACE metric 5

ip route get $JETSON_IP
ping -c 1 -W 1 $JETSON_IP
```

Expected:

```text
192.168.55.1 dev enx964a2d124484 src 192.168.55.100
1 packets transmitted, 1 received, 0% packet loss
```

If `ip route get` still shows `Mihomo`, `tun*`, `tap*`, `wg*`, or `table 2022`, do not SSH yet.

## 4. Fix Stale SSH Host Key If Needed

If SSH prints `REMOTE HOST IDENTIFICATION HAS CHANGED`, remove the old key for this USB IP:

```bash
JETSON_IP=192.168.55.1
ssh-keygen -f "$HOME/.ssh/known_hosts" -R "$JETSON_IP"
```

Then reconnect and accept the new key only when you are physically connected to the expected Jetson:

```bash
ssh -F /dev/null \
  -o ProxyCommand=none \
  -o ConnectTimeout=5 \
  miya@$JETSON_IP
```

Observed ED25519 fingerprint on 2026-09-05:

```text
SHA256:WBIdwRD+Z4G1sFCUDRLAVF/w/qF6Gw8ZG8pRfrHvtMQ
```

The fingerprint may legitimately change after a Jetson reflash. Treat unexpected changes as a reason to re-check the physical USB connection and route before accepting.

## 5. SSH Command for Normal Use

Run from the PC after the route is direct:

```bash
JETSON_IP=192.168.55.1

ssh -F /dev/null \
  -o ProxyCommand=none \
  -o ConnectTimeout=5 \
  miya@$JETSON_IP
```

`-F /dev/null` and `ProxyCommand=none` avoid accidental SSH config proxying.

## 6. After Login

For the next TRON1 network-only phase, remain read-only:

```bash
hostname
ip -brief addr
ip route get 10.192.1.2
ping -c 1 -W 1 10.192.1.2
```

Do not run:

```text
ros2 launch ...
ros2 run robot_hw ...
ros2 topic pub ...
enable_motion:=true
L1 + Y / triangle controller activation
```

USB SSH success only proves PC-to-Jetson login. It is not TRON1 movement permission.
