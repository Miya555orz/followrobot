# TRON1 Jetson Network Setup Repeatable Flow - 2026-09-05

Scope: make the Jetson-to-TRON1 Ethernet route repeatable after the 2026-09-05 network-only PASS. This document covers only IP link/address/route setup. It is not controller bringup and not motion permission.

## Current Verified Values

```text
TRON_IP=10.192.1.2
TRON_IFACE=enP8p1s0
TRON_NET=10.192.1.0/24
JETSON_TRON_ADDR=10.192.1.200
Expected route: 10.192.1.2 dev enP8p1s0 src 10.192.1.200
Expected preflight: PASS=2 WARN=0 BLOCK=0 FAIL=0
```

Keep `TRON_NET=10.192.1.0/24` and `JETSON_TRON_ADDR=10.192.1.200` unless the hardware network plan is explicitly changed and reviewed.

## Recommended Short-Term Flow

Use the temporary route helper on the Jetson. This is the default recommended flow for the next session. It defaults to dry-run:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3  # or the Jetson repo directory if different
TRON_IFACE=enP8p1s0 ./tools/tron1_bringup/jetson_tron1_route_setup.sh --dry-run
```

After checking the printed commands:

```bash
cd /home/miya/follow_ws/src/fcr_ros2_3  # or the Jetson repo directory if different
CONFIRM_TRON1_ROUTE_SETUP=yes TRON_IFACE=enP8p1s0 ./tools/tron1_bringup/jetson_tron1_route_setup.sh --apply
TRON_LINK_IFACE=enP8p1s0 ./tools/tron1_bringup/jetson_tron1_network_preflight.sh
```

Temporary-route rollback, if the manual setup needs to be removed before reboot:

```bash
TRON_IFACE=enP8p1s0
sudo ip route del 10.192.1.0/24 dev "$TRON_IFACE" src 10.192.1.200 metric 10 2>/dev/null || true
sudo ip addr del 10.192.1.200/24 dev "$TRON_IFACE" 2>/dev/null || true
TRON_LINK_IFACE="$TRON_IFACE" ./tools/tron1_bringup/jetson_tron1_network_preflight.sh
```

If the first ping after link setup fails because ARP/link negotiation is still settling, rerun the network-only preflight once; do not continue past a repeated `BLOCK`.

This helper only runs:

```text
sudo ip link set ...
sudo ip addr add ...
sudo ip route replace ...
ip route get ...
ping -c 1 -W 1 ...
```

It does not source ROS, launch `robot_hw`, activate the official controller, or publish velocity.

In `--apply` mode it requires `CONFIRM_TRON1_ROUTE_SETUP=yes`, then re-checks `ip route get 10.192.1.2` and one read-only ping. If a setup command fails, or `10.192.1.200/24` already belongs to another interface, it returns `FAIL=1`; if the route does not use `dev enP8p1s0 src 10.192.1.200`, or ping fails, it returns `BLOCK=3`.

As of this document, the helper's dry-run and fail-closed branches were verified locally; `--apply` still needs one supervised Jetson end-to-end run before marking the repeatable flow fully verified.

## Optional NetworkManager Profile

Use this only if the temporary flow is too annoying to repeat, and only after a separate human review/test. Keep `autoconnect no` so this profile does not silently take over the Ethernet port later.

Because `autoconnect no` is intentional, this profile still requires a manual `nmcli con up tron1-direct` after boot. Before bringing it up, check whether the same interface already has an autoconnecting DHCP/lab-network profile so the TRON1 route is not silently replaced; rerun `jetson_tron1_network_preflight.sh` after any NetworkManager change.

```bash
TRON_IFACE=enP8p1s0

sudo nmcli con add type ethernet ifname "$TRON_IFACE" con-name tron1-direct \
  ipv4.method manual \
  ipv4.addresses 10.192.1.200/24 \
  ipv4.never-default yes \
  ipv6.method disabled \
  connection.autoconnect no

sudo nmcli con modify tron1-direct +ipv4.routes "10.192.1.0/24"
sudo nmcli con up tron1-direct

ip route get 10.192.1.2
ping -c 1 -W 1 10.192.1.2
```

Rollback:

```bash
sudo nmcli con down tron1-direct 2>/dev/null || true
sudo nmcli con delete tron1-direct
```

## Safety Boundary

Allowed:

- `ip -brief addr`
- `ip -brief link`
- `ip route get 10.192.1.2`
- `ping -c 1 -W 1 10.192.1.2`
- `jetson_tron1_route_setup.sh`
- `jetson_tron1_network_preflight.sh`

Forbidden in this phase:

- `ros2 launch`
- `ros2 run robot_hw`
- `ros2 topic pub`
- `enable_motion:=true`
- remote-controller `L1 + Y/triangle`

Network setup success proves only route and ping. It is not a real-motion permit and not a 100% safety guarantee.
