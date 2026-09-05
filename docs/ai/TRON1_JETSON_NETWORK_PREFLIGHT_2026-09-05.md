# TRON1 Jetson Network-Only Preflight Record - 2026-09-05

Scope: read-only Jetson-to-TRON1 Ethernet connectivity check. This test did not source ROS, did not launch `robot_hw`, did not activate the official controller, and did not publish any velocity command.

## Physical Topology

```text
PC --USB/SSH--> Jetson
Jetson --Ethernet--> TRON1
```

Observed hosts and interfaces:

```text
Jetson hostname: ubuntu
TRON_IP: 10.192.1.2
Jetson TRON1 Ethernet interface: enP8p1s0
Jetson TRON1 source address used for the route: 10.192.1.200
Jetson USB bridge: l4tbr0 = 192.168.55.1/24
```

## Initial Bad Route

Before the Jetson TRON1 Ethernet route was corrected, `10.192.1.2` was routed back toward the PC USB bridge:

```text
10.192.1.2 via 192.168.55.100 dev l4tbr0 src 192.168.55.1
ping: 1 packets transmitted, 0 received, 100% packet loss
```

This was not a TRON1 connectivity pass.

## Corrected Read-Only Route

The Jetson interface was configured only at the IP route level:

```bash
TRON_IFACE=enP8p1s0
TRON_IP=10.192.1.2
JETSON_TRON_ADDR=10.192.1.200

sudo ip link set $TRON_IFACE up
sudo ip addr add ${JETSON_TRON_ADDR}/24 dev $TRON_IFACE 2>/dev/null || true
sudo ip route replace 10.192.1.0/24 dev $TRON_IFACE src $JETSON_TRON_ADDR metric 10
```

Observed route and ping:

```text
10.192.1.2 dev enP8p1s0 src 10.192.1.200 uid 1000
PING 10.192.1.2: 1 packets transmitted, 1 received, 0% packet loss
rtt min/avg/max/mdev = 0.352/0.352/0.352/0.000 ms
```

## Scripted Preflight Result

The Jetson copy of `tools/tron1_bringup/jetson_tron1_network_preflight.sh` was installed over PC USB SSH because the Jetson could not resolve `github.com`.

Observed scripted result:

```text
TRON_LINK_IFACE=enP8p1s0 ./tools/tron1_bringup/jetson_tron1_network_preflight.sh
[PASS] TRON_IP route uses direct wired-looking dev enP8p1s0.
[PASS] TRON_IP responds to one read-only ping.
Summary: PASS=2 WARN=0 BLOCK=0 FAIL=0
```

## Interpretation

This proves the Jetson-to-TRON1 Ethernet path is reachable in a network-only check. It is not a real-motion permit, not controller bringup, and not a 100% safety guarantee.

Continue to keep real motion paused. Do not proceed to any movement until the A-10 physical stop/damping/controller-watchdog items, Gazebo zero-drift blocker, controller/SDK/hardware-stop review, and staged real-test checklist are complete in a suitable protected space.
