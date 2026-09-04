#!/usr/bin/env python3
"""TRON1 安全模式管理 20 组仿真验收。

默认只启动 FCR 侧 `tron1_mode_manager_node + tron1_safety_limiter_node`，
验证状态机、授权、限速、急停、timeout。它不会连接真实 TRON1。

如需同时启动官方 Gazebo/robot_hw_sim，可加 `--with-gazebo`。
"""

from __future__ import annotations

import argparse
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Callable

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


TRANSIENT_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


@dataclass
class CaseResult:
    name: str
    passed: bool
    detail: str


class SafeModeProbe(Node):
    def __init__(self) -> None:
        super().__init__("tron1_safe_mode_acceptance_probe")
        self.mode_request_pub = self.create_publisher(String, "/tron1/mode_request", 10)
        self.estop_pub = self.create_publisher(Bool, "/safety/estop_state", TRANSIENT_QOS)
        self.limiter_estop_clear_pub = self.create_publisher(
            Bool, "/tron1/limiter_clear_estop", 10
        )
        self.cmd_pub = self.create_publisher(TwistStamped, "/fcr/cmd_vel_stamped", 10)

        self.last_state: str | None = None
        self.last_auth: bool | None = None
        self.last_limiter_state: str | None = None
        self.outputs: list[tuple[float, float, float]] = []
        self._estop_state = False
        self._last_estop_heartbeat = 0.0
        self._estop_heartbeat_enabled = True

        self.create_subscription(String, "/tron1/mode_state", self._on_state, TRANSIENT_QOS)
        self.create_subscription(Bool, "/tron1/motion_authorized", self._on_auth, TRANSIENT_QOS)
        self.create_subscription(String, "/tron1/limiter_state", self._on_limiter_state, TRANSIENT_QOS)
        self.create_subscription(TwistStamped, "/unused", lambda _: None, 10)
        self.create_subscription(Twist, "/fcr_tron/cmd_vel", self._on_output, 10)

    def _on_state(self, msg: String) -> None:
        self.last_state = msg.data

    def _on_auth(self, msg: Bool) -> None:
        self.last_auth = bool(msg.data)

    def _on_limiter_state(self, msg: String) -> None:
        self.last_limiter_state = msg.data

    def _on_output(self, msg) -> None:
        self.outputs.append((msg.linear.x, msg.linear.y, msg.angular.z))
        self.outputs = self.outputs[-200:]

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            now = time.monotonic()
            if self._estop_heartbeat_enabled and now - self._last_estop_heartbeat >= 0.05:
                self.estop_pub.publish(Bool(data=self._estop_state))
                self._last_estop_heartbeat = now
            rclpy.spin_once(self, timeout_sec=0.02)

    def request(self, text: str) -> None:
        msg = String()
        msg.data = text
        for _ in range(3):
            self.mode_request_pub.publish(msg)
            self.spin_for(0.05)

    def estop(self, active: bool) -> None:
        self._estop_heartbeat_enabled = True
        self._estop_state = active
        msg = Bool()
        msg.data = active
        for _ in range(3):
            self.estop_pub.publish(msg)
            self.spin_for(0.05)

    def set_estop_heartbeat_enabled(self, enabled: bool) -> None:
        self._estop_heartbeat_enabled = enabled
        self._last_estop_heartbeat = 0.0

    def clear_limiter_estop(self) -> None:
        msg = Bool()
        msg.data = True
        for _ in range(3):
            self.limiter_estop_clear_pub.publish(msg)
            self.spin_for(0.05)

    def cmd(self, x: float, y: float, yaw: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "acceptance"
        msg.twist.linear.x = x
        msg.twist.linear.y = y
        msg.twist.angular.z = yaw
        self.cmd_pub.publish(msg)

    def publish_cmd_for(self, x: float, y: float, yaw: float, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.cmd(x, y, yaw)
            self.spin_for(0.05)

    def reset_outputs(self) -> None:
        self.outputs.clear()

    def max_abs_output(self) -> tuple[float, float, float]:
        if not self.outputs:
            return (0.0, 0.0, 0.0)
        return (
            max(abs(v[0]) for v in self.outputs),
            max(abs(v[1]) for v in self.outputs),
            max(abs(v[2]) for v in self.outputs),
        )

    def last_output_near_zero(self, eps: float = 1e-4) -> bool:
        if not self.outputs:
            return True
        x, y, yaw = self.outputs[-1]
        return abs(x) <= eps and abs(y) <= eps and abs(yaw) <= eps


def start_process(cmd: list[str], env: dict[str, str]) -> subprocess.Popen:
    return subprocess.Popen(
        cmd,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        preexec_fn=os.setsid,
    )


def stop_process(proc: subprocess.Popen | None) -> None:
    if proc is None or proc.poll() is not None:
        return
    os.killpg(os.getpgid(proc.pid), signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
        proc.wait(timeout=8)


def matching_processes(pattern: str) -> list[str]:
    proc = subprocess.run(
        ["pgrep", "-af", pattern],
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=False,
    )
    if proc.returncode != 0:
        return []
    return [
        line
        for line in proc.stdout.splitlines()
        if "tron1_safe_mode_acceptance.py" not in line and "pgrep -af" not in line
    ]


def robot_network_reachable(ip: str) -> bool:
    proc = subprocess.run(
        ["ping", "-c", "1", "-W", "1", ip],
        text=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    return proc.returncode == 0


def real_robot_guard(allow_robot_network: bool, robot_ip: str) -> tuple[bool, str]:
    risky_processes = matching_processes(
        r"pointfoot_node|/robot_hw_node|robot_hw_node|pointfoot_hw.launch.py"
    )
    if risky_processes:
        return False, "发现可能连接真机的残留进程：\n" + "\n".join(risky_processes)
    if not allow_robot_network and robot_network_reachable(robot_ip):
        return (
            False,
            f"检测到 TRON1 地址 {robot_ip} 可达；自动仿真验收默认拒跑，"
            "避免同 domain 误驱动真机。确认完全隔离后可显式加 --allow-robot-network。",
        )
    return True, "未发现真机进程；TRON1 网络未授权接入自动验收"


def check(name: str, fn: Callable[[], tuple[bool, str]]) -> CaseResult:
    try:
        passed, detail = fn()
    except Exception as exc:  # noqa: BLE001
        return CaseResult(name, False, f"异常：{exc}")
    return CaseResult(name, passed, detail)


def wait_until(probe: SafeModeProbe, predicate: Callable[[], bool], timeout: float) -> bool:
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        probe.spin_for(0.05)
        if predicate():
            return True
    return False


def run_cases(probe: SafeModeProbe, require_robot_hw_subscriber: bool) -> list[CaseResult]:
    results: list[CaseResult] = []
    tol = 1e-3

    def output_has_single_limiter_publisher() -> tuple[bool, str]:
        infos = probe.get_publishers_info_by_topic("/fcr_tron/cmd_vel")
        names = [info.node_name for info in infos]
        ok = len(names) == 1 and names[0] == "tron1_safety_limiter"
        return ok, f"publishers={names}"

    def robot_hw_subscribes_output() -> tuple[bool, str]:
        if not require_robot_hw_subscriber:
            infos = probe.get_subscriptions_info_by_topic("/fcr_tron/cmd_vel")
            names = [info.node_name for info in infos]
            return True, f"跳过：未请求 --with-gazebo；subscribers={names}"
        ok = wait_until(
            probe,
            lambda: any(
                info.node_name in ("cmd_vel_node", "robot_hw_node")
                for info in probe.get_subscriptions_info_by_topic("/fcr_tron/cmd_vel")
            ),
            8.0,
        )
        infos = probe.get_subscriptions_info_by_topic("/fcr_tron/cmd_vel")
        names = [info.node_name for info in infos]
        return ok, f"subscribers={names}"

    def kill_mode_manager() -> tuple[bool, str]:
        before = subprocess.run(
            ["pgrep", "-af", "[t]ron1_mode_manager_node"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        proc = subprocess.run(
            ["pkill", "-f", "[t]ron1_mode_manager_node"],
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
        )
        after = None
        end = time.monotonic() + 2.0
        while time.monotonic() < end:
            probe.spin_for(0.05)
            after = subprocess.run(
                ["pgrep", "-af", "[t]ron1_mode_manager_node"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
            if after.returncode != 0:
                break
        if after is not None and after.returncode == 0:
            subprocess.run(["pkill", "-TERM", "-f", "[t]ron1_mode_manager_node"], check=False)
            probe.spin_for(0.3)
            after = subprocess.run(
                ["pgrep", "-af", "[t]ron1_mode_manager_node"],
                text=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                check=False,
            )
        ok = before.returncode == 0 and proc.returncode == 0 and after is not None and after.returncode != 0
        return ok, (
            f"before={before.stdout.strip()!r}, "
            f"pkill_return={proc.returncode}, after={after.stdout.strip() if after else '<none>'!r}"
        )

    def state_is(expected: str) -> tuple[bool, str]:
        ok = wait_until(probe, lambda: probe.last_state == expected, 1.0)
        return ok, f"state={probe.last_state}, expected={expected}"

    def auth_is(expected: bool) -> tuple[bool, str]:
        ok = wait_until(probe, lambda: probe.last_auth is expected, 1.0)
        return ok, f"motion_authorized={probe.last_auth}, expected={expected}"

    def limiter_state_is(expected: str) -> tuple[bool, str]:
        ok = wait_until(probe, lambda: probe.last_limiter_state == expected, 1.0)
        return ok, f"limiter_state={probe.last_limiter_state}, expected={expected}"

    probe.estop(False)
    probe.clear_limiter_estop()
    probe.request("reset")
    results.append(
        check("00 /fcr_tron/cmd_vel 只有 limiter 一个发布者", output_has_single_limiter_publisher)
    )
    results.append(
        check("00b 官方 robot_hw 订阅 limiter 输出", robot_hw_subscribes_output)
    )
    results.append(check("01 初始/复位后进入 IDLE", lambda: state_is("IDLE")))
    results.append(check("02 IDLE 不授权运动", lambda: auth_is(False)))
    results.append(
        check(
            "02b limiter 状态显示未授权阻塞",
            lambda: limiter_state_is("BLOCKED_MOTION_NOT_AUTHORIZED"),
        )
    )

    probe.reset_outputs()
    probe.publish_cmd_for(0.5, 0.5, 0.5, 0.4)
    results.append(
        check(
            "03 IDLE 下输入大速度仍输出零",
            lambda: (
                probe.last_output_near_zero(),
                f"last_output={probe.outputs[-1] if probe.outputs else None}",
            ),
        )
    )

    probe.request("tron_follow")
    results.append(check("04 非法跳转到 TRON_FOLLOW 被拒绝", lambda: state_is("IDLE")))
    results.append(check("05 非法跳转后仍不授权", lambda: auth_is(False)))

    sequence = [
        ("developer_mode", "DEVELOPER_MODE", "06 进入开发者模式"),
        ("developer_self_check_pass", "DEVELOPER_SELF_CHECK", "07 开发者算法自检通过"),
        ("stand_ready", "REMOTE_STAND_READY", "08 进入同款蹲起/站立准备"),
        ("walk_ready", "REMOTE_WALK_READY", "09 进入同款行走准备"),
    ]
    for req, expected, name in sequence:
        probe.request(req)
        results.append(check(name, lambda expected=expected: state_is(expected)))

    results.append(check("10 行走准备态默认仍不授权运动", lambda: auth_is(False)))
    probe.reset_outputs()
    probe.publish_cmd_for(0.5, 0.5, 0.5, 0.4)
    results.append(
        check(
            "11 行走准备态大速度仍输出零",
            lambda: (
                probe.last_output_near_zero(),
                f"last_output={probe.outputs[-1] if probe.outputs else None}",
            ),
        )
    )

    sequence2 = [
        ("device_self_check_pass", "DEVICE_SELF_CHECK", "12 设备自检通过"),
        ("gimbal_follow", "GIMBAL_FOLLOW", "13 云台进入跟随模式"),
    ]
    for req, expected, name in sequence2:
        probe.request(req)
        results.append(check(name, lambda expected=expected: state_is(expected)))
    results.append(check("14 云台跟随态仍不授权 TRON 运动", lambda: auth_is(False)))

    probe.request("tron_follow")
    results.append(check("15 进入 TRON 跟随态", lambda: state_is("TRON_FOLLOW")))
    results.append(check("16 TRON 跟随态授权运动", lambda: auth_is(True)))

    probe.reset_outputs()
    probe.publish_cmd_for(1.0, 1.0, 1.0, 1.1)
    results.append(
        check(
            "16b limiter 状态显示正在尝试放行限幅命令",
            lambda: limiter_state_is("INTENT_PASSING_LIMITED_CMD"),
        )
    )
    max_x, max_y, max_yaw = probe.max_abs_output()
    results.append(
        check(
            "17 TRON 跟随态 linear.x 被限幅",
            lambda: (max_x <= 0.03 + tol and max_x > 0.005, f"max_x={max_x:.4f}"),
        )
    )
    results.append(
        check(
            "18 TRON 跟随态 linear.y 被强制为零",
            lambda: (max_y <= tol, f"max_y={max_y:.4f}"),
        )
    )
    results.append(
        check(
            "19 TRON 跟随态 angular.z 被限幅",
            lambda: (max_yaw <= 0.10 + tol and max_yaw > 0.005, f"max_yaw={max_yaw:.4f}"),
        )
    )

    probe.spin_for(0.45)
    results.append(
        check(
            "20 输入 timeout 后输出自动归零",
            lambda: (
                probe.last_output_near_zero(),
                f"last_output={probe.outputs[-1] if probe.outputs else None}",
            ),
        )
    )
    results.append(
        check("20b limiter 状态显示输入 timeout", lambda: limiter_state_is("BLOCKED_INPUT_TIMEOUT"))
    )

    probe.reset_outputs()
    probe.set_estop_heartbeat_enabled(False)
    probe.publish_cmd_for(0.03, 0.0, 0.05, 0.8)
    results.append(
        check(
            "20c estop 样本超时后继续发命令仍输出零",
            lambda: (
                probe.last_output_near_zero(),
                f"last_output={probe.outputs[-1] if probe.outputs else None}",
            ),
        )
    )
    results.append(
        check("20d limiter 状态显示 estop 样本超时", lambda: limiter_state_is("BLOCKED_ESTOP_TIMEOUT"))
    )
    probe.estop(False)
    probe.clear_limiter_estop()

    probe.publish_cmd_for(0.3, 0.0, 0.0, 0.3)
    probe.estop(True)
    results.append(check("21 外部急停强制进入 ESTOP", lambda: state_is("ESTOP")))
    results.append(check("22 外部急停后取消运动授权", lambda: auth_is(False)))
    results.append(
        check("22b limiter 状态显示急停锁存", lambda: limiter_state_is("BLOCKED_ESTOP_LATCHED"))
    )
    results.append(
        check(
            "23 外部急停后输出归零",
            lambda: (
                probe.last_output_near_zero(),
                f"last_output={probe.outputs[-1] if probe.outputs else None}",
            ),
        )
    )

    probe.request("tron_follow")
    results.append(check("24 急停锁存时拒绝继续进入 TRON_FOLLOW", lambda: state_is("ESTOP")))
    probe.request("clear_estop")
    results.append(check("25 外部急停未解除时 clear_estop 无效", lambda: state_is("ESTOP")))
    results.append(check("26 外部急停未解除时仍不授权", lambda: auth_is(False)))
    probe.estop(False)
    probe.clear_limiter_estop()
    probe.request("clear_estop")
    results.append(check("27 clear_estop 后回到 IDLE", lambda: state_is("IDLE")))
    results.append(check("28 clear_estop 后仍不授权", lambda: auth_is(False)))

    probe.request("estop")
    results.append(check("29 软件模式请求 estop 进入 ESTOP", lambda: state_is("ESTOP")))
    results.append(check("30 软件 estop 后不授权", lambda: auth_is(False)))
    probe.request("reset")
    results.append(check("31 软件 estop 锁存时 reset 不能清除急停", lambda: state_is("ESTOP")))
    probe.request("clear_estop")
    probe.clear_limiter_estop()
    results.append(check("32 clear_estop 后回到 IDLE", lambda: state_is("IDLE")))
    results.append(check("33 clear_estop 后输出保持零", lambda: (probe.last_output_near_zero(), "已归零")))

    for req in [
        "developer_mode",
        "developer_self_check_pass",
        "stand_ready",
        "walk_ready",
        "device_self_check_pass",
        "gimbal_follow",
        "tron_follow",
    ]:
        probe.request(req)
    results.append(check("34 死亡测试前已进入 TRON_FOLLOW 授权态", lambda: auth_is(True)))
    results.append(check("35 杀死 mode manager 进程", kill_mode_manager))
    probe.reset_outputs()
    probe.publish_cmd_for(0.03, 0.0, 0.05, 0.8)
    results.append(
        check(
            "36 mode manager 死亡后继续发命令仍因授权超时归零",
            lambda: (
                probe.last_output_near_zero(),
                f"last_output={probe.outputs[-1] if probe.outputs else None}, last_auth={probe.last_auth}",
            ),
        )
    )
    results.append(
        check(
            "37 mode manager 死亡后 limiter 状态显示授权超时",
            lambda: limiter_state_is("BLOCKED_AUTHORIZATION_TIMEOUT"),
        )
    )

    return results


def graph_has_gazebo_or_robot_hw(probe: SafeModeProbe) -> bool:
    names = set(probe.get_node_names())
    return "/gazebo" in names or "gazebo" in names or "robot_hw_node" in names


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-gazebo", action="store_true", help="额外启动官方 Gazebo/robot_hw_sim")
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    parser.add_argument("--robot-ip", default="10.192.1.2")
    parser.add_argument(
        "--allow-robot-network",
        action="store_true",
        help="仅在确认 TRON1 真机网络不会被当前 ROS graph 触达时使用。",
    )
    args = parser.parse_args()

    env = os.environ.copy()
    env.setdefault("ROBOT_TYPE", "WF_TRON1A")
    env.setdefault("RL_TYPE", "isaacgym")
    env.setdefault("FCR_TRON_CMD_VEL_TOPIC", "/fcr_tron/cmd_vel")
    env.setdefault("FCR_TRON_CMD_VEL_TIMEOUT_SEC", "0.25")

    guard_ok, guard_detail = real_robot_guard(args.allow_robot_network, args.robot_ip)
    if not guard_ok:
        print("[FAIL] 真机进程/网络守卫拒绝启动自动验收")
        print(guard_detail)
        return 4
    print(f"[PASS] 真机进程/网络守卫：{guard_detail}")

    safe_stack_cmd = [
        "ros2",
        "launch",
        "bringup_pkg",
        "fcr_tron_safe_mode_sim.launch.py",
        "enable_motion:=true",
        "allow_tron_follow_motion:=true",
        "max_linear_x:=0.03",
        "max_angular_z:=0.10",
        "input_timeout_sec:=0.25",
        "estop_timeout_sec:=0.50",
        "motion_authorized_timeout_sec:=0.50",
    ]
    safe_proc = start_process(safe_stack_cmd, env)
    gazebo_proc = None
    if args.with_gazebo:
        gazebo_proc = start_process(
            [
                "ros2",
                "launch",
                "robot_hw",
                "pointfoot_hw_sim.launch.py",
                "use_gazebo:=true",
                "fcr_cmd_vel_topic:=/fcr_tron/cmd_vel",
                "start_steering_gui:=false",
            ],
            env,
        )

    try:
        rclpy.init()
        probe = SafeModeProbe()
        if not wait_until(
            probe,
            lambda: probe.last_state is not None and probe.last_auth is not None,
            args.startup_timeout,
        ):
            print("[FAIL] 等待 mode_manager / limiter 启动超时")
            return 2

        if args.with_gazebo and not wait_until(
            probe, lambda: graph_has_gazebo_or_robot_hw(probe), args.startup_timeout
        ):
            print("[FAIL] --with-gazebo 已请求，但 ROS graph 中未看到 /gazebo 或 robot_hw_node")
            return 3

        results = run_cases(probe, require_robot_hw_subscriber=args.with_gazebo)
        passed = sum(1 for item in results if item.passed)
        for item in results:
            mark = "PASS" if item.passed else "FAIL"
            print(f"[{mark}] {item.name}: {item.detail}")

        print(f"\n结果：{passed}/{len(results)} 组通过")
        if args.with_gazebo:
            print("Gazebo/robot_hw_sim 已随测试启动；姿态漂移不作为本脚本判定项。")
        else:
            print("本次为 Gazebo 前置 topic 安全仿真；未启动真实 Gazebo 物理世界。")
        return 0 if passed == len(results) and len(results) >= 20 else 1
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        stop_process(gazebo_proc)
        stop_process(safe_proc)


if __name__ == "__main__":
    sys.exit(main())
