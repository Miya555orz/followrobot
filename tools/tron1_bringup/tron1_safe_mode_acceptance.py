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
        self.cmd_pub = self.create_publisher(TwistStamped, "/fcr/cmd_vel_stamped", 10)

        self.last_state: str | None = None
        self.last_auth: bool | None = None
        self.outputs: list[tuple[float, float, float]] = []

        self.create_subscription(String, "/tron1/mode_state", self._on_state, TRANSIENT_QOS)
        self.create_subscription(Bool, "/tron1/motion_authorized", self._on_auth, TRANSIENT_QOS)
        self.create_subscription(TwistStamped, "/unused", lambda _: None, 10)
        self.create_subscription(Twist, "/fcr_tron/cmd_vel", self._on_output, 10)

    def _on_state(self, msg: String) -> None:
        self.last_state = msg.data

    def _on_auth(self, msg: Bool) -> None:
        self.last_auth = bool(msg.data)

    def _on_output(self, msg) -> None:
        self.outputs.append((msg.linear.x, msg.linear.y, msg.angular.z))
        self.outputs = self.outputs[-200:]

    def spin_for(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def request(self, text: str) -> None:
        msg = String()
        msg.data = text
        for _ in range(3):
            self.mode_request_pub.publish(msg)
            self.spin_for(0.05)

    def estop(self, active: bool) -> None:
        msg = Bool()
        msg.data = active
        for _ in range(3):
            self.estop_pub.publish(msg)
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


def run_cases(probe: SafeModeProbe) -> list[CaseResult]:
    results: list[CaseResult] = []
    tol = 1e-3

    def state_is(expected: str) -> tuple[bool, str]:
        ok = wait_until(probe, lambda: probe.last_state == expected, 1.0)
        return ok, f"state={probe.last_state}, expected={expected}"

    def auth_is(expected: bool) -> tuple[bool, str]:
        ok = wait_until(probe, lambda: probe.last_auth is expected, 1.0)
        return ok, f"motion_authorized={probe.last_auth}, expected={expected}"

    probe.estop(False)
    probe.request("reset")
    results.append(check("01 初始/复位后进入 IDLE", lambda: state_is("IDLE")))
    results.append(check("02 IDLE 不授权运动", lambda: auth_is(False)))

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

    probe.publish_cmd_for(0.3, 0.0, 0.0, 0.3)
    probe.estop(True)
    results.append(check("21 外部急停强制进入 ESTOP", lambda: state_is("ESTOP")))
    results.append(check("22 外部急停后取消运动授权", lambda: auth_is(False)))
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
    probe.request("clear_estop")
    results.append(check("27 clear_estop 后回到 IDLE", lambda: state_is("IDLE")))
    results.append(check("28 clear_estop 后仍不授权", lambda: auth_is(False)))

    probe.request("estop")
    results.append(check("29 软件模式请求 estop 进入 ESTOP", lambda: state_is("ESTOP")))
    results.append(check("30 软件 estop 后不授权", lambda: auth_is(False)))
    probe.request("reset")
    results.append(check("31 reset 可回到 IDLE", lambda: state_is("IDLE")))
    results.append(check("32 reset 后输出保持零", lambda: (probe.last_output_near_zero(), "已归零")))

    return results


def graph_has_gazebo_or_robot_hw(probe: SafeModeProbe) -> bool:
    names = set(probe.get_node_names())
    return "/gazebo" in names or "gazebo" in names or "robot_hw_node" in names


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--with-gazebo", action="store_true", help="额外启动官方 Gazebo/robot_hw_sim")
    parser.add_argument("--startup-timeout", type=float, default=8.0)
    args = parser.parse_args()

    env = os.environ.copy()
    env.setdefault("ROBOT_TYPE", "WF_TRON1A")
    env.setdefault("RL_TYPE", "isaacgym")
    env.setdefault("FCR_TRON_CMD_VEL_TOPIC", "/fcr_tron/cmd_vel")
    env.setdefault("FCR_TRON_CMD_VEL_TIMEOUT_SEC", "0.25")

    safe_stack_cmd = [
        "ros2",
        "launch",
        "bringup_pkg",
        "fcr_tron_safe_mode_sim.launch.py",
        "enable_motion:=true",
        "max_linear_x:=0.03",
        "max_angular_z:=0.10",
        "input_timeout_sec:=0.25",
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

        results = run_cases(probe)
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
