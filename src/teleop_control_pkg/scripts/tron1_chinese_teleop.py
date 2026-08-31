#!/usr/bin/env python3
"""Chinese line-based teleop for TRON1 simulation through the FCR safety bridge.

This tool intentionally publishes TwistStamped commands to /fcr/cmd_vel_stamped.
It never publishes directly to TRON1 /cmd_vel; tron1_safety_limiter must remain
between this console and the robot controller.
"""

from __future__ import annotations

import re
import sys
import threading
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass
class Command:
    x: float = 0.0
    yaw: float = 0.0


class Tron1ChineseTeleop(Node):
    def __init__(self) -> None:
        super().__init__("tron1_chinese_teleop")
        self.declare_parameter("cmd_topic", "/fcr/cmd_vel_stamped")
        self.declare_parameter("estop_topic", "/safety/estop_state")
        self.declare_parameter("publish_rate_hz", 10.0)
        self.declare_parameter("default_linear_x", 0.03)
        self.declare_parameter("default_angular_z", 0.10)
        self.declare_parameter("max_linear_x", 0.05)
        self.declare_parameter("max_angular_z", 0.15)
        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("require_cmd_subscriber", True)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.estop_topic = str(self.get_parameter("estop_topic").value)
        self.rate_hz = max(float(self.get_parameter("publish_rate_hz").value), 1.0)
        self.default_x = abs(float(self.get_parameter("default_linear_x").value))
        self.default_yaw = abs(float(self.get_parameter("default_angular_z").value))
        self.max_x = abs(float(self.get_parameter("max_linear_x").value))
        self.max_yaw = abs(float(self.get_parameter("max_angular_z").value))
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.require_cmd_subscriber = bool(
            self.get_parameter("require_cmd_subscriber").value
        )

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.cmd_pub = self.create_publisher(TwistStamped, self.cmd_topic, qos)
        self.estop_pub = self.create_publisher(Bool, self.estop_topic, qos)

        self._lock = threading.Lock()
        self._current = Command()
        self._running = True
        self._input_thread = threading.Thread(target=self._input_loop, daemon=True)
        self._input_thread.start()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._publish_current)

        self.get_logger().warn(
            f"中文控制台已启动：发布 {self.cmd_topic}，急停 {self.estop_topic}；"
            f"最大 x={self.max_x:.3f} m/s, yaw={self.max_yaw:.3f} rad/s"
        )
        self._print_help()

    def destroy_node(self) -> bool:
        self._running = False
        self._set_command(Command())
        for _ in range(5):
            self._publish_current()
        return super().destroy_node()

    def _input_loop(self) -> None:
        while self._running and rclpy.ok():
            try:
                line = input("中文控制> ").strip()
            except EOFError:
                self._running = False
                rclpy.shutdown()
                return
            except KeyboardInterrupt:
                self._running = False
                rclpy.shutdown()
                return

            if not line:
                continue
            self._handle_line(line)

    def _handle_line(self, line: str) -> None:
        text = line.replace("，", " ").replace(",", " ").strip().lower()

        if text in {"帮助", "help", "h", "？", "?"}:
            self._print_help()
            return

        if text in {"退出", "quit", "exit", "q"}:
            self._set_command(Command())
            self._running = False
            rclpy.shutdown()
            return

        if any(word in text for word in ("急停", "急刹", "刹车", "危险")):
            self._set_command(Command())
            self.estop_pub.publish(Bool(data=True))
            print("已发送急停：/safety/estop_state=true，输出应立即归零。")
            return

        if any(word in text for word in ("解除急停", "恢复急停", "清除急停")):
            self.estop_pub.publish(Bool(data=False))
            print("已尝试解除限速器急停状态：/safety/estop_state=false。")
            return

        if text in {"停", "停止", "停车", "别动", "stop", "s"}:
            self._set_command(Command())
            print("已停车：持续发布 0 速度。")
            return

        cmd = self._parse_motion(text)
        if cmd is None:
            print("没听懂这句。输入“帮助”可以看示例。为了安全，当前命令不变。")
            return
        if self.require_cmd_subscriber and self.cmd_pub.get_subscription_count() == 0:
            self._set_command(Command())
            print(
                f"没有检测到任何节点订阅 {self.cmd_topic}，运动命令已拒绝。\n"
                "请先启动 tron1_safety_limiter；如果 Gazebo/限速器还没开，"
                "这里输入“直走/后退/转弯”不会生效。"
            )
            return

        self._set_command(cmd)
        print(f"当前命令：前后 x={cmd.x:.3f} m/s，转向 yaw={cmd.yaw:.3f} rad/s")

    def _parse_motion(self, text: str) -> Optional[Command]:
        x = 0.0
        yaw = 0.0
        has_motion_word = False

        if any(word in text for word in ("直走", "前进", "向前", "往前", "走")):
            x = self.default_x
            has_motion_word = True
        if any(word in text for word in ("后退", "倒车", "向后", "往后")):
            x = -self.default_x
            has_motion_word = True
        if any(word in text for word in ("左转", "向左转", "往左转")):
            yaw = self.default_yaw
            has_motion_word = True
        if any(word in text for word in ("右转", "向右转", "往右转")):
            yaw = -self.default_yaw
            has_motion_word = True

        speed = self._extract_number_after(text, ("速度", "线速度", "前进速度", "x"))
        if speed is not None:
            if x < 0.0 or any(word in text for word in ("后退", "倒车", "向后", "往后")):
                x = -abs(speed)
            else:
                x = abs(speed)
            has_motion_word = True

        yaw_speed = self._extract_number_after(text, ("转速", "角速度", "yaw", "z"))
        if yaw_speed is not None:
            if yaw < 0.0 or any(word in text for word in ("右转", "向右转", "往右转")):
                yaw = -abs(yaw_speed)
            else:
                yaw = abs(yaw_speed)
            has_motion_word = True

        if "快" in text:
            x *= 1.5
            yaw *= 1.5
        if "慢" in text:
            x *= 0.5
            yaw *= 0.5

        if not has_motion_word:
            return None

        return Command(
            x=clamp(x, -self.max_x, self.max_x),
            yaw=clamp(yaw, -self.max_yaw, self.max_yaw),
        )

    @staticmethod
    def _extract_number_after(text: str, keys: tuple[str, ...]) -> Optional[float]:
        for key in keys:
            match = re.search(rf"{re.escape(key)}\s*[:=：]?\s*(-?\d+(?:\.\d+)?)", text)
            if match:
                return float(match.group(1))
        return None

    def _set_command(self, cmd: Command) -> None:
        with self._lock:
            self._current = cmd

    def _publish_current(self) -> None:
        with self._lock:
            cmd = Command(self._current.x, self._current.yaw)
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.twist.linear.x = cmd.x
        msg.twist.linear.y = 0.0
        msg.twist.angular.z = cmd.yaw
        self.cmd_pub.publish(msg)

    def _print_help(self) -> None:
        print(
            "\n中文控制示例：\n"
            "  直走              # 默认 0.03 m/s\n"
            "  后退              # 默认 -0.03 m/s\n"
            "  左转              # 默认 0.10 rad/s\n"
            "  右转              # 默认 -0.10 rad/s\n"
            "  直走 左转          # 小弧线\n"
            "  速度 0.04         # 设置前进速度，仍会被限幅\n"
            "  左转 转速 0.12     # 设置转向角速度，仍会被限幅\n"
            "  慢慢直走 / 快点左转\n"
            "  停 / 停止 / 停车\n"
            "  急停\n"
            "  解除急停\n"
            "  帮助\n"
            "  退出\n"
        )


def main() -> None:
    rclpy.init()
    node = Tron1ChineseTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
