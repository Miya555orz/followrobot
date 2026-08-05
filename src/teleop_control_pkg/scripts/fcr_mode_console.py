#!/usr/bin/env python3
"""Interactive owner console for follow, teleop, cinematic and manual jog."""

from __future__ import annotations

import math
import select
import sys
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger
from vision_servo_msgs.action import CinematicMove, ManualJog
from vision_servo_msgs.msg import GimbalCmd, GimbalNudge


@dataclass
class Motion:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0


class Keyboard:
    def __init__(self) -> None:
        if not sys.stdin.isatty():
            raise RuntimeError("集成控制台必须运行在交互式终端中")
        import termios
        import tty

        self._termios = termios
        self._old = termios.tcgetattr(sys.stdin)
        self._buffer = ""
        self._escape_started: Optional[float] = None
        tty.setcbreak(sys.stdin.fileno())

    def read(self) -> Optional[str]:
        while select.select([sys.stdin], [], [], 0.0)[0]:
            self._buffer += sys.stdin.read(1)
        if not self._buffer:
            return None
        if not self._buffer.startswith("\x1b"):
            key, self._buffer = self._buffer[0], self._buffer[1:]
            return key
        if self._escape_started is None:
            self._escape_started = time.monotonic()
        if len(self._buffer) < 3:
            if time.monotonic() - self._escape_started > 0.15:
                self._buffer = self._buffer[1:]
                self._escape_started = None
            return None
        sequence, self._buffer = self._buffer[:3], self._buffer[3:]
        self._escape_started = None
        return {
            "\x1b[A": "UP", "\x1b[B": "DOWN",
            "\x1b[C": "RIGHT", "\x1b[D": "LEFT",
        }.get(sequence)

    def close(self) -> None:
        self._termios.tcsetattr(sys.stdin, self._termios.TCSADRAIN, self._old)


class FcrModeConsole(Node):
    STANDBY = "STANDBY"
    FOLLOW = "FOLLOW"
    CINEMATIC = "CINEMATIC_READY"
    TELEOP = "TELEOP"
    JOG_MENU = "MANUAL_JOG_MENU"

    def __init__(self) -> None:
        super().__init__("fcr_mode_console")
        self.declare_parameter("rate_hz", 30.0)
        self.declare_parameter("key_timeout_sec", 0.12)
        self.declare_parameter("linear_speed", 0.06)
        self.declare_parameter("strafe_speed", 0.06)
        self.declare_parameter("yaw_rate", 0.25)
        self.declare_parameter("gimbal_step_deg", 3.0)
        self.declare_parameter("jog_translation_m", 0.10)
        self.declare_parameter("jog_rotation_deg", 5.0)
        self.declare_parameter("cinematic_displacement_m", 0.50)
        self.declare_parameter("cinematic_orbit_deg", 30.0)
        self.declare_parameter("cinematic_speed", 0.08)

        self.rate_hz = max(float(self.get_parameter("rate_hz").value), 10.0)
        self.key_timeout = max(float(self.get_parameter("key_timeout_sec").value), 0.05)
        self.linear_speed = abs(float(self.get_parameter("linear_speed").value))
        self.strafe_speed = abs(float(self.get_parameter("strafe_speed").value))
        self.yaw_rate = abs(float(self.get_parameter("yaw_rate").value))
        self.gimbal_step = math.radians(abs(float(self.get_parameter("gimbal_step_deg").value)))
        self.jog_translation = abs(float(self.get_parameter("jog_translation_m").value))
        self.jog_rotation = math.radians(abs(float(self.get_parameter("jog_rotation_deg").value)))
        self.cinematic_displacement = abs(
            float(self.get_parameter("cinematic_displacement_m").value)
        )
        self.cinematic_orbit = abs(float(self.get_parameter("cinematic_orbit_deg").value))
        self.cinematic_speed = abs(float(self.get_parameter("cinematic_speed").value))

        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.mode_pub = self.create_publisher(String, "/teleop/mode", qos)
        self.heartbeat_pub = self.create_publisher(Empty, "/teleop/heartbeat", qos)
        self.deadman_pub = self.create_publisher(Bool, "/teleop/deadman", qos)
        self.velocity_pub = self.create_publisher(TwistStamped, "/teleop/cmd_vel", qos)
        self.gimbal_pub = self.create_publisher(GimbalCmd, "/teleop/cmd_gimbal", qos)
        self.gimbal_nudge_pub = self.create_publisher(
            GimbalNudge, "/teleop/gimbal_nudge", qos
        )
        self.estop_pub = self.create_publisher(Bool, "/teleop/estop", qos)
        self.clear_estop_pub = self.create_publisher(Empty, "/teleop/clear_estop", qos)
        self.console_status_pub = self.create_publisher(
            String, "/operator_console/status", latched
        )

        self.cinematic_enter = self.create_client(Trigger, "/cinematic/enter")
        self.cinematic_exit = self.create_client(Trigger, "/cinematic/exit")
        self.cinematic_stop = self.create_client(Trigger, "/cinematic/stop")
        self.jog_stop = self.create_client(Trigger, "/manual_jog/stop")
        self.cinematic_action = ActionClient(self, CinematicMove, "/cinematic/execute")
        self.jog_action = ActionClient(self, ManualJog, "/manual_jog/execute")

        self.state = self.STANDBY
        self.motion = Motion()
        self.motion_time = time.monotonic()
        self.deadman = False
        self.speed_scale = 1.0
        self.gimbal_nudge_id = 0
        self.active_cinematic = None
        self.active_jog = None
        self.pending_transition = None
        self.transition_at = 0.0
        self.shutdown_requested = False
        self.keyboard = Keyboard()
        self.timer = self.create_timer(1.0 / self.rate_hz, self._tick)
        self._enter_standby("启动完成")
        self._print_main_help()

    def close(self) -> None:
        self._safe_stop(cancel_tasks=True)
        self.keyboard.close()

    def _tick(self) -> None:
        if self.pending_transition is not None and time.monotonic() >= self.transition_at:
            transition = self.pending_transition
            self.pending_transition = None
            transition()
        key = self.keyboard.read()
        if key is not None:
            self._handle_key(key)
        if self.state == self.TELEOP:
            if time.monotonic() - self.motion_time > self.key_timeout:
                self.motion = Motion()
                self.deadman = False
            self.heartbeat_pub.publish(Empty())
            self._publish_teleop()
        if self.shutdown_requested:
            rclpy.shutdown()

    def _handle_key(self, key: str) -> None:
        if key == "\x03":
            self.shutdown_requested = True
            return
        if key in ("h", "?"):
            self._print_main_help()
            return
        if key == "x":
            self._safe_stop(cancel_tasks=True)
            self.estop_pub.publish(Bool(data=True))
            self._set_state(self.STANDBY, "软件急停")
            return
        if key == "c":
            self.clear_estop_pub.publish(Empty())
            self.get_logger().warning("已请求清除软件急停；仍需确认现场安全")
            return
        if key == "1":
            self._enter_standby("人工切换")
        elif key == "2":
            self._enter_follow()
        elif key == "3":
            self._enter_cinematic()
        elif key == "4":
            self._prepare_jog()
        elif key == "5":
            self._enter_teleop()
        elif self.state == self.TELEOP:
            self._handle_teleop_key(key)
        elif self.state == self.CINEMATIC:
            self._handle_cinematic_key(key)
        elif self.state == self.JOG_MENU:
            self._handle_jog_key(key)

    def _enter_standby(self, detail: str) -> None:
        self._safe_stop(cancel_tasks=True)
        self._call(self.cinematic_exit, "退出运镜")
        self._set_state(self.STANDBY, detail)

    def _enter_follow(self) -> None:
        self._safe_stop(cancel_tasks=True)
        self._call(self.cinematic_exit, "退出运镜")
        self._schedule_transition(self._finish_enter_follow, "FOLLOW准备中")

    def _finish_enter_follow(self) -> None:
        self._publish_mode("auto")
        self._set_state(self.FOLLOW, "自动跟随已接管")

    def _enter_cinematic(self) -> None:
        self._safe_stop(cancel_tasks=True)
        self._schedule_transition(self._finish_enter_cinematic, "CINEMATIC准备中")

    def _finish_enter_cinematic(self) -> None:
        if not self.cinematic_enter.service_is_ready():
            self.get_logger().error("运镜进入服务未就绪")
            self._set_state(self.STANDBY, "运镜不可用")
            return
        future = self.cinematic_enter.call_async(Trigger.Request())
        future.add_done_callback(self._cinematic_entered)

    def _cinematic_entered(self, future) -> None:
        response = future.result()
        if not response.success:
            self.get_logger().error(f"进入运镜失败: {response.message}")
            self._set_state(self.STANDBY, response.message)
            return
        self._set_state(self.CINEMATIC, "等待运镜动作")
        self.get_logger().info("运镜键: Z静态，W/S推拉，A/D横移，Q/E环绕")

    def _prepare_jog(self) -> None:
        self._safe_stop(cancel_tasks=True)
        self._call(self.cinematic_exit, "退出运镜")
        self._schedule_transition(self._finish_prepare_jog, "MANUAL_JOG准备中")

    def _finish_prepare_jog(self) -> None:
        self._set_state(self.JOG_MENU, "等待一次微调")
        self.get_logger().info(
            "微调键: W/S前后 A/D横移 Q/E旋转；方向键控制云台；执行完自动待机"
        )

    def _enter_teleop(self) -> None:
        self._safe_stop(cancel_tasks=True)
        self._call(self.cinematic_exit, "退出运镜")
        self._schedule_transition(self._finish_enter_teleop, "TELEOP准备中")

    def _finish_enter_teleop(self) -> None:
        self._publish_mode("manual")
        self._set_state(self.TELEOP, "连续遥控")
        self.get_logger().info("遥控键: W/S A/D Q/E，方向键云台；空格停车，+/-调速")

    def _handle_teleop_key(self, key: str) -> None:
        scale = self.speed_scale
        motion = None
        if key == "w": motion = Motion(x=self.linear_speed * scale)
        elif key == "s": motion = Motion(x=-self.linear_speed * scale)
        elif key == "a": motion = Motion(y=self.strafe_speed * scale)
        elif key == "d": motion = Motion(y=-self.strafe_speed * scale)
        elif key == "q": motion = Motion(yaw=self.yaw_rate * scale)
        elif key == "e": motion = Motion(yaw=-self.yaw_rate * scale)
        elif key == "UP": self._gimbal_nudge(0.0, self.gimbal_step * scale)
        elif key == "DOWN": self._gimbal_nudge(0.0, -self.gimbal_step * scale)
        elif key == "LEFT": self._gimbal_nudge(self.gimbal_step * scale, 0.0)
        elif key == "RIGHT": self._gimbal_nudge(-self.gimbal_step * scale, 0.0)
        elif key == " ": self._zero_manual()
        elif key in ("+", "="):
            self.speed_scale = min(1.5, self.speed_scale + 0.25)
            self.get_logger().info(f"遥控速度倍率 {self.speed_scale:.2f}")
        elif key in ("-", "_"):
            self.speed_scale = max(0.25, self.speed_scale - 0.25)
            self.get_logger().info(f"遥控速度倍率 {self.speed_scale:.2f}")
        if motion is not None:
            self.motion = motion
            self.deadman = True
            self.motion_time = time.monotonic()

    def _handle_jog_key(self, key: str) -> None:
        axis = displacement = speed = None
        if key == "w": axis, displacement, speed = ManualJog.Goal.CHASSIS_X, self.jog_translation, 0.06
        elif key == "s": axis, displacement, speed = ManualJog.Goal.CHASSIS_X, -self.jog_translation, 0.06
        elif key == "a": axis, displacement, speed = ManualJog.Goal.CHASSIS_Y, self.jog_translation, 0.06
        elif key == "d": axis, displacement, speed = ManualJog.Goal.CHASSIS_Y, -self.jog_translation, 0.06
        elif key == "q": axis, displacement, speed = ManualJog.Goal.CHASSIS_YAW, self.jog_rotation, 0.20
        elif key == "e": axis, displacement, speed = ManualJog.Goal.CHASSIS_YAW, -self.jog_rotation, 0.20
        elif key == "UP": axis, displacement, speed = ManualJog.Goal.GIMBAL_PITCH, self.jog_rotation, 0.20
        elif key == "DOWN": axis, displacement, speed = ManualJog.Goal.GIMBAL_PITCH, -self.jog_rotation, 0.20
        elif key == "LEFT": axis, displacement, speed = ManualJog.Goal.GIMBAL_YAW, self.jog_rotation, 0.20
        elif key == "RIGHT": axis, displacement, speed = ManualJog.Goal.GIMBAL_YAW, -self.jog_rotation, 0.20
        if axis is None:
            return
        goal = ManualJog.Goal()
        goal.axis = axis
        goal.displacement = float(displacement)
        goal.max_speed = float(speed)
        goal.timeout_sec = 6.0
        self._send_jog(goal)

    def _handle_cinematic_key(self, key: str) -> None:
        mode = None
        direction = CinematicMove.Goal.DIRECTION_AUTO
        if key in ("z", "Z"): mode = CinematicMove.Goal.STATIC_TRACK
        elif key == "w": mode = CinematicMove.Goal.DOLLY_IN_OUT; direction = 1
        elif key == "s": mode = CinematicMove.Goal.DOLLY_IN_OUT; direction = -1
        elif key == "a": mode = CinematicMove.Goal.TRUCK_LEFT_RIGHT; direction = 1
        elif key == "d": mode = CinematicMove.Goal.TRUCK_LEFT_RIGHT; direction = -1
        elif key == "q": mode = CinematicMove.Goal.ORBIT_ARC; direction = 1
        elif key == "e": mode = CinematicMove.Goal.ORBIT_ARC; direction = -1
        if mode is None:
            return
        goal = CinematicMove.Goal()
        goal.mode = mode
        goal.tracking_id = -1
        goal.target_distance_m = 2.0
        goal.displacement_m = self.cinematic_displacement
        goal.orbit_angle_deg = self.cinematic_orbit
        goal.orbit_radius_m = 2.0
        goal.max_speed = self.cinematic_speed
        goal.duration_sec = 15.0
        goal.direction = direction
        self._send_cinematic(goal)

    def _send_jog(self, goal: ManualJog.Goal) -> None:
        if not self.jog_action.server_is_ready():
            self.get_logger().error("MANUAL_JOG Action未就绪")
            self._set_state(self.STANDBY, "微调不可用")
            return
        future = self.jog_action.send_goal_async(goal)
        future.add_done_callback(self._jog_goal_response)
        self._set_state("MANUAL_JOG", "动作已发送")

    def _jog_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("MANUAL_JOG动作被拒绝")
            self._set_state(self.STANDBY, "微调被拒绝")
            return
        self.active_jog = handle
        result = handle.get_result_async()
        result.add_done_callback(self._jog_done)

    def _jog_done(self, future) -> None:
        result = future.result().result
        self.active_jog = None
        if self.state == "MANUAL_JOG":
            self._set_state(self.STANDBY, result.message)

    def _send_cinematic(self, goal: CinematicMove.Goal) -> None:
        if self.active_cinematic is not None:
            self.get_logger().warning("已有运镜动作，先停止或等待完成")
            return
        if not self.cinematic_action.server_is_ready():
            self.get_logger().error("运镜Action未就绪")
            return
        future = self.cinematic_action.send_goal_async(goal)
        future.add_done_callback(self._cinematic_goal_response)

    def _cinematic_goal_response(self, future) -> None:
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("运镜动作被拒绝；确认已进入CINEMATIC_READY")
            return
        self.active_cinematic = handle
        handle.get_result_async().add_done_callback(self._cinematic_done)
        self._set_state("CINEMATIC_EXECUTING", "动作执行中")

    def _cinematic_done(self, future) -> None:
        result = future.result().result
        self.active_cinematic = None
        if self.state == "CINEMATIC_EXECUTING":
            self._set_state(self.CINEMATIC, result.message)

    def _safe_stop(self, cancel_tasks: bool) -> None:
        self.pending_transition = None
        self._zero_manual()
        self._publish_mode("stop")
        if cancel_tasks:
            if self.active_jog is not None:
                self.active_jog.cancel_goal_async()
            if self.active_cinematic is not None:
                self.active_cinematic.cancel_goal_async()
            self._call(self.jog_stop, "停止微调")
            self._call(self.cinematic_stop, "停止运镜")

    def _schedule_transition(self, callback, detail: str) -> None:
        # Both MANUAL_JOG and cinematic controllers have a bounded settling
        # phase. Delay ownership grant so their final STOP cannot overwrite the
        # newly selected mode.
        self.pending_transition = callback
        self.transition_at = time.monotonic() + 0.45
        self._set_state("TRANSITION", detail)

    def _zero_manual(self) -> None:
        self.motion = Motion()
        self.deadman = False
        for _ in range(2):
            self.heartbeat_pub.publish(Empty())
            self._publish_teleop()

    def _publish_teleop(self) -> None:
        stamp = self.get_clock().now().to_msg()
        velocity = TwistStamped()
        velocity.header.stamp = stamp
        velocity.header.frame_id = "base_link"
        velocity.twist.linear.x = self.motion.x
        velocity.twist.linear.y = self.motion.y
        velocity.twist.angular.z = self.motion.yaw
        self.velocity_pub.publish(velocity)
        gimbal = GimbalCmd()
        gimbal.header.stamp = stamp
        gimbal.hold_yaw = True
        gimbal.hold_pitch = True
        self.gimbal_pub.publish(gimbal)
        self.deadman_pub.publish(Bool(data=self.deadman))

    def _gimbal_nudge(self, yaw: float, pitch: float) -> None:
        self.gimbal_nudge_id += 1
        command = GimbalNudge()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = "gimbal_link"
        command.command_id = self.gimbal_nudge_id
        command.yaw_delta = float(yaw)
        command.pitch_delta = float(pitch)
        command.duration = 0.15
        self.gimbal_nudge_pub.publish(command)
        self.deadman = True
        self.motion_time = time.monotonic()

    def _publish_mode(self, mode: str) -> None:
        self.mode_pub.publish(String(data=mode))

    @staticmethod
    def _call(client, label: str) -> None:
        if client.service_is_ready():
            client.call_async(Trigger.Request())
        else:
            # Optional subsystems may be intentionally disabled.
            del label

    def _set_state(self, state: str, detail: str) -> None:
        self.state = state
        self.console_status_pub.publish(String(data=f'{state}: {detail}'))
        self.get_logger().warning(f"控制台状态 -> {state}: {detail}")

    def _print_main_help(self) -> None:
        print(
            "\n========== FCR 实机控制台 ==========\n"
            "  1 待机/停止     2 自动跟随\n"
            "  3 运镜模式      4 单次微调\n"
            "  5 连续遥控      H 帮助\n"
            "  X 软件急停      C 清除急停\n"
            "  Ctrl-C 关闭整套实机系统\n"
            "====================================\n",
            flush=True,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FcrModeConsole()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
