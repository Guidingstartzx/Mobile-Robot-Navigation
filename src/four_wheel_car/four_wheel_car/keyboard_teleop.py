#!/usr/bin/python3
import select
import sys
import termios
import time
import tty
from typing import TextIO

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


HELP = """
WASD incremental teleop for /cmd_vel

  w/s : increase/decrease center-of-mass linear x velocity
  a/d : increase/decrease center-of-mass yaw rate
  space : clear command to zero
  q or Ctrl-C : quit
"""


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


class KeyboardTeleop(Node):
    def __init__(self) -> None:
        super().__init__("four_wheel_keyboard_teleop")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("linear_step", 0.10)
        self.declare_parameter("angular_step", 0.20)
        self.declare_parameter("max_linear", 1.50)
        self.declare_parameter("max_angular", 2.00)
        self.declare_parameter("publish_rate", 20.0)

        topic = self.get_parameter("cmd_vel_topic").value
        self.linear_step = float(self.get_parameter("linear_step").value)
        self.angular_step = float(self.get_parameter("angular_step").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)
        self.publish_period = 1.0 / float(self.get_parameter("publish_rate").value)

        self.publisher = self.create_publisher(Twist, topic, 10)
        self.cmd = Twist()

    def apply_key(self, key: str) -> bool:
        if key in ("q", "Q", "\x03"):
            self.zero()
            return False
        if key in ("w", "W"):
            self.cmd.linear.x = clamp(
                self.cmd.linear.x + self.linear_step,
                -self.max_linear,
                self.max_linear,
            )
        elif key in ("s", "S"):
            self.cmd.linear.x = clamp(
                self.cmd.linear.x - self.linear_step,
                -self.max_linear,
                self.max_linear,
            )
        elif key in ("a", "A"):
            self.cmd.angular.z = clamp(
                self.cmd.angular.z + self.angular_step,
                -self.max_angular,
                self.max_angular,
            )
        elif key in ("d", "D"):
            self.cmd.angular.z = clamp(
                self.cmd.angular.z - self.angular_step,
                -self.max_angular,
                self.max_angular,
            )
        elif key == " ":
            self.zero()
        else:
            return True

        self.publish()
        self.print_status()
        return True

    def zero(self) -> None:
        self.cmd = Twist()
        self.publish()

    def publish(self) -> None:
        self.publisher.publish(self.cmd)

    def print_status(self) -> None:
        sys.stdout.write(
            f"\rlinear.x={self.cmd.linear.x:+.2f} m/s  "
            f"angular.z={self.cmd.angular.z:+.2f} rad/s      "
        )
        sys.stdout.flush()


def open_keyboard_stream() -> TextIO | None:
    if sys.stdin.isatty():
        return sys.stdin

    try:
        return open("/dev/tty", "r", encoding="utf-8", buffering=1)
    except OSError:
        return None


def read_key(input_stream: TextIO, timeout: float) -> str:
    readable, _, _ = select.select([input_stream], [], [], timeout)
    if readable:
        return input_stream.read(1)
    return ""


def main() -> None:
    rclpy.init()
    node = KeyboardTeleop()

    input_stream = open_keyboard_stream()
    if input_stream is None:
        node.get_logger().error(
            "keyboard_teleop needs a terminal. Run it from a real terminal with "
            "'ros2 run four_wheel_car keyboard_teleop'."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    old_settings = termios.tcgetattr(input_stream.fileno())
    try:
        tty.setcbreak(input_stream.fileno())
        print(HELP)
        node.print_status()
        running = True
        next_publish = time.monotonic()
        while rclpy.ok() and running:
            key = read_key(input_stream, 0.02)
            if key:
                running = node.apply_key(key)

            now = time.monotonic()
            if now >= next_publish:
                node.publish()
                next_publish = now + node.publish_period

            rclpy.spin_once(node, timeout_sec=0.0)
    finally:
        node.zero()
        termios.tcsetattr(input_stream.fileno(), termios.TCSADRAIN, old_settings)
        if input_stream is not sys.stdin:
            input_stream.close()
        print()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
