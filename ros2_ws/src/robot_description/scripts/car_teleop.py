#!/usr/bin/env python3
"""Drive the mobile winch car from an interactive terminal."""

import math
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions


class CarTeleop(Node):
    def __init__(self):
        super().__init__("car_teleop")
        self.speed = self.declare_parameter("speed", 0.5).value
        self.turn_rate = self.declare_parameter("turn_rate", 0.2).value
        self.key_timeout = self.declare_parameter("key_timeout", 0.6).value
        for name, value in (("speed", self.speed), ("turn_rate", self.turn_rate),
                            ("key_timeout", self.key_timeout)):
            if not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.publisher = self.create_publisher(Twist, "/car/cmd_vel", 1)
        self.command = Twist()
        self.last_key = time.monotonic()

    def handle_key(self, key):
        # Turns include forward/reverse motion because an Ackermann car cannot spin in place.
        directions = {"w": (1, 0), "s": (-1, 0), "a": (1, 1), "d": (1, -1),
                      "z": (-1, -1), "c": (-1, 1)}
        linear, angular = directions.get(key, (0, 0))
        self.command.linear.x = linear * self.speed
        self.command.angular.z = angular * self.turn_rate
        self.last_key = time.monotonic()

    def publish_command(self):
        if time.monotonic() - self.last_key > self.key_timeout:
            self.command = Twist()
        self.publisher.publish(self.command)


def main():
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    settings = None
    try:
        if not sys.stdin.isatty():
            raise RuntimeError("Run car_teleop.py in an interactive terminal")
        node = CarTeleop()
        settings = termios.tcgetattr(sys.stdin)
        print("W/S forward/reverse | A/D forward turns | Z/C reverse turns\r")
        print("Space stops | Q or Ctrl+C exits | Release keys to stop after key_timeout\r")
        tty.setcbreak(sys.stdin.fileno())
        while rclpy.ok():
            readable, _, _ = select.select([sys.stdin], [], [], 0.05)
            if readable:
                key = sys.stdin.read(1).lower()
                if key in ("q", "\x03"):
                    break
                node.handle_key(key)
            rclpy.spin_once(node, timeout_sec=0.0)
            node.publish_command()
    except KeyboardInterrupt:
        pass
    finally:
        if settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        if node is not None:
            if rclpy.ok():
                node.publisher.publish(Twist())
                rclpy.spin_once(node, timeout_sec=0.05)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
