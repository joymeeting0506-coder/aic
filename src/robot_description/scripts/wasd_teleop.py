#!/usr/bin/env python3
"""WASD 键盘遥控：W 前进 / S 后退 / A 左转 / D 右转，Q 退出，空格停。"""
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class WasdTeleop(Node):
    def __init__(self):
        super().__init__('wasd_teleop')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.speed = 0.15
        self.turn = 0.6
        self.get_logger().info('WASD 遥控：W前进 S后退 A左转 D右转 Q退出，速度 0.15')

    def run(self):
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        try:
            while rclpy.ok():
                ch = sys.stdin.read(1)
                cmd = Twist()
                if ch == 'w':
                    cmd.linear.x = self.speed
                elif ch == 's':
                    cmd.linear.x = -self.speed
                elif ch == 'a':
                    cmd.angular.z = self.turn
                elif ch == 'd':
                    cmd.angular.z = -self.turn
                elif ch in ('q', '\x03'):
                    break
                self.pub.publish(cmd)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)
            self.pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = WasdTeleop()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
