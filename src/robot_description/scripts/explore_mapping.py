#!/usr/bin/env python3
"""改进的自主探索建图节点。

状态机 FORWARD / TURN / BACKUP：
- 低速直线为主，转向一律原地旋转（差速底盘原地转向最不容易打滑漂移）
- 前方到安全距离就原地转向更开阔的一侧，转到前方持续开阔再走
- 卡死（odom 不动）先后退再转向
相对随机避障漫游，轨迹更规整，SLAM 重影/错位更少。
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

SCAN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=20,
)


class Explorer(Node):
    def __init__(self):
        super().__init__('explorer')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan_clean', self.scan_cb, SCAN_QOS)
        self.create_subscription(Odometry, '/odom_clean', self.odom_cb, 20)

        self.lin = 0.14
        self.turn = 0.45
        self.safe = 0.45
        self.too_close = 0.2

        self.ranges = []
        self.state = 'FORWARD'
        self.state_t0 = time.time()
        self.turn_dir = 1.0
        self.open_since = None
        self.last_pose = None
        self.stall_t0 = None

        self.create_timer(0.15, self.step)
        self.get_logger().info('explorer 已启动')

    def odom_cb(self, m):
        x = m.pose.pose.position.x
        y = m.pose.pose.position.y
        now = time.time()
        if self.last_pose is None:
            self.last_pose = (x, y, now)
            return
        moved = math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        if now - self.last_pose[2] >= 2.0:
            if moved < 0.03 and self.state == 'FORWARD':
                if self.stall_t0 is None:
                    self.stall_t0 = now
                elif now - self.stall_t0 > 1.5:
                    self.set_state('BACKUP')
            else:
                self.stall_t0 = None
            self.last_pose = (x, y, now)

    def scan_cb(self, m):
        self.ranges = list(m.ranges)

    def sector_min(self, lo_deg, hi_deg):
        if not self.ranges:
            return 999.0
        n = len(self.ranges)
        a = int((lo_deg / 360.0 + 0.5) * n) % n
        b = int((hi_deg / 360.0 + 0.5) * n) % n
        if a < b:
            vals = self.ranges[a:b]
        else:
            vals = self.ranges[a:] + self.ranges[:b]
        vals = [v for v in vals if v == v and v > 0.05]
        return min(vals) if vals else 999.0

    def set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
        self.state = s
        self.state_t0 = time.time()
        self.open_since = None

    def step(self):
        if not self.ranges:
            return
        front = self.sector_min(-20, 20)
        left = self.sector_min(25, 110)
        right = self.sector_min(-110, -25)
        rear = self.sector_min(150, 210)

        cmd = Twist()
        now = time.time()

        if self.state == 'FORWARD':
            if front < self.safe:
                self.turn_dir = 1.0 if left >= right else -1.0
                self.set_state('TURN')
            else:
                cmd.linear.x = self.lin

        elif self.state == 'TURN':
            cmd.angular.z = self.turn * self.turn_dir
            if front > self.safe + 0.1:
                if self.open_since is None:
                    self.open_since = now
                elif now - self.open_since > 0.45:
                    self.set_state('FORWARD')
            else:
                self.open_since = None
            if now - self.state_t0 > 6:
                self.turn_dir *= -1.0
                self.set_state('TURN')

        elif self.state == 'BACKUP':
            cmd.linear.x = -0.10
            cmd.angular.z = self.turn * (-self.turn_dir)
            if (rear > 0.4 and now - self.state_t0 > 1.2) or \
               now - self.state_t0 > 3:
                self.turn_dir = 1.0 if left >= right else -1.0
                self.set_state('TURN')

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Explorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
