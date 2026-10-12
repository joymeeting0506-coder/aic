#!/usr/bin/env python3
"""自动建图：LiDAR 避障探索，驱动机器人走遍场地。

策略：持续前进；前方 90° 扇形内有障碍（< 0.4m）就朝更开阔的一侧转向。
跑一段时间后由用户 Ctrl+C 停止，再保存地图。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class AutoMapping(Node):
    def __init__(self):
        super().__init__('auto_mapping')
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        sens = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )
        self.create_subscription(LaserScan, '/scan_clean', self.scan_cb, sens)

        self.front = 999.0
        self.left = 999.0
        self.right = 999.0
        self.speed = 0.15
        self.turn = 0.5
        self.create_timer(0.2, self.publish_cmd)
        self.get_logger().info('auto_mapping 已启动，Ctrl+C 停止')

    def _min(self, ranges, a, b):
        vals = [r for r in ranges[a:b] if r > 0.05]
        return min(vals) if vals else 999.0

    def scan_cb(self, msg):
        r = msg.ranges
        n = len(r)
        # 前方 90°: [3n/8, 5n/8]；左前 90°: [5n/8, 7n/8]；右前 90°: [n/8, 3n/8]
        self.front = self._min(r, 3 * n // 8, 5 * n // 8)
        self.left = self._min(r, 5 * n // 8, 7 * n // 8)
        self.right = self._min(r, n // 8, 3 * n // 8)

    def publish_cmd(self):
        cmd = Twist()
        if self.front > 0.5:
            # 前方安全，前进
            cmd.linear.x = self.speed
        elif self.front < 0.25:
            # 太近，后退一点并朝开阔侧转（避免撞墙打滑）
            cmd.linear.x = -0.1
            cmd.angular.z = self.turn if self.left > self.right else -self.turn
        else:
            # 中等距离，原地朝开阔侧转
            cmd.angular.z = self.turn if self.left > self.right else -self.turn
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = AutoMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停车
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
