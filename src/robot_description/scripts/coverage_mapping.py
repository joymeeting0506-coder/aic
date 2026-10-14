#!/usr/bin/env python3
"""确定性全覆盖建图节点（蛇形扫场）。

用 odom（相对 spawn 点起算，见 WAYPOINTS 注释）做 go-to-point 控制，让小车沿预设
蛇形路径扫过整个 4.2m × 4.2m 场地。速度慢 + 原地转向，尽量降低 SLAM 重影/错位；
前方遇到薄障碍（红绿灯杆等）时原地转向开阔侧绕行，避免卡死。

跑完一轮蛇形后停车，等待用户用 map_saver_cli 保存地图。

路径避开：右停车区(x>3.6)、人偶A区(x∈[0.75,1.76], y∈[3.05,3.47])、
          人偶B区(x∈[0.76,1.58], y∈[1.7,2.18])（均为世界坐标）。
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

# 蛇形覆盖路径（odom 坐标，m）。mapping.launch.py 在左下角 spawn：世界 (0.4, 0.4)
# 朝东 yaw=0，gz DiffDrive 的 odom 相对 spawn 从 (0,0) 起、odom +x = 东 = 世界 +x，
# 因此 odom = 世界坐标 − 0.4。覆盖世界 x∈[0.4,3.8]、y∈[0.4,3.8]（避开右停车区
# x>3.6、人偶A区 x∈[0.75,1.76] y∈[3.05,3.47]、人偶B区 x∈[0.76,1.58] y∈[1.7,2.18]）。
# 四周外墙 + 右侧三块车牌车面板 + 中部人偶立牌都要被 LiDAR 近距离扫到，
# 故走一圈外围 + 一条中央竖路 + 一条中低横穿。
WAYPOINTS = [
    (0.0, 0.0),     # 0 底部左（起点 = 世界 0.4,0.4）
    (3.0, 0.0),     # 1 底部路东向
    (3.0, 0.7),     # 2
    (0.0, 0.7),     # 3 中低横穿西向（扫右侧停车区车面板）
    (0.0, 2.2),     # 4
    (3.4, 2.2),     # 5 右路东向（世界 3.8，贴近右墙，扫清右上角伪影）
    (3.4, 3.4),     # 6 右上角（世界 3.8,3.8，扫起点区 3.88,3.88）
    (3.0, 3.35),    # 7 回撤到顶路（避免斜穿红绿灯杆 (1.6,3.48)）
    (0.0, 3.35),    # 8 顶部路西向
    (0.0, 2.2),     # 9 左侧南下
    (2.15, 2.2),    # 10 中央路东侧入口（世界 2.55，绕到红绿灯杆东侧）
    (2.15, 0.0),    # 11 中央路东侧南下（红绿灯杆 span 世界 x∈[1.69,2.31]，走其东侧）
    (0.0, 0.0),     # 12 回起点（闭环，利于 SLAM 回环）
]


class CoverageMapper(Node):
    def __init__(self):
        super().__init__('coverage_mapper')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan_clean', self.scan_cb, SCAN_QOS)
        self.create_subscription(Odometry, '/odom_clean', self.odom_cb, 20)

        self.lin = 0.15          # 慢速直线，降低打滑/重影
        self.turn = 0.5          # 原地转向角速度
        self.arrive_dist = 0.15  # 到点判据（m）
        self.too_close = 0.22    # 前方障碍停车距离（m）

        self.waypoint_idx = 0
        self.ranges = []
        self.pose = None         # (x, y, yaw)
        self.state = 'GOTO'
        self.turn_dir = 1.0
        self.open_since = None
        self.done = False

        self.create_timer(0.15, self.step)
        self.get_logger().info('coverage_mapper 启动，开始蛇形全覆盖建图')

    # ------------------------------------------------------------------
    def odom_cb(self, m):
        q = m.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

    def scan_cb(self, m):
        self.ranges = list(m.ranges)

    def _sector(self, lo_deg, hi_deg):
        """返回 [lo_deg, hi_deg] 扇形内最小有效距离（m），无有效值返回 999。"""
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

    def _goal(self):
        """返回 (到目标点距离, 目标航向与当前朝向的误差角)。"""
        wx, wy = WAYPOINTS[self.waypoint_idx]
        dx = wx - self.pose[0]
        dy = wy - self.pose[1]
        dist = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)
        err = ang - self.pose[2]
        err = math.atan2(math.sin(err), math.cos(err))
        return dist, err

    def _set_state(self, s):
        if s != self.state:
            self.get_logger().info(f'{self.state} -> {s}')
        self.state = s
        self.open_since = None

    def step(self):
        if self.done or self.pose is None:
            return
        cmd = Twist()
        now = time.time()

        if self.state == 'GOTO':
            dist, err = self._goal()
            if dist < self.arrive_dist:
                self.waypoint_idx += 1
                if self.waypoint_idx >= len(WAYPOINTS):
                    self.done = True
                    self.get_logger().info('蛇形建图完成，已停车。请用 map_saver_cli 保存地图。')
                    self.pub.publish(Twist())
                    return
                self.get_logger().info(
                    f'到点，前往 {WAYPOINTS[self.waypoint_idx]}')
                return

            # 前方有薄障碍（红绿灯杆等）：原地转向更开阔的一侧绕行
            front = self._sector(-25, 25)
            if front < self.too_close:
                self.turn_dir = 1.0 if self._sector(20, 110) >= self._sector(-110, -20) else -1.0
                self._set_state('TURN')
                return

            if abs(err) > 0.2:
                cmd.angular.z = self.turn * (1.0 if err > 0 else -1.0)
            else:
                cmd.linear.x = self.lin

        elif self.state == 'TURN':
            cmd.angular.z = self.turn * self.turn_dir
            front = self._sector(-25, 25)
            if front > self.too_close + 0.15:
                if self.open_since is None:
                    self.open_since = now
                elif now - self.open_since > 0.4:
                    self._set_state('GOTO')
            else:
                self.open_since = None

        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CoverageMapper()
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
