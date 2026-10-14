#!/usr/bin/env python3
"""AIC 2026 智慧社区复赛：沿 official_route.yaml 低速行驶并进行 SLAM 建图。

定位：这是“建图阶段”控制器，不是最终复赛巡检程序。

设计原则：
- 不依赖 Nav2；严格沿 waypoint 折线行驶，不自由探索、不自行改道。
- /scan_clean 只做健康检查与紧急停车，白线约束由路线坐标与轨迹偏差保护保证。
- 以 official_route.yaml 的 START 作为局部 odom 原点。
- 对 /odom_clean、/scan_clean 做超时保护；无有效激光点时禁止行驶。
- 等待 /map 首帧和传感器稳定后才开始运动，避免 slam_toolbox 未初始化就起步。
- 检查 spawn 默认位姿与路线 START；不一致时默认阻止正式建图。
- 检查 /cmd_vel 是否有其它发布者，避免 teleop / 其它控制器与本节点抢控制权。
- 对当前路线段增加横向偏差保护，偏离过大立即停车而不是“自己找路”。
- 持续检查 /map publisher；SLAM 节点退出时立即停车。
- 前向激光扇区必须有足够有效点，避免 VMware 渲染异常时把全 inf 误判为畅通。
"""

import math
import os
import re
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
import yaml


SCAN_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    depth=20,
)
MAP_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    depth=1,
)


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def parse_launch_default(text, name):
    # DeclareLaunchArgument supplies the runtime default; inspect it before the
    # LaunchConfiguration fallback so a stale duplicate fallback cannot fool the guard.
    patterns = [
        rf"DeclareLaunchArgument\('{re.escape(name)}',\s*default_value='([^']+)'",
        rf"LaunchConfiguration\('{re.escape(name)}',\s*default='([^']+)'\)",
    ]
    for pat in patterns:
        m = re.search(pat, text)
        if m:
            try:
                return float(m.group(1))
            except ValueError:
                return None
    return None


class RouteMapping(Node):
    def __init__(self):
        super().__init__('route_mapping')

        # 对 4.2m × 4.2m 复赛场地偏保守的默认值。
        self.declare_parameter('max_linear_speed', 0.12)
        self.declare_parameter('min_linear_speed', 0.045)
        self.declare_parameter('max_angular_speed', 0.42)
        self.declare_parameter('max_tracking_angular_speed', 0.22)
        self.declare_parameter('waypoint_tolerance', 0.04)
        self.declare_parameter('yaw_tolerance', 0.05)
        self.declare_parameter('turn_in_place_threshold', 0.12)
        self.declare_parameter('max_cross_track_error', 0.06)
        self.declare_parameter('emergency_distance', 0.22)
        self.declare_parameter('emergency_release_distance', 0.27)
        self.declare_parameter('settle_time', 0.8)
        # 仅是“建图时在信号灯 waypoint 多停一下”，不是正式红灯识别。
        self.declare_parameter('traffic_waypoint_settle_time', 1.5)
        self.declare_parameter('startup_delay', 3.0)
        self.declare_parameter('sensor_timeout', 0.75)
        self.declare_parameter('min_valid_scan_points', 20)
        self.declare_parameter('min_valid_scan_fraction', 0.05)
        self.declare_parameter('min_valid_front_points', 3)
        self.declare_parameter('require_map_ready', True)
        self.declare_parameter('strict_spawn_match', True)
        self.declare_parameter('spawn_match_tolerance', 0.015)
        self.declare_parameter('spawn_yaw_tolerance', 0.02)
        self.declare_parameter('strict_cmd_vel_exclusive', True)
        self.declare_parameter('strict_map_publisher', True)
        self.declare_parameter('route_file', '')

        gp = lambda n: self.get_parameter(n).value
        self.max_lin = float(gp('max_linear_speed'))
        self.min_lin = float(gp('min_linear_speed'))
        self.max_ang = float(gp('max_angular_speed'))
        self.max_track_ang = float(gp('max_tracking_angular_speed'))
        self.wp_tol = float(gp('waypoint_tolerance'))
        self.yaw_tol = float(gp('yaw_tolerance'))
        self.turn_thresh = float(gp('turn_in_place_threshold'))
        self.max_cross_track_error = float(gp('max_cross_track_error'))
        self.emergency_dist = float(gp('emergency_distance'))
        self.emergency_release = float(gp('emergency_release_distance'))
        self.settle_time = float(gp('settle_time'))
        self.traffic_settle_time = float(gp('traffic_waypoint_settle_time'))
        self.startup_delay = float(gp('startup_delay'))
        self.sensor_timeout = float(gp('sensor_timeout'))
        self.min_valid_scan_points = int(gp('min_valid_scan_points'))
        self.min_valid_scan_fraction = float(gp('min_valid_scan_fraction'))
        self.min_valid_front_points = int(gp('min_valid_front_points'))
        self.require_map_ready = bool(gp('require_map_ready'))
        self.strict_spawn_match = bool(gp('strict_spawn_match'))
        self.spawn_match_tolerance = float(gp('spawn_match_tolerance'))
        self.spawn_yaw_tolerance = float(gp('spawn_yaw_tolerance'))
        self.strict_cmd_vel_exclusive = bool(gp('strict_cmd_vel_exclusive'))
        self.strict_map_publisher = bool(gp('strict_map_publisher'))

        route_file = str(gp('route_file')).strip()
        if not route_file:
            route_file = os.path.join(
                get_package_share_directory('smart_community_gazebo'),
                'config', 'official_route.yaml')
        self.route_file = route_file
        self.raw_route = self._load_route(route_file)
        self.route = self._convert_route_to_odom(self.raw_route)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom_clean', self.odom_cb, 20)
        self.create_subscription(LaserScan, '/scan_clean', self.scan_cb, SCAN_QOS)
        self.create_subscription(OccupancyGrid, '/map', self.map_cb, MAP_QOS)

        self.pose = None
        self.scan = None
        self.last_odom_rx = None
        self.last_scan_rx = None
        self.map_ready = False
        self.map_info = None
        self.data_ready_since = None
        self.index = 0
        self.phase = 'POSITION'
        self.settle_until = None
        self.emergency_active = False
        self.route_deviation_active = False
        self.last_health_log = 0.0
        self.last_block_log = 0.0
        self.last_graph_check = 0.0
        self.cmd_vel_conflict = False
        self.cmd_vel_conflict_names = []
        self.map_publisher_alive = False
        self.map_publisher_names = []
        self.complete = False

        self.spawn_match_ok, self.spawn_match_reason = self._check_spawn_match()

        self.create_timer(0.05, self.step)  # 20 Hz 控制
        self.get_logger().info(f'official route: {self.route_file}')
        self.get_logger().warn(
            'official_route.yaml frame_id=map is a legacy project label; during mapping its x/y are treated '
            'as Gazebo-world geometry and converted to local odom. Do not reuse raw x/y directly in final Nav2.')
        self.get_logger().info(f'loaded {len(self.route)} waypoints')
        self.get_logger().info(
            f'route safety: waypoint_tol={self.wp_tol:.3f}m, '
            f'max_cross_track={self.max_cross_track_error:.3f}m, '
            f'emergency_front={self.emergency_dist:.3f}m')
        if self.spawn_match_ok:
            self.get_logger().info('START/SPAWN guard: OK')
        else:
            self.get_logger().error('START/SPAWN guard: ' + self.spawn_match_reason)
            if self.strict_spawn_match:
                self.get_logger().error('FORMAL_MAPPING_BLOCKED until START/SPAWN are unified')
        self._log_route()

    def _load_route(self, path):
        if not os.path.isfile(path):
            raise RuntimeError(f'official route file not installed: {path}')
        with open(path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
        if not isinstance(data, dict) or data.get('frame_id') != 'map':
            raise RuntimeError(f'route frame_id must be map: {path}')
        route = data.get('route')
        if not route or len(route) < 2:
            raise RuntimeError(f'official route invalid or empty: {path}')
        required = ('name', 'x', 'y', 'yaw', 'action')
        names = set()
        for i, p in enumerate(route):
            missing = [k for k in required if k not in p]
            if missing:
                raise RuntimeError(f'route[{i}] missing fields: {missing}')
            if str(p['name']) in names:
                raise RuntimeError(f'duplicate waypoint name: {p["name"]}')
            names.add(str(p['name']))
        return route

    def _convert_route_to_odom(self, route):
        start = route[0]
        x0, y0, yaw0 = float(start['x']), float(start['y']), float(start['yaw'])
        c, s = math.cos(yaw0), math.sin(yaw0)
        converted = []
        for p in route:
            dx = float(p['x']) - x0
            dy = float(p['y']) - y0
            converted.append({
                'name': str(p['name']),
                'x': c * dx + s * dy,
                'y': -s * dx + c * dy,
                'yaw': wrap_angle(float(p['yaw']) - yaw0),
                'action': str(p['action']),
            })
        return converted

    def _check_spawn_match(self):
        try:
            launch_path = os.path.join(
                get_package_share_directory('robot_description'),
                'launch', 'spawn_robot.launch.py')
            text = open(launch_path, 'r', encoding='utf-8').read()
            sx = parse_launch_default(text, 'x')
            sy = parse_launch_default(text, 'y')
            syaw = parse_launch_default(text, 'yaw')
        except Exception as e:
            return False, f'cannot inspect spawn defaults: {e}'

        if None in (sx, sy, syaw):
            return False, 'cannot parse spawn default x/y/yaw'

        start = self.raw_route[0]
        rx, ry, ryaw = float(start['x']), float(start['y']), float(start['yaw'])
        pos_delta = math.hypot(sx - rx, sy - ry)
        yaw_delta = abs(wrap_angle(syaw - ryaw))
        if pos_delta > self.spawn_match_tolerance or yaw_delta > self.spawn_yaw_tolerance:
            return False, (
                f'spawn=({sx:.3f},{sy:.3f},{syaw:.3f}) vs '
                f'route START=({rx:.3f},{ry:.3f},{ryaw:.3f}), '
                f'delta={pos_delta:.3f}m/{yaw_delta:.3f}rad')
        return True, (
            f'spawn and route START aligned: delta={pos_delta:.3f}m/{yaw_delta:.3f}rad')

    def _log_route(self):
        s = self.raw_route[0]
        self.get_logger().info(
            f"world->odom anchor START=({float(s['x']):.3f}, {float(s['y']):.3f}, {float(s['yaw']):.3f})")
        for i, p in enumerate(self.route):
            self.get_logger().info(
                f"[{i:02d}] {p['name']}: odom=({p['x']:.3f}, {p['y']:.3f}, {p['yaw']:.3f}) "
                f"action={p['action']}")

    def odom_cb(self, msg):
        self.pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.last_odom_rx = time.monotonic()

    def scan_cb(self, msg):
        self.scan = msg
        self.last_scan_rx = time.monotonic()

    def map_cb(self, msg):
        if msg.info.width > 0 and msg.info.height > 0 and msg.info.resolution > 0.0:
            first = not self.map_ready
            self.map_ready = True
            self.map_info = (msg.info.width, msg.info.height, msg.info.resolution)
            if first:
                self.get_logger().info(
                    f'/map ready: {msg.info.width}x{msg.info.height}, '
                    f'resolution={msg.info.resolution:.3f}m')

    def scan_valid_count(self):
        if self.scan is None:
            return 0
        rmin = max(0.03, float(self.scan.range_min))
        rmax = float(self.scan.range_max) if self.scan.range_max > 0 else math.inf
        return sum(1 for r in self.scan.ranges if math.isfinite(r) and rmin <= r <= rmax)

    def front_stats(self, half_angle_deg=18.0):
        """Return (minimum_range, valid_count) in the forward safety sector.

        v3 treated an all-inf forward sector as "clear". On VMware GPU LiDAR this
        can be unsafe because a corrupt render frame may contain valid side rays but
        no usable forward rays. v4 therefore refuses to drive without enough valid
        forward samples.
        """
        if self.scan is None or not self.scan.ranges:
            return math.inf, 0
        half = math.radians(half_angle_deg)
        best = math.inf
        count = 0
        a0 = float(self.scan.angle_min)
        inc = float(self.scan.angle_increment)
        rmin = max(0.03, float(self.scan.range_min))
        rmax = float(self.scan.range_max) if self.scan.range_max > 0 else math.inf
        for i, r in enumerate(self.scan.ranges):
            ang = a0 + i * inc
            if abs(wrap_angle(ang)) > half:
                continue
            if not math.isfinite(r) or r < rmin or r > rmax:
                continue
            count += 1
            best = min(best, r)
        return best, count

    def front_min(self, half_angle_deg=18.0):
        return self.front_stats(half_angle_deg)[0]

    def stop(self):
        self.pub.publish(Twist())

    @staticmethod
    def clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def angular_cmd(self, err, gain=1.7):
        w = self.clamp(gain * err, -self.max_ang, self.max_ang)
        if abs(err) > self.yaw_tol and 0.0 < abs(w) < 0.10:
            w = math.copysign(0.10, w)
        return w

    def sensors_healthy(self, now):
        if self.pose is None or self.scan is None or self.last_odom_rx is None or self.last_scan_rx is None:
            return False, 'waiting for /odom_clean and /scan_clean'
        if now - self.last_odom_rx > self.sensor_timeout:
            return False, f'/odom_clean stale ({now - self.last_odom_rx:.2f}s)'
        if now - self.last_scan_rx > self.sensor_timeout:
            return False, f'/scan_clean stale ({now - self.last_scan_rx:.2f}s)'
        valid = self.scan_valid_count()
        needed = max(self.min_valid_scan_points, int(math.ceil(len(self.scan.ranges) * self.min_valid_scan_fraction)))
        if valid < needed:
            return False, f'/scan_clean unhealthy: only {valid} valid ranges (need >= {needed})'
        return True, ''

    def check_graph_health(self, now):
        """Check command exclusivity and that SLAM still owns /map."""
        if now - self.last_graph_check < 1.0:
            return
        self.last_graph_check = now

        if self.strict_cmd_vel_exclusive:
            others = []
            try:
                for info in self.get_publishers_info_by_topic('/cmd_vel'):
                    if info.node_name != self.get_name():
                        others.append(f'{info.node_namespace}/{info.node_name}'.replace('//', '/'))
            except Exception as e:
                self.get_logger().warn(f'cmd_vel publisher check failed: {e}')
            else:
                others = sorted(set(others))
                conflict = bool(others)
                if conflict != self.cmd_vel_conflict or others != self.cmd_vel_conflict_names:
                    self.cmd_vel_conflict = conflict
                    self.cmd_vel_conflict_names = others
                    if conflict:
                        self.get_logger().error('CMD_VEL_CONFLICT: other publishers=' + ', '.join(others))
                    else:
                        self.get_logger().info('CMD_VEL_CONFLICT cleared')

        if self.strict_map_publisher:
            pubs = []
            try:
                for info in self.get_publishers_info_by_topic('/map'):
                    pubs.append(f'{info.node_namespace}/{info.node_name}'.replace('//', '/'))
            except Exception as e:
                self.get_logger().warn(f'/map publisher check failed: {e}')
            else:
                pubs = sorted(set(pubs))
                slam_pubs = [n for n in pubs if 'slam_toolbox' in n.lower()]
                alive = bool(slam_pubs)
                if alive != self.map_publisher_alive or pubs != self.map_publisher_names:
                    self.map_publisher_alive = alive
                    self.map_publisher_names = pubs
                    if alive:
                        self.get_logger().info('/map slam_toolbox publisher(s): ' + ', '.join(slam_pubs))
                    else:
                        self.get_logger().error(
                            'MAP_PUBLISHER_MISSING: /map has no active slam_toolbox publisher; '
                            f'all publishers={pubs}')

    @staticmethod
    def point_to_segment_distance(px, py, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        vv = vx * vx + vy * vy
        if vv < 1e-12:
            return math.hypot(px - ax, py - ay)
        t = max(0.0, min(1.0, (wx * vx + wy * vy) / vv))
        qx, qy = ax + t * vx, ay + t * vy
        return math.hypot(px - qx, py - qy)

    def cross_track_error(self):
        if self.index <= 0 or self.index >= len(self.route) or self.pose is None:
            return 0.0
        a = self.route[self.index - 1]
        b = self.route[self.index]
        x, y, _ = self.pose
        return self.point_to_segment_distance(x, y, a['x'], a['y'], b['x'], b['y'])

    def reach_position(self, target):
        self.stop()
        self.phase = 'ALIGN'
        x, y, yaw = self.pose
        self.get_logger().info(
            f"position reached {target['name']} current=({x:.3f},{y:.3f},{yaw:.3f}) "
            f"target=({target['x']:.3f},{target['y']:.3f},{target['yaw']:.3f}) "
            f"action={target['action']}")

    def reach_waypoint(self, target):
        self.stop()
        if target['action'] == 'traffic_light_wait':
            delay = self.traffic_settle_time
            self.get_logger().warn(
                'MAPPING_ONLY_TRAFFIC_PAUSE: fixed pause for SLAM only; '
                'this is NOT red/green recognition and must not be used as final patrol compliance.')
        else:
            delay = self.settle_time
        self.phase = 'SETTLE'
        self.settle_until = time.monotonic() + delay
        x, y, yaw = self.pose
        self.get_logger().info(
            f"WAYPOINT_REACHED {target['name']} current=({x:.3f},{y:.3f},{yaw:.3f}) "
            f"target=({target['x']:.3f},{target['y']:.3f},{target['yaw']:.3f}) "
            f"action={target['action']} settle={delay:.1f}s")

    def finish(self):
        self.stop()
        self.complete = True
        self.get_logger().info('ROUTE_MAPPING_COMPLETE')
        self.get_logger().info('机器人已停车；可以保存 slam_toolbox 发布的 /map。')

    def step(self):
        if self.complete:
            self.stop()
            return

        now = time.monotonic()
        self.check_graph_health(now)

        if self.strict_spawn_match and not self.spawn_match_ok:
            self.data_ready_since = None
            self.stop()
            if now - self.last_block_log > 3.0:
                self.get_logger().error('FORMAL_MAPPING_BLOCKED: ' + self.spawn_match_reason)
                self.last_block_log = now
            return

        if self.cmd_vel_conflict:
            self.data_ready_since = None
            self.stop()
            if now - self.last_block_log > 3.0:
                self.get_logger().error('FORMAL_MAPPING_BLOCKED: /cmd_vel has another publisher')
                self.last_block_log = now
            return

        if self.strict_map_publisher and not self.map_publisher_alive:
            self.data_ready_since = None
            self.stop()
            if now - self.last_block_log > 3.0:
                self.get_logger().error('FORMAL_MAPPING_BLOCKED: no active /map publisher')
                self.last_block_log = now
            return

        healthy, reason = self.sensors_healthy(now)
        if not healthy:
            self.data_ready_since = None
            self.stop()
            if now - self.last_health_log > 2.0:
                self.get_logger().warn(reason)
                self.last_health_log = now
            return

        if self.require_map_ready and not self.map_ready:
            self.data_ready_since = None
            self.stop()
            if now - self.last_health_log > 2.0:
                self.get_logger().warn('waiting for first valid /map from slam_toolbox')
                self.last_health_log = now
            return

        if self.data_ready_since is None:
            self.data_ready_since = now
            self.get_logger().info(
                f'system ready; holding still {self.startup_delay:.1f}s before route motion')
        if now - self.data_ready_since < self.startup_delay:
            self.stop()
            return

        if self.index >= len(self.route):
            self.finish()
            return

        target = self.route[self.index]
        x, y, yaw = self.pose

        if self.phase == 'SETTLE':
            self.stop()
            if self.settle_until is not None and now >= self.settle_until:
                self.index += 1
                if self.index >= len(self.route):
                    self.finish()
                else:
                    self.phase = 'POSITION'
                    self.settle_until = None
            return

        if self.phase == 'ALIGN':
            err = wrap_angle(target['yaw'] - yaw)
            if abs(err) <= self.yaw_tol:
                self.reach_waypoint(target)
                return
            cmd = Twist()
            cmd.angular.z = self.angular_cmd(err, gain=1.8)
            self.pub.publish(cmd)
            return

        # POSITION：先检查是否明显偏离当前官方路线段。
        cte = self.cross_track_error()
        if self.index > 0 and cte > self.max_cross_track_error:
            self.route_deviation_active = True
            self.stop()
            if now - self.last_block_log > 2.0:
                self.get_logger().error(
                    f'ROUTE_DEVIATION_STOP cross_track={cte:.3f}m > '
                    f'{self.max_cross_track_error:.3f}m at segment '
                    f'{self.route[self.index-1]["name"]}->{target["name"]}')
                self.last_block_log = now
            return
        elif self.route_deviation_active:
            self.route_deviation_active = False
            self.get_logger().info(f'route deviation cleared: cross_track={cte:.3f}m')

        front, front_valid = self.front_stats()
        if front_valid < self.min_valid_front_points:
            self.stop()
            if now - self.last_health_log > 2.0:
                self.get_logger().error(
                    f'FRONT_SCAN_UNHEALTHY: only {front_valid} valid forward ranges; refusing to drive blind')
                self.last_health_log = now
            return
        if self.emergency_active:
            if front >= self.emergency_release:
                self.emergency_active = False
                self.get_logger().info(f'emergency cleared: front={front:.3f}m')
            else:
                self.stop()
                return

        if front < self.emergency_dist:
            self.emergency_active = True
            self.stop()
            self.get_logger().error(
                f'EMERGENCY_STOP front={front:.3f}m < {self.emergency_dist:.3f}m; '
                'controller will not deviate from official route')
            return

        dx, dy = target['x'] - x, target['y'] - y
        dist = math.hypot(dx, dy)
        if dist <= self.wp_tol:
            self.reach_position(target)
            return

        desired = math.atan2(dy, dx)
        err = wrap_angle(desired - yaw)
        cmd = Twist()

        if abs(err) > self.turn_thresh:
            # 大角度误差先原地转，避免切角压线。
            cmd.angular.z = self.angular_cmd(err)
        else:
            v = self.clamp(0.75 * dist, self.min_lin, self.max_lin)
            if dist < 0.18:
                v = min(v, 0.075)
            heading_scale = max(0.35, 1.0 - abs(err) / self.turn_thresh)
            cmd.linear.x = v * heading_scale
            cmd.angular.z = self.clamp(1.25 * err, -self.max_track_ang, self.max_track_ang)
        self.pub.publish(cmd)

    def destroy_node(self):
        try:
            for _ in range(3):
                self.stop()
                time.sleep(0.03)
        except Exception:
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RouteMapping()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
