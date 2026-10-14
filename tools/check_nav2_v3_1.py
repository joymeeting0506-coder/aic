#!/usr/bin/env python3
"""Non-motion runtime check independent of ros2 daemon.
Never publishes velocity, goals, or initial pose.
"""
import math
import sys
import time
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from tf2_ros import Buffer, TransformListener
import rclpy
from rclpy.node import Node


class Diagnostics(Node):
    def __init__(self):
        super().__init__('aic_v31_nav2_checker', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        qmap = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.map_size = None
        self.odom_ok = False
        self.pose_seen = False
        self.scan_ok = False
        self.scan_desc = 'waiting for /scan_clean'
        self.create_subscription(OccupancyGrid, '/map', self.cb_map, qmap)
        self.create_subscription(Odometry, '/odom_clean', self.cb_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_pose, 10)
        self.create_subscription(LaserScan, '/scan_clean', self.cb_scan, qos_profile_sensor_data)
        self.buf = Buffer()
        self.tf = TransformListener(self.buf, self, spin_thread=False)

    def cb_map(self, msg):
        self.map_size = (msg.info.width, msg.info.height, msg.info.resolution)

    def cb_odom(self, msg):
        self.odom_ok = msg.header.frame_id == 'odom' and msg.child_frame_id in ('base_footprint', 'base_link')

    def cb_pose(self, msg):
        self.pose_seen = msg.header.frame_id == 'map'

    def cb_scan(self, msg):
        vals = [r for r in msg.ranges if math.isfinite(r) and msg.range_min <= r <= msg.range_max]
        if vals:
            mn, mx = min(vals), max(vals)
            self.scan_desc = f'rays={len(msg.ranges)} valid={len(vals)} min={mn:.3f} max={mx:.3f}'
            self.scan_ok = len(vals) >= 100 and mx > .50 and mx - mn > .20

    def endpoints(self, topic):
        pubs = sorted({ep.node_name for ep in self.get_publishers_info_by_topic(topic)})
        subs = sorted({ep.node_name for ep in self.get_subscriptions_info_by_topic(topic)})
        print(f'{topic}: pub={pubs}, sub={subs}', flush=True)
        return pubs, subs

    def state(self, name):
        service = f'/{name}/get_state'
        client = self.create_client(GetState, service)
        try:
            if not client.wait_for_service(timeout_sec=3.0):
                print(f'FAIL lifecycle {name}: service not available', flush=True)
                return False
            fut = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            if not fut.done() or fut.result() is None:
                print(f'FAIL lifecycle {name}: timed out', flush=True)
                return False
            label = fut.result().current_state.label
            print(f'Lifecycle {name}: {label}', flush=True)
            return label == 'active'
        finally:
            self.destroy_client(client)


def main():
    rclpy.init()
    d = Diagnostics()
    bad = []
    frames = [('map', 'odom'), ('odom', 'base_footprint'),
              ('base_footprint', 'lidar_link'), ('map', 'base_footprint')]
    try:
        start = time.monotonic()
        while time.monotonic() - start < 15.0:
            rclpy.spin_once(d, timeout_sec=.2)
            if d.map_size and d.scan_ok and d.odom_ok and all(d.buf.can_transform(a,b,Time()) for a,b in frames):
                break
        if d.map_size and d.map_size[:2] == (85, 85):
            print(f'OK /map {d.map_size}', flush=True)
        else:
            print(f'FAIL /map: {d.map_size}', flush=True)
            bad.append('/map')
        print(('OK' if d.scan_ok else 'FAIL') + ' /scan_clean ' + d.scan_desc, flush=True)
        if not d.scan_ok: bad.append('/scan_clean')
        print(('OK' if d.odom_ok else 'FAIL') + ' /odom_clean', flush=True)
        if not d.odom_ok: bad.append('/odom_clean')
        for a,b in frames:
            good = d.buf.can_transform(a, b, Time())
            print(f'{"OK" if good else "FAIL"} TF {a}->{b}', flush=True)
            if not good: bad.append(f'TF {a}->{b}')
        print('AMCL /amcl_pose: '+ ('received' if d.pose_seen else 'not received during short sampling (warning; may not update while stationary)'), flush=True)
        for n in ('map_server', 'amcl', 'planner_server', 'controller_server',
                  'bt_navigator', 'velocity_smoother', 'collision_monitor'):
            if not d.state(n): bad.append('lifecycle '+n)
        pubs, subs = d.endpoints('/cmd_vel')
        if pubs != ['collision_monitor'] or 'ros_gz_bridge' not in subs:
            bad.append('/cmd_vel wiring: require only collision_monitor -> ros_gz_bridge')
        pubs, subs = d.endpoints('/cmd_vel_smoothed')
        if pubs != ['velocity_smoother'] or 'collision_monitor' not in subs:
            bad.append('/cmd_vel_smoothed wiring')
        pubs, subs = d.endpoints('/cmd_vel_nav')
        if 'controller_server' not in pubs or 'velocity_smoother' not in subs:
            bad.append('/cmd_vel_nav wiring')
        present = d.get_node_names()
        if 'docking_server' in present:
            bad.append('unexpected docking_server')
        if 'slam_toolbox' in present or 'async_slam_toolbox_node' in present:
            bad.append('slam_toolbox should not run in navigation mode')
        print('=== RESULT ===', flush=True)
        if bad:
            for e in bad: print('FAIL:',e,flush=True)
            print('NAV2_CHECK_FAIL', flush=True)
            return 1
        print('NAV2_CHECK_PASS (runtime infrastructure only; NOT competition lane/traffic validation)',flush=True)
        return 0
    finally:
        d.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    sys.exit(main())
