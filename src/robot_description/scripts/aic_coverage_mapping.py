#!/usr/bin/env python3
"""AIC mapping-stage autonomous road-network coverage, ROS 2 Jazzy.

Uses *live* slam_toolbox map + TF + scan; DOES NOT load a saved map.
The current Gazebo edition uses a pre-checked road-centreline safety geofence.
It cannot recognize painted lanes in an unknown physical site: DO NOT use on
real hardware without a certified live road/line perception safety layer.

Run with drive:=false first (DRY RUN). Use drive:=true only under supervision.
"""
import json
import hashlib
import math
import os
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException

from aic_coverage_core import (Grid, masks, road_samples, select_target,
                               closest_cell, astar, path_distance, polyline_dist)

MAP_QOS=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
SCAN_QOS=QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT)


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))


def wrap(a):
    return math.atan2(math.sin(a),math.cos(a))


def segment_distance(p,a,b):
    return polyline_dist(*p,[a,b])


class AutonomousCoverage(Node):
    def __init__(self):
        super().__init__('aic_coverage_mapping')
        d=self.declare_parameter
        d('drive',False)
        d('graph_path','')
        d('resume_legacy_route',False)
        d('linear_speed',.075)
        d('angular_speed',.30)
        d('robot_radius',.22)
        d('road_center_tolerance',.070)
        d('emergency_stop',.29)
        d('sensor_timeout',1.0)
        d('map_timeout',30.0)
        d('save_output_dir','')
        d('handoff_path','')
        d('preview_status_path','')
        d('return_home',True)
        self.driving=bool(self.p('drive'))
        src=str(self.p('graph_path'))
        if not src or not os.path.isfile(src):
            raise RuntimeError('A checked simulation road graph is REQUIRED: graph_path')
        raw=json.loads(Path(src).read_text(encoding='utf-8'))
        if raw.get('schema') != 'aic_sim_road_graph_v1':
            raise RuntimeError('Invalid road graph schema')
        self.segments=[(tuple(map(float,seg[0])),tuple(map(float,seg[1])))
                       for seg in raw['road_segments_map']]
        self.samples=road_samples(self.segments,step=.28)
        self.legacy=[tuple(map(float,p)) for p in raw.get('legacy_route_map',[])]
        self.visited=set()
        if self.p('resume_legacy_route'):
            if len(self.legacy)<2:
                raise RuntimeError('resume_legacy_route: missing logged route data')
            # Only for a session where the old 17-point mapping run JUST finished!
            for i,p in enumerate(self.samples):
                if polyline_dist(*p,self.legacy)<.19:
                    self.visited.add(i)
        self.rejected=set()
        self.grid=None
        self.valid=set()
        self.map_rx=None
        self.scan_rx=None
        self.odom_rx=None
        self.scan=None
        self.pose=None
        self.target=None
        self.path=[]
        self.last_plan=0
        self.last_report=0
        self.last_error=0
        self.running=False
        self.anchor_verified=False
        self.preview_reported=False
        self.finished=False
        self.failed=False
        self.returning=False
        self.saving=False
        self.save_done=False
        self.tf_buffer=Buffer()
        self.tf_listener=TransformListener(self.tf_buffer,self)
        self.submap=self.create_subscription(OccupancyGrid,'/map',self.on_map,MAP_QOS)
        self.subscan=self.create_subscription(LaserScan,'/scan_clean',self.on_scan,SCAN_QOS)
        self.subodom=self.create_subscription(Odometry,'/odom_clean',self.on_odom,SCAN_QOS)
        # Avoid stealing another controller's /cmd_vel. Do not publish until gates pass.
        self.pub=self.create_publisher(Twist,'/cmd_vel',10)
        self.timer=self.create_timer(.10,self.tick)
        self.get_logger().info(
            f'AIC ROAD-COVERAGE edition=SIM only; samples={len(self.samples)} '
            f'previously_traversed={len(self.visited)} drive={self.driving}. '
            'Only LIVE SLAM /map is used. Default is motionless preview.')

    def p(self,k):return self.get_parameter(k).value

    def stop(self):
        # Context may already be invalid after SIGINT or after map save.
        if not rclpy.ok():
            return
        try:
            self.pub.publish(Twist())
        except Exception:
            # Ignore shutdown races only; preserve live-context failures.
            if rclpy.ok():
                raise

    def on_map(self,msg):
        if msg.header.frame_id!='map':
            self.warn('Unexpected map frame; refusing to drive')
            return
        o=msg.info.origin
        self.grid=Grid(msg.info.width,msg.info.height,msg.info.resolution,
                       o.position.x,o.position.y,yaw(o.orientation),msg.data)
        self.map_rx=time.monotonic()
        self.valid=masks(self.grid,self.segments,
                         corridor_radius=float(self.p('road_center_tolerance')),
                         robot_radius=float(self.p('robot_radius')))

    def on_scan(self,msg):
        self.scan=msg;self.scan_rx=time.monotonic()

    def on_odom(self,msg):
        self.odom_rx=time.monotonic()

    def warn(self,msg):
        t=time.monotonic()
        if t-self.last_error>3:
            self.get_logger().warning(msg)
            self.last_error=t

    def fresh(self):
        now=time.monotonic()
        timeout=float(self.p('sensor_timeout'))
        if self.grid is None or self.map_rx is None or now-self.map_rx>float(self.p('map_timeout')):
            return False,'no fresh live SLAM map'
        if self.scan_rx is None or now-self.scan_rx>timeout:
            return False,'stale /scan_clean'
        if self.odom_rx is None or now-self.odom_rx>timeout:
            return False,'stale /odom_clean'
        # Static maps from map_server must NOT be mistaken for live mapping.
        infos=self.get_publishers_info_by_topic('/map')
        if len(infos)!=1:
            return False,'expected exactly one live /map publisher'
        if not any('slam_toolbox' in info.node_name for info in infos):
            return False,'/map is not published by live slam_toolbox'
        pubs=self.get_publishers_info_by_topic('/cmd_vel')
        others=[x.node_name for x in pubs if x.node_name!=self.get_name()]
        if others:
            return False,'competing /cmd_vel publisher: '+', '.join(others)
        if self.count_subscribers('/cmd_vel')==0:
            return False,'/cmd_vel has no bridge subscriber'
        if self.scan is None:return False,'no laser'
        valid=[v for v in self.scan.ranges if math.isfinite(v)
               and self.scan.range_min<=v<=self.scan.range_max]
        if len(valid)<120 or (max(valid)-min(valid))<.05:
            return False,'invalid /scan_clean distribution'
        return True,''

    def safety_distance(self):
        msg=self.scan
        forward=[]
        for k,x in enumerate(msg.ranges):
            a=wrap(msg.angle_min+k*msg.angle_increment)
            if abs(a)<math.radians(26):
                if not math.isfinite(x):
                    # Inf is permitted for no return, but cannot count as valid evidence.
                    continue
                if msg.range_min<=x<=msg.range_max:
                    forward.append(x)
        if len(forward)<3:return None
        return min(forward)

    def read_pose(self):
        try:
            tf=self.tf_buffer.lookup_transform('map','base_footprint',Time())
        except TransformException as exc:
            self.warn('missing map->base_footprint TF: '+str(exc))
            return None
        a=tf.transform.translation
        # odom fresh + TF lookup protects against stale location in current session.
        return (a.x,a.y,yaw(tf.transform.rotation))

    def update_visited(self):
        for i,p in enumerate(self.samples):
            if math.dist(self.pose[:2],p)<.18:
                self.visited.add(i)

    def save(self):
        if self.saving:return
        self.saving=True
        self.stop()
        # Async so ROS executor keeps serving callbacks and Gazebo can keep running.
        root=Path(str(self.p('save_output_dir')) or
                  str(Path.home()/'aic_ws/aic_results/autonomous_mapping'))
        root.mkdir(parents=True,exist_ok=True)
        prefix=str(root/('coverage_'+datetime.now().strftime('%Y%m%d_%H%M%S')))
        def worker():
            try:
                res=subprocess.run(['ros2','run','nav2_map_server','map_saver_cli',
                                    '-f',prefix,'--ros-args',
                                    '-p','save_map_timeout:=15.0'],
                                   text=True,capture_output=True,timeout=35)
                yaml_path=Path(prefix+'.yaml');pgm_path=Path(prefix+'.pgm')
                if res.returncode!=0 or not yaml_path.is_file() or not pgm_path.is_file():
                    raise RuntimeError('Map save failed: '+(res.stderr or res.stdout)[-1200:])
                if yaml_path.stat().st_size<60 or pgm_path.stat().st_size<100:
                    raise RuntimeError('Map save output too small')
                handoff=str(self.p('handoff_path'))
                if not handoff:
                    self.get_logger().info('MAP_SAVED_NO_HANDOFF '+str(yaml_path))
                else:
                    if not self.returning:
                        raise RuntimeError('Cannot handoff without confirmed return to START')
                    # Map and geometry are in current SLAM map frame, not Gazebo-world.
                    pose=list(self.pose)
                    if math.hypot(pose[0],pose[1])>.09 or abs(wrap(pose[2]))>.14:
                        raise RuntimeError('START pose guard failed before handoff')
                    manifest={'schema':'aic_slam_to_nav2_v2','complete':True,
                        'map_yaml':str(yaml_path.resolve()),
                        'map_pgm':str(pgm_path.resolve()),
                        'map_sha256':hashlib.sha256(pgm_path.read_bytes()).hexdigest(),
                        'pose_map':{'x':pose[0],'y':pose[1],'yaw':pose[2]},
                        'visited':len(self.visited),'total':len(self.samples),
                        'map_frame':'map','robot_frame':'base_footprint',
                        'saved_at':datetime.now().astimezone().isoformat()}
                    if manifest['visited'] != manifest['total']:
                        raise RuntimeError('Coverage incomplete; handoff forbidden')
                    dst=Path(handoff);dst.parent.mkdir(parents=True,exist_ok=True)
                    tmp=dst.with_suffix('.tmp')
                    tmp.write_text(json.dumps(manifest,indent=2),encoding='utf-8')
                    os.replace(tmp,dst)
                    self.get_logger().info('HANDOFF_READY '+str(dst))
                self.get_logger().info('AUTONOMOUS_MAP_SAVED '+str(yaml_path))
                self.save_done=True
            except Exception as exc:
                self.failed=True
                self.get_logger().error('HANDOFF_FAILED: '+str(exc))
            finally:
                self.stop()
        threading.Thread(target=worker,daemon=True).start()

    def choose(self):
        self.last_plan=time.monotonic()
        result=select_target(self.grid,self.valid,self.pose[:2],self.samples,
                             self.visited,self.rejected)
        if result is None:
            if not self.driving:
                self.get_logger().info('PREVIEW: no reachable uncovered road sample; no motion')
                return
            if len(self.visited)==len(self.samples):
                self.stop()
                if not bool(self.p('return_home')):
                    self.failed=True
                    self.get_logger().error('HOLD: return_home must be true for safe handoff')
                else:
                    self.returning=True
                    self.target=None;self.path=[]
                    self.get_logger().info('ROAD_COVERAGE_COMPLETE; returning to START for AMCL handoff')
            else:
                self.failed=True
                self.stop()
                missing=len(self.samples)-len(self.visited)
                self.get_logger().error(
                    f'COVERAGE_INCOMPLETE: {missing} road samples unvisited/unreachable; '
                    'no map-to-navigation handoff permitted')
            return
        self.target,self.path=result
        self.last_plan=time.monotonic()
        target=self.samples[self.target]
        self.get_logger().info(f'TARGET {self.target+1}/{len(self.samples)} '
                               f'({target[0]:.2f},{target[1]:.2f}) '
                               f'path_cells={len(self.path)} '
                               f'visited={len(self.visited)}/{len(self.samples)}')

    def tick(self):
        if self.save_done:
            self.stop()
            if rclpy.ok():rclpy.shutdown()
            return
        if self.failed:
            self.stop()
            raise RuntimeError('COVERAGE_ABORTED: missing safety condition, unreachable targets, or map save failure')
        if self.finished:
            self.stop()
            return
        ok,why=self.fresh()
        if not ok:
            self.stop();self.warn('HOLD: '+why)
            return
        self.pose=self.read_pose()
        if self.pose is None:
            self.stop();return
        # Map origin and graph anchor must correspond to the current START pose.
        if not self.anchor_verified:
            if math.hypot(self.pose[0],self.pose[1])>.20:
                self.stop();self.warn('HOLD: SIM road-graph START anchor mismatch (>0.20m)')
                return
            self.anchor_verified=True
        self.update_visited()
        g=self.grid
        # Do not enter unknown, nonroad, inflated obstacle or map edge.
        current=closest_cell(g,self.valid,self.pose[:2],radius=.15)
        if current is None:
            self.stop()
            self.warn('HOLD: robot outside safe mapped road corridor')
            return
        dist=self.safety_distance()
        if dist is None or dist<float(self.p('emergency_stop')):
            self.stop();self.warn(f'HOLD: forward safety laser distance={dist}')
            return
        if not self.driving:
            # Planning preview is motionless by design.
            if self.target is None or time.monotonic()-self.last_plan>4:
                self.choose()
            # Report success only after sensor, TF, map and A* path checks.
            if self.target is not None and self.path and not self.preview_reported:
                dest=str(self.p('preview_status_path'))
                if dest:
                    report={'schema':'aic_coverage_preview_v1','ready':True,
                            'samples':len(self.samples),'visited':len(self.visited),
                            'target_index':self.target,'path_cells':len(self.path)}
                    path=Path(dest)
                    path.parent.mkdir(parents=True,exist_ok=True)
                    tmp=path.with_suffix('.tmp')
                    tmp.write_text(json.dumps(report),encoding='utf-8')
                    os.replace(tmp,path)
                self.get_logger().info(
                    f'PREVIEW_PLAN_READY: target={self.target+1}/{len(self.samples)} '
                    f'path_cells={len(self.path)} visited={len(self.visited)}')
                self.preview_reported=True
            if time.monotonic()-self.last_report>5:
                self.get_logger().info('PREVIEW ONLY: no motion requested')
                self.last_report=time.monotonic()
            self.stop();return
        if not self.returning and self.target is not None and self.target in self.visited:
            self.target=None;self.path=[]
        if not self.returning and self.target is None:
            self.choose()
            if self.target is None and not self.returning:return
        target=(0.,0.) if self.returning else self.samples[self.target]
        # For patrol handoff, finish at the ORIGINAL START pose, including yaw.
        if self.returning and math.dist(self.pose[:2],target)<.065:
            err=wrap(-self.pose[2])
            if abs(err)<.10:
                self.finished=True
                self.stop()
                self.get_logger().info('COVERAGE_COMPLETE_AND_HOME: saving map for Nav2')
                self.save()
                return
            cmd=Twist()
            cmd.angular.z=max(-.20,min(.20,1.1*err))
            self.pub.publish(cmd)
            return
        # Updating the live map may invalidate a path; replan repeatedly.
        if not self.path or time.monotonic()-self.last_plan>1.5:
            goal=closest_cell(g,self.valid,target,.17 if self.returning else .11)
            replanned=astar(g,self.valid,current,goal) if goal is not None else None
            if replanned is None:
                if self.returning:
                    self.failed=True;self.stop()
                    self.get_logger().error('RETURN_HOME_BLOCKED: STOP; no Nav2 handoff')
                    return
                self.get_logger().warning(f'Target {self.target} now unreachable: skip, report at end')
                self.rejected.add(self.target)
                self.target=None;self.path=[]
                self.stop();return
            self.path=replanned
            self.last_plan=time.monotonic()
        # Look ahead along connected safe cells, reducing grid-following jitter.
        look=self.pose[:2]
        for c in self.path:
            p=g.to_world(c%g.w,c//g.w)
            if math.dist(self.pose[:2],p)>.13:
                look=p;break
            look=p
        dx,dy=look[0]-self.pose[0],look[1]-self.pose[1]
        angle=wrap(math.atan2(dy,dx)-self.pose[2])
        cmd=Twist()
        if abs(angle)>.18:
            cmd.angular.z=max(-float(self.p('angular_speed')),
                              min(float(self.p('angular_speed')),1.7*angle))
        else:
            cmd.linear.x=min(float(self.p('linear_speed')),
                             max(.035, .045+ .10*math.dist(self.pose[:2],target)))
            cmd.angular.z=max(-.12,min(.12, .9*angle))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node=None
    try:
        node=AutonomousCoverage()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if rclpy.ok():
                for _ in range(3):
                    node.stop();time.sleep(.05)
            node.destroy_node()
        if rclpy.ok():rclpy.shutdown()


if __name__=='__main__':
    main()
