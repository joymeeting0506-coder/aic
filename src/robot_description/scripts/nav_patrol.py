#!/usr/bin/env python3
"""AIC Nav2 v3: FIRST-LEG-ONLY safety tests. NO competitive full patrol.

Modes:
  preview : offline listing, NEVER moves
  smoke   : travel 0.65m from START, crossing the official-diagram parking stripe
  stage   : travel 1.04m from START, stop before top zebra/traffic light (NOT
            a verified stop line). Stage requires a fresh spawn at START.
  full    : HARD BLOCKED until camera-driven 2-light interlock exists
"""
import argparse
import math
import os
import time
import yaml

from map_segment_gate import screen_first_leg
from lane_geometry_preflight import inspect as screen_lanes


def world_to_map(route):
    start=route[0]
    x0,y0,yaw0=[float(start[k]) for k in ('x','y','yaw')]
    c=math.cos(yaw0);s=math.sin(yaw0)
    out=[]
    for p in route:
        dx=float(p['x'])-x0;dy=float(p['y'])-y0
        yaw=float(p['yaw'])-yaw0
        yaw=math.atan2(math.sin(yaw),math.cos(yaw))
        out.append({'name':p['name'],'x':c*dx+s*dy,
                    'y':-s*dx+c*dy,'yaw':yaw,'action':p.get('action','')})
    return out


def load_route(route_file):
    raw=yaml.safe_load(open(route_file,encoding='utf-8'))['route']
    assert len(raw) in (17,22) and raw[0]['name']=='START', 'Unexpected official route'
    return raw,world_to_map(raw)


def preflight(route_file,map_file,world_file,mode,distance):
    if not screen_first_leg(route_file,map_file,distance,radius=.24):
        raise RuntimeError('BLOCKED: saved-map static occupancy screen failed')
    if not screen_lanes(route_file,world_file,mode=mode,
                        distance=distance,radius=.24):
        raise RuntimeError('BLOCKED: other continuous visual stripes intersect route')


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode',choices=['preview','smoke','stage','full'],default='preview')
    ap.add_argument('--ack-official-crossing',action='store_true',
                    help='Confirm supervised test of START crossing shown by organizer diagram; '
                         'this is NOT a ruling that crossing is penalty-free')
    ap.add_argument('--timeout',type=float,default=90.)
    args=ap.parse_args()
    if args.mode=='full':
        ap.error('FULL BLOCKED: no camera traffic-gate, verified stop lines, '
                 'lane keepout or obstacle-avoidance track record. No override.')
    # Delay ROS imports, so preview/offline checks do not require running nodes.
    from ament_index_python.packages import get_package_share_directory
    share=get_package_share_directory('smart_community_gazebo')
    route_path=os.path.join(share,'config','official_route.yaml')
    map_path=os.path.join(share,'maps','community.yaml')
    world_path=os.path.join(share,'worlds','smart_community_v2.sdf')
    raw,route=load_route(route_path)
    print(f'[V3] Inspection route {len(raw)} points / world-to-map anchored at START')
    for i,p in enumerate(route):
        print(f"  {i:02d} {p['name']}: map=({p['x']:+.3f},{p['y']:+.3f},"
              f"{p['yaw']:+.3f}) action={p['action']}")
    if args.mode=='preview':
        print('PREVIEW_ONLY; no ROS motion; no competition compliance certified')
        return
    if not args.ack_official_crossing:
        ap.error('This supervised first leg crosses the continuous parking stripe '
                 'shown in the OFFICIAL diagram. Its rule status is ambiguous. '
                 'After visually checking the track use --ack-official-crossing.')
    distance=.65 if args.mode=='smoke' else 1.04
    # Test cannot go beyond FIRST_LIGHT_WAIT or cross top zebra stripes.
    length=math.hypot(route[1]['x'],route[1]['y'])
    if distance>length-.15:
        raise RuntimeError('Goal too near first traffic-light waiting marker')
    preflight(route_path,map_path,world_path,args.mode,distance)

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import qos_profile_sensor_data
    from geometry_msgs.msg import PoseStamped
    from sensor_msgs.msg import LaserScan
    from nav_msgs.msg import Odometry
    from lifecycle_msgs.srv import GetState
    from rclpy.time import Time
    from tf2_ros import Buffer,TransformListener
    from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult

    def make_goal(node,goal_dist):
        start,nextp=route[:2]
        t=goal_dist/length
        x=start['x']+t*(nextp['x']-start['x'])
        y=start['y']+t*(nextp['y']-start['y'])
        theta=math.atan2(nextp['y']-start['y'],nextp['x']-start['x'])
        goal=PoseStamped()
        goal.header.frame_id='map'
        goal.header.stamp=node.get_clock().now().to_msg()
        goal.pose.position.x=x;goal.pose.position.y=y
        goal.pose.orientation.z=math.sin(theta*.5)
        goal.pose.orientation.w=math.cos(theta*.5)
        return goal

    rclpy.init()
    nav=BasicNavigator(node_name='aic_v3_stage_nav')
    nav.set_parameters([Parameter('use_sim_time',Parameter.Type.BOOL,True)])
    last={'scan':None,'odom':None,'laser_ok':False}

    def on_scan(msg):
        valid=[float(x) for x in msg.ranges if math.isfinite(x) and
               msg.range_min<=x<=msg.range_max]
        if len(valid)>=100:
            last['laser_ok']=(max(valid)>.5 and max(valid)-min(valid)>.2)
        else:
            last['laser_ok']=False
        last['scan']=time.monotonic()

    def on_odom(_):
        last['odom']=time.monotonic()

    sub_scan=nav.create_subscription(LaserScan,'/scan_clean',on_scan,qos_profile_sensor_data)
    sub_odom=nav.create_subscription(Odometry,'/odom_clean',on_odom,qos_profile_sensor_data)
    tf_buffer=Buffer()
    tf_listener=TransformListener(tf_buffer,nav,spin_thread=False)
    goal_sent=False
    try:
        def spin(t=.10):
            rclpy.spin_once(nav,timeout_sec=t)

        begin=time.monotonic()
        while nav.get_clock().now().nanoseconds==0:
            spin()
            if time.monotonic()-begin>40:
                raise RuntimeError('No /clock after 40s')

        def lifecycle_active(name,timeout=40):
            cli=nav.create_client(GetState,f'/{name}/get_state')
            try:
                start=time.monotonic()
                while time.monotonic()-start<timeout:
                    if not cli.wait_for_service(timeout_sec=.4):
                        continue
                    fut=cli.call_async(GetState.Request())
                    rclpy.spin_until_future_complete(nav,fut,timeout_sec=1.5)
                    if fut.done() and fut.result() and fut.result().current_state.label=='active':
                        return
                raise RuntimeError(f'Lifecycle node not active: {name}')
            finally:
                nav.destroy_client(cli)

        for name in ('map_server','amcl','planner_server','controller_server',
                     'bt_navigator','velocity_smoother','collision_monitor'):
            lifecycle_active(name)
        print('[V3] Required lifecycle nodes active')
        begin=time.monotonic()
        while (last['scan'] is None or last['odom'] is None or not last['laser_ok']):
            spin()
            if time.monotonic()-begin>15:
                raise RuntimeError('Sensor preflight timeout or bad /scan_clean')

        # Publisher graph should be collision_monitor -> /cmd_vel -> bridge.
        # No teleop / route_mapping publisher is permitted simultaneously.
        publishers=nav.get_publishers_info_by_topic('/cmd_vel')
        names=[getattr(n,'node_name','') for n in publishers]
        print(f'[V3] /cmd_vel publishers: {names}')
        if len(publishers)!=1 or names[0]!='collision_monitor':
            raise RuntimeError('BLOCKED: /cmd_vel must have one collision_monitor publisher; '
                               'stop teleop/mapping or inspect ROS topic remaps')

        def tf_pose():
            tf=tf_buffer.lookup_transform('map','base_footprint',Time())
            p=tf.transform.translation
            q=tf.transform.rotation
            yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
            return p.x,p.y,yaw

        begin=time.monotonic()
        while True:
            spin(.1)
            try:
                x,y,yaw=tf_pose()
                break
            except Exception:
                if time.monotonic()-begin>15:
                    raise RuntimeError('No map->base_footprint TF')
        if math.hypot(x,y)>.10 or abs(math.atan2(math.sin(yaw),math.cos(yaw)))>.20:
            raise RuntimeError(f'ROBOT_NOT_AT_START map=({x:.3f},{y:.3f},{yaw:.3f}); '
                               'do not reset AMCL pose while the robot is displaced')
        print(f'[V3] START TF OK, sending {args.mode} {distance:.2f}m goal')
        nav.goToPose(make_goal(nav,distance))
        goal_sent=True
        start_time=time.monotonic()
        while not nav.isTaskComplete():
            spin(.05)
            now=time.monotonic()
            if (now-start_time>args.timeout or last['scan'] is None or
                last['odom'] is None or not last['laser_ok'] or
                now-last['scan']>2.0 or now-last['odom']>2.0):
                nav.cancelTask()
                raise RuntimeError('SAFETY_ABORT timeout/stale sensor/bad LiDAR')
            # This is a runtime centerline watchdog, NOT a certified vehicle
            # footprint/painted lane constraint. Cancelling has latency.
            try:
                x,y,_=tf_pose()
            except Exception:
                nav.cancelTask()
                raise RuntimeError('SAFETY_ABORT lost TF')
            if x < -.10 or x>distance+.16 or abs(y)>.10:
                nav.cancelTask()
                raise RuntimeError(f'SAFETY_ABORT first-leg deviation map=({x:.3f},{y:.3f})')
            graph=nav.get_publishers_info_by_topic('/cmd_vel')
            if len(graph)!=1 or getattr(graph[0],'node_name','')!='collision_monitor':
                nav.cancelTask()
                raise RuntimeError('SAFETY_ABORT /cmd_vel publisher conflict')
        outcome=nav.getResult()
        if outcome!=TaskResult.SUCCEEDED:
            raise RuntimeError(f'Nav2 result={outcome} (not succeeded)')
        spin(.25)
        x,y,_=tf_pose()
        if abs(x-distance)>.15 or abs(y)>.10:
            raise RuntimeError(f'Nav2 reported success but stopped off target: '
                               f'({x:.3f},{y:.3f}); target x={distance:.3f}')
        print(f'NAV3_{args.mode.upper()}_TEST_SUCCESS map=({x:.3f},{y:.3f}); '
              'NOT_COMPETITION_CERTIFIED')
    except BaseException:
        if goal_sent:
            try:
                nav.cancelTask()
            except Exception:
                pass
        raise
    finally:
        # Do not inject /cmd_vel from a SECOND publisher at shutdown.
        nav.destroyNode()
        if rclpy.ok():
            rclpy.shutdown()


if __name__=='__main__':
    main()
