#!/usr/bin/env python3
"""AIC ROS2 Jazzy: one continuous 17-point inspection mission.

Executes every original official waypoint (as either a stop or a Nav2
NavigateThroughPoses via) and two upstream signal staging points. Uses
camera-pixel GREEN decisions; never simulates a green signal. Requires a
single Gazebo server, an actual patrol_bot, healthy sensor + TF graph and
usable footprint map clearance. Runs without SLAM (map_server + AMCL).

'patrol' performs the complete task sequence only when all gates are actually
seen and navigation succeeds; there is no unsafe bypass switch.
"""
import argparse, json, math, subprocess, sys, time
from pathlib import Path
from aic_v5_route import load_points, planned_stops, centerline_error, normalize
from aic_v5_audit import load_map, clearance, report, segment_check, lateral_margin_check

STOPS_AT={'A_LEFT_VIEW','A_BOTTOM_B_TOP_VIEW','B_RIGHT_VIEW','PLATE_1','PLATE_2','PLATE_3'}
# Camera is mounted facing robot +X. Route point yaw often looks along the
# road, but the plates are EAST of the robot. Aim at the target before capture.
# Angles below are in *world* coordinates, transformed to map via START yaw.
PHOTO_WORLD_HEADINGS={
    'A_LEFT_VIEW': (0.0,),            # A boards lie east of west sidewalk
    'A_BOTTOM_B_TOP_VIEW': (math.pi/2,-math.pi/2), # A north, B south
    'B_RIGHT_VIEW': (math.pi,),       # B boards lie to the west
    'PLATE_1': (0.0,), 'PLATE_2': (0.0,), 'PLATE_3': (0.0,),
}

# Pass-through points are actual official points, not fictitious substitutes.
GATE_SPANS={
    'top':('TOP_STAGING',('TOP_LIGHT_WAIT','TOP_AFTER_LIGHT')),
    'mid':('MID_STAGING',('B_RIGHT_VIEW','MID_LIGHT_WAIT','BOTTOM_TURN')),
}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode', choices=['preview','top-stage','patrol'],default='preview')
    ap.add_argument('--ack-official-crossing',action='store_true')
    ap.add_argument('--leg-timeout',type=float,default=180,help='Per-leg SIMULATION seconds (not wall seconds)')
    ap.add_argument('--green-timeout',type=float,default=90,help='Traffic wait SIMULATION seconds (not wall seconds)')
    ap.add_argument('--centerline-max',type=float,default=.10,
                    help='Emergency deviation stop, not proof of painted-line compliance')
    args=ap.parse_args()
    if args.leg_timeout<30 or args.green_timeout<20 or not .10<=args.centerline_max<=.18:
        ap.error('Unsupported timeout or centerline-max')
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('smart_community_gazebo'))
    route=share/'config/official_route.yaml';map_yaml=share/'maps/community.yaml'
    raw=load_points(route);stops=planned_stops(raw)
    expected={p['name'] for p in raw}
    covered={p['name'] for p in stops}|{'START'}
    if not expected.issubset(covered):
        raise RuntimeError('Original route points missing from execution: '+str(expected-covered))
    grid=load_map(map_yaml)
    print(f'[V5] {len(raw)} ORIGINAL official checkpoints; {len(stops)} navigation targets (incl 2 staging points)')
    for i,p in enumerate(stops):print(f"  {i:02d} {p['name']:<26} ({p['x']:+.3f},{p['y']:+.3f},{p['yaw']:+.3f})")
    blocked=report(route,map_yaml)
    if args.mode=='preview':
        print('V5_PREVIEW_OK: no motion; footprint grid audit, not a driving/traffic compliance certificate')
        return 0
    if blocked:
        raise RuntimeError('V5_PREFLIGHT_BLOCKED: oriented footprint overlaps occupied/unknown map cells')
    if not args.ack_official_crossing:
        ap.error('Official START line crossing remains ambiguous: supervision requires --ack-official-crossing')
    servers=subprocess.run(['pgrep','-af','^gz sim server'],capture_output=True,text=True,timeout=4)
    n=len([x for x in servers.stdout.splitlines() if x.strip()])
    if n!=1:raise RuntimeError('Need exactly 1 Gazebo server; found '+str(n))
    models=subprocess.run(['gz','model','--list'],capture_output=True,text=True,timeout=10)
    if models.returncode or not any(x.strip()=='- patrol_bot' for x in models.stdout.splitlines()):
        raise RuntimeError('Gazebo patrol_bot is absent; no motion')

    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Image,LaserScan
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseStamped
    from lifecycle_msgs.srv import GetState
    from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
    from tf2_ros import Buffer, TransformListener

    rclpy.init()
    nav=BasicNavigator(node_name='aic_v5_full_patrol')
    nav.set_parameters([Parameter('use_sim_time',Parameter.Type.BOOL,True)])
    tf_buf=Buffer();tf_listener=TransformListener(tf_buf,nav,spin_thread=False)
    stamp=time.strftime('%Y%m%d_%H%M%S')
    out=Path.home()/'aic_ws'/'aic_results'/('v5_patrol_'+stamp)
    out.mkdir(parents=True,exist_ok=True)
    flog=(out/'events.jsonl').open('w',encoding='utf-8')
    latest={'laser_wall':0,'laser_ok':False,'odom_wall':0,'camera_wall':0,
            'signal':None,'signal_wall':0,'capture':None,'capture_wall':0}
    moving=False
    def log(tag,**k):
        obj={'date':time.strftime('%Y-%m-%dT%H:%M:%S'),'event':tag,**k}
        print('[V5]',json.dumps(obj,ensure_ascii=False),flush=True)
        flog.write(json.dumps(obj,ensure_ascii=False)+'\n');flog.flush()
    def spin(t=.1):rclpy.spin_once(nav,timeout_sec=t)
    def sim_seconds():return nav.get_clock().now().nanoseconds*1e-9
    def cb_scan(m):
        vs=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        latest['laser_wall']=time.monotonic()
        latest['laser_ok']=len(vs)>=100 and max(vs)>.5 and max(vs)-min(vs)>.2
    def cb_odom(m):latest['odom_wall']=time.monotonic()
    def cb_camera(m):latest['camera_wall']=time.monotonic()
    def cb_signal(m):
        try:
            val=json.loads(m.data)
            if val.get('source')=='camera' and val.get('gate') in ('top','mid'):
                latest['signal']=val;latest['signal_wall']=time.monotonic()
        except (TypeError,ValueError):pass
    def cb_capture(m):
        try:latest['capture']=json.loads(m.data);latest['capture_wall']=time.monotonic()
        except (TypeError,ValueError):pass
    nav.create_subscription(LaserScan,'/scan_clean',cb_scan,qos_profile_sensor_data)
    nav.create_subscription(Odometry,'/odom_clean',cb_odom,qos_profile_sensor_data)
    nav.create_subscription(Image,'/camera/image_raw',cb_camera,qos_profile_sensor_data)
    nav.create_subscription(String,'/aic/traffic/observation',cb_signal,10)
    nav.create_subscription(String,'/aic/inspection/result',cb_capture,10)
    target_pub=nav.create_publisher(String,'/aic/traffic/target',10)
    snap_pub=nav.create_publisher(String,'/aic/inspection/request',10)
    def goal(p):
        m=PoseStamped();m.header.frame_id='map';m.header.stamp=nav.get_clock().now().to_msg()
        m.pose.position.x=float(p['x']);m.pose.position.y=float(p['y'])
        m.pose.orientation.z=math.sin(p['yaw']/2);m.pose.orientation.w=math.cos(p['yaw']/2)
        return m
    def pos():
        tf=tf_buf.lookup_transform('map','base_footprint',Time())
        p=tf.transform.translation;q=tf.transform.rotation
        yaw=math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        return p.x,p.y,yaw
    def abort(why):
        nonlocal moving
        if moving:
            try:nav.cancelTask()
            except Exception:pass
            moving=False
        log('SAFETY_ABORT',reason=why)
        raise RuntimeError(why)
    def health(camera=False):
        t=time.monotonic()
        if not latest['laser_ok'] or t-latest['laser_wall']>3 or t-latest['odom_wall']>3:
            abort('Laser/odometry stale or laser scans invalid')
        if camera and (not latest['camera_wall'] or t-latest['camera_wall']>3):
            abort('Real camera stream missing/stale')
        if camera:
            observers=nav.get_publishers_info_by_topic('/aic/traffic/observation')
            if len(observers)!=1 or observers[0].node_name!='aic_v5_camera_perception':
                abort('Expected exactly one real-camera v5 traffic classifier; check duplicate vision nodes')
        pubs=nav.get_publishers_info_by_topic('/cmd_vel')
        if len(pubs)!=1 or pubs[0].node_name!='collision_monitor':
            abort('Final /cmd_vel must only be published by collision_monitor')
        if nav.get_clock().now().nanoseconds==0:abort('Missing simulation clock')
    def camera_look_and_inspect(checkpoint, current):
        # Rotations are genuine Nav2 pose goals, not fake camera pan commands.
        # Return to original travel heading after capture for next leg.
        for index,world_heading in enumerate(PHOTO_WORLD_HEADINGS[checkpoint['name']]):
            camera_pose=dict(checkpoint)
            camera_pose['yaw']=normalize(world_heading-float(raw[0]['yaw']))
            camera_pose['name']=checkpoint['name']+'_LOOK_'+str(index+1)
            current=travel([camera_pose],current)
            label=checkpoint['name'] if len(PHOTO_WORLD_HEADINGS[checkpoint['name']])==1 else checkpoint['name']+('_A' if index==0 else '_B')
            inspect(label)
        return travel([checkpoint],current)

    def inspect(label):
        # Non-blocking machine-vision event: result must be real. Missing detector
        # remains UNKNOWN in the output log; never report a guessed detection.
        m=String();m.data=label
        latest['capture']=None;start=time.monotonic()
        for _ in range(3):snap_pub.publish(m);spin(.08)
        while time.monotonic()-start<5:
            spin(.1)
            result=latest['capture']
            if result and result.get('name')==label and latest['capture_wall']>=start:
                log('INSPECTION',checkpoint=label,result=result);return
        log('INSPECTION_UNAVAILABLE',checkpoint=label)
    def send_gate(name):
        m=String();m.data=name;target_pub.publish(m)
    def wait_green(gate):
        # Wait for a full signal cycle: observe RED/YELLOW after targeting, then
        # observe a new stable GREEN. Avoid entering on the final second of a
        # green already in progress. Camera-only input (not SDF/controller topic).
        log('TRAFFIC_WAIT',gate=gate,require='red/yellow then NEW GREEN')
        latest['signal']=None;latest['signal_wall']=0
        non_green=False;last_ts=0;greens=0
        started_sim=sim_seconds();started_wall=time.monotonic()
        while sim_seconds()-started_sim<args.green_timeout and time.monotonic()-started_wall<1800:
            send_gate(gate);spin(.12);health(camera=True)
            val=latest['signal'];wall=latest['signal_wall']
            if not val or wall==last_ts or time.monotonic()-wall>1.2:continue
            last_ts=wall
            if val.get('gate')!=gate:continue
            state=val.get('state');conf=float(val.get('confidence') or 0)
            if state in ('RED','YELLOW') and conf>=.7:
                non_green=True;greens=0
            elif non_green and state=='GREEN' and conf>=.7:
                greens+=1
                if greens>=4:
                    log('TRAFFIC_GREEN_FRESH',gate=gate,samples=greens)
                    return
            else:greens=0
        abort(f'Traffic signal {gate}: no new, stable camera GREEN within {args.green_timeout}s (ROI calibration may be missing)')
    def track(points):
        # Calculate the distance to closest planned leg, as the controller may
        # blend multiple through-poses at junctions.
        a=points[0];d=float('inf');segment=None
        x,y,yaw=pos()
        for b in points[1:]:
            candidate=centerline_error(x,y,a,b)
            if candidate<d:d=candidate;segment=(a['name'],b['name'])
            a=b
        return d,(x,y,yaw),segment
    def travel(seq,current,gate=None):
        nonlocal moving
        goals=[current]+seq
        for a,b in zip(goals,goals[1:]):
            counts,bad=segment_check(grid,a,b)
            _,side_bad=lateral_margin_check(grid,a,b,margin=.04)
            if bad or side_bad:
                abort(f'Preflight segment {a["name"]}->{b["name"]} grid collision/margin: {(bad+side_bad)[0]}')
        health(camera=args.mode=='patrol')
        log('NAV_START',from_point=current['name'],goals=[p['name'] for p in seq],gate=gate)
        if len(seq)==1:nav.goToPose(goal(seq[0]))
        else:nav.goThroughPoses([goal(p) for p in seq])
        moving=True;start_sim=sim_seconds();start_wall=time.monotonic()
        # For the first safe distance after GREEN staging, red/yellow before
        # actual entry triggers stop. After commitment do NOT abruptly brake in
        # the crosswalk; check the traffic gate before entry instead.
        gate_limit=.12 if gate else 0
        while not nav.isTaskComplete():
            spin(.04);health(camera=args.mode=='patrol')
            if sim_seconds()-start_sim>args.leg_timeout or time.monotonic()-start_wall>1800:
                abort('Nav2 leg timeout (sim clock or wall limit)')
            try:
                d,(x,y,yaw),segment=track(goals)
            except Exception as e:abort('Lost TF: '+str(e))
            # Narrow lamp-post corridor: 5cm lateral error already collides
            # with an occupied pixel in the user's ORIGINAL map.
            narrow=segment and segment[1]=='BOTTOM_TURN' and segment[0] in ('MID_LIGHT_WAIT','MID_STAGING','B_RIGHT_VIEW')
            bound=min(args.centerline_max,.035) if narrow else args.centerline_max
            if d>bound:
                abort(f'Centerline deviation {d:.3f}m (limit {bound:.3f}, segment={segment}); safety stop')
            # Runtime footprint occupancy prevents passing known obstacles; Nav2
            # dynamic collision monitor provides an independent laser guard.
            cl=clearance(grid,x,y,yaw)
            if cl!='FREE':abort(f'Oriented robot footprint intersects {cl} grid at {(round(x,3),round(y,3))}')
            if gate:
                traveled=math.hypot(x-current['x'],y-current['y'])
                if traveled<gate_limit:
                    v=latest['signal']
                    if not (v and v.get('gate')==gate and v.get('state')=='GREEN'
                            and v.get('confidence',0)>=.7
                            and time.monotonic()-latest['signal_wall']<1.2):
                        abort(f'Traffic {gate} lost fresh green before commitment')
        moving=False
        result=nav.getResult()
        if result!=TaskResult.SUCCEEDED:abort('Nav2 failed: '+str(result))
        spin(.15)
        x,y,yaw=pos();end=seq[-1]
        err=math.hypot(x-end['x'],y-end['y'])
        log('NAV_DONE',to_point=end['name'],actual=[x,y,yaw],goal_error=round(err,3))
        if err>.17:abort(f'Nav2 returned success but final error {err:.3f}m')
        return end
    try:
        t=time.monotonic()
        while nav.get_clock().now().nanoseconds==0:
            spin(.1)
            if time.monotonic()-t>30:abort('No /clock')
        for nm in ('map_server','amcl','planner_server','controller_server',
                   'bt_navigator','velocity_smoother','collision_monitor'):
            c=nav.create_client(GetState,'/'+nm+'/get_state')
            if not c.wait_for_service(timeout_sec=5):abort('Missing lifecycle service '+nm)
            fut=c.call_async(GetState.Request());rclpy.spin_until_future_complete(nav,fut,timeout_sec=5)
            if not fut.done() or not fut.result() or fut.result().current_state.label!='active':
                abort('Lifecycle not active: '+nm)
            nav.destroy_client(c)
        t=time.monotonic()
        while latest['laser_wall']==0 or latest['odom_wall']==0:
            spin(.1)
            if time.monotonic()-t>20:abort('No scan/odom input')
        health(camera=args.mode=='patrol')
        t=time.monotonic()
        while True:
            spin(.1)
            try:x,y,yaw=pos();break
            except Exception:
                if time.monotonic()-t>20:abort('No map->base_footprint TF')
        if math.hypot(x,y)>.12 or abs(normalize(yaw))>.25:
            abort(f'Robot not at map START {(x,y,yaw)}; restart cleanly, no AMCL reset on moving robot')
        log('READY',start=[x,y,yaw],out=str(out))
        prev={'name':'START','x':x,'y':y,'yaw':yaw}
        name_index={p['name']:i for i,p in enumerate(stops)}
        i=0
        while i<len(stops):
            p=stops[i]
            if p['name']=='TOP_STAGING' or p['name']=='MID_STAGING':
                prev=travel([p],prev);i+=1
                if args.mode=='top-stage':
                    log('TOP_STAGE_COMPLETE',notes='Camera gate not yet entered')
                    print('V5_TOP_STAGE_OK');return 0
                gate='top' if p['name']=='TOP_STAGING' else 'mid'
                if gate=='mid':
                    # B_RIGHT_VIEW is a pass-through checkpoint inside the gated
                    # transit. Take the west-looking picture at MID_STAGING,
                    # just 10cm upstream, while still before the crosswalk.
                    # Do not pretend a snapshot was taken at the via point.
                    log('INSPECTION_PROXY',checkpoint='B_RIGHT_VIEW',actual_location='MID_STAGING')
                    photo_at_stage=dict(prev,name='B_RIGHT_VIEW')
                    prev=camera_look_and_inspect(photo_at_stage,prev)
                wait_green(gate)
                expected_names=GATE_SPANS[gate][1]
                via=stops[i:i+len(expected_names)]
                if [p['name'] for p in via]!=list(expected_names):
                    abort('Route gate checkpoints mismatch '+gate)
                prev=travel(via,prev,gate=gate)
                for checkpoint in via:
                    log('CHECKPOINT_PASSED',name=checkpoint['name'],via=True)
                i+=len(via)
                continue
            prev=travel([p],prev)
            if p['name'] in STOPS_AT:prev=camera_look_and_inspect(p,prev)
            i+=1
        log('FULL_PATROL_NAV_FINISHED',official_points=len(raw),
            disclaimer='No claim of lane, traffic, vision accuracy or dynamic collision certification')
        print('V5_FULL_ROUTE_NAV_FINISHED: Nav2 reached FINISH; inspect log, video and recognition outputs')
        return 0
    except KeyboardInterrupt:
        if moving:
            try:nav.cancelTask()
            except Exception:pass
        log('INTERRUPTED_BY_OPERATOR')
        return 130
    except Exception as exc:
        if moving:
            try:nav.cancelTask()
            except Exception:pass
        log('PATROL_FAILED',reason=str(exc));return 4
    finally:
        flog.close()
        nav.destroyNode()
        if rclpy.ok():rclpy.shutdown()

if __name__=='__main__':sys.exit(main())
