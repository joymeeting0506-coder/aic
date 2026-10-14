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
import xml.etree.ElementTree as ET
from pathlib import Path
from aic_v5_route import load_points, planned_stops, centerline_error, normalize
from aic_v5_audit import load_map, clearance, report, segment_check, lateral_margin_check, poly_at, overlapping

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

# AIC_V5_5_GUARDED_LANE_CORNER_TURN
# AIC_LANE_FOLLOW_V5_4: special lane-constrained straight-road legs only.
# NO automatic replanning inside these legs: unexpected obstacles -> stop/fail,
# do not steer around painted markings. Other legs still use Nav2 planner.
LANE_FOLLOW_LEGS = {('LEFT_CORNER','A_LEFT_VIEW'),
                    ('A_LEFT_VIEW','MIDDLE_LEFT'),
                    ('BOTTOM_TURN','PARKING_ENTRY')}


# AIC_V56_UNIFIED_LANE_FOLLOW_20261013
# The START/FINISH line crosses a modelled parking-lot paint marker. Keep
# those two boundary transitions on the original guarded Nav2 pose controller;
# do NOT conceal the crossing with a fake lane-compliance pass.
PAINT_BOUNDARY_TRANSITIONS = {('START','TOP_STAGING'), ('RIGHT_TOP','FINISH')}
UNIFIED_MIN_LENGTH_M = .30

def use_unified_follow(a, seq):
    """Select the SAME centerline controller for every sufficiently long,
    straight road. Multi-poses for GREEN crossings must remain collinear.
    Short waypoints and in-place camera yaws retain original pose navigation.
    """
    if not seq or (a['name'],seq[-1]['name']) in PAINT_BOUNDARY_TRANSITIONS:
        return False
    b=seq[-1]
    dx=float(b['x'])-float(a['x']);dy=float(b['y'])-float(a['y'])
    length=math.hypot(dx,dy)
    if length < UNIFIED_MIN_LENGTH_M:
        return False
    for p in seq:
        # All points in one FollowPath must lie on the same directed line.
        if centerline_error(p['x'],p['y'],a,b)>.012:
            return False
        t=((p['x']-a['x'])*dx+(p['y']-a['y'])*dy)/(length*length)
        if t < -.001 or t > 1.001:
            return False
    return True

GATE_SPANS={
    'top':('TOP_STAGING',('TOP_LIGHT_WAIT','TOP_AFTER_LIGHT')),
    'mid':('MID_STAGING',('B_RIGHT_VIEW','MID_LIGHT_WAIT','BOTTOM_TURN')),
}


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--mode', choices=['preview','top-stage','patrol'],default='preview')
    ap.add_argument('--map-yaml', default='', help='Use the freshly saved SLAM map in both preview and patrol')
    ap.add_argument('--ack-official-crossing',action='store_true')
    ap.add_argument('--resume-top-stage', action='store_true',
                    help='Guarded recovery only: robot physically at TOP_STAGING; first leg omitted')
    ap.add_argument('--leg-timeout',type=float,default=180,help='Per-leg SIMULATION seconds (not wall seconds)')
    ap.add_argument('--green-timeout',type=float,default=90,help='Traffic wait SIMULATION seconds (not wall seconds)')
    ap.add_argument('--centerline-max',type=float,default=.10,
                    help='Emergency deviation stop, not proof of painted-line compliance')
    args=ap.parse_args()
    if args.resume_top_stage and args.mode!='patrol':
        ap.error('--resume-top-stage requires --mode patrol')
    if args.leg_timeout<30 or args.green_timeout<20 or not .10<=args.centerline_max<=.18:
        ap.error('Unsupported timeout or centerline-max')
    from ament_index_python.packages import get_package_share_directory
    share=Path(get_package_share_directory('smart_community_gazebo'))
    route=share/'config/official_route.yaml';map_yaml=(Path(args.map_yaml).expanduser().resolve() if args.map_yaml else share/'maps/community.yaml')
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
        previous={'name':'START','x':0.,'y':0.,'yaw':0.}
        counts={'CENTERLINE_FOLLOW':0,'SHORT_OR_BOUNDARY_NAV2':0}
        for target in stops:
            mode='CENTERLINE_FOLLOW' if use_unified_follow(previous,[target]) else 'SHORT_OR_BOUNDARY_NAV2'
            counts[mode]+=1
            print('UNIFIED_PLAN',previous['name'],'->',target['name'],mode)
            previous=target
        print('UNIFIED_PREVIEW_PASS',counts, 'traffic groups are separately checked in patrol')
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
    from nav_msgs.msg import Odometry, Path as NavPath
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
    latest={'motion_linear':None, 'motion_angular':None,
            'laser_wall':0,'laser_ok':False,'odom_wall':0,'camera_wall':0,
            'signal':None,'signal_wall':0,'capture':None,'capture_wall':0}
    moving=False
    def log(tag,**k):
        obj={'date':time.strftime('%Y-%m-%dT%H:%M:%S'),'event':tag,**k}
        print('[V5]',json.dumps(obj,ensure_ascii=False),flush=True)
        flog.write(json.dumps(obj,ensure_ascii=False)+'\n');flog.flush()
    def spin(t=.1):rclpy.spin_once(nav,timeout_sec=t)
    def sim_seconds():return nav.get_clock().now().nanoseconds*1e-9
    def cb_scan(m):
        # +inf means a normal no-hit beam; reject corruption, not open space.
        vs=[v for v in m.ranges if math.isfinite(v) and m.range_min<=v<=m.range_max]
        nohit=sum(1 for v in m.ranges if math.isinf(v) and v>0)
        total=len(m.ranges)
        latest['laser_wall']=time.monotonic()
        latest['laser_ok']=(total>=180 and m.range_max>m.range_min>=0
            and len(vs)>=max(60,math.ceil(total*.12))
            and len(vs)+nohit>=math.ceil(total*.95))
    def cb_odom(m):
        latest['odom_wall']=time.monotonic()
        t=m.twist.twist
        latest['motion_linear']=math.hypot(t.linear.x,t.linear.y)
        latest['motion_angular']=abs(t.angular.z)
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
        while time.monotonic()-start<45:
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
        non_green=False;last_ts=0;last_state=None;last_image_stamp=-1;greens=0
        started_sim=sim_seconds();started_wall=time.monotonic()
        while sim_seconds()-started_sim<args.green_timeout and time.monotonic()-started_wall<1800:
            send_gate(gate);spin(.12);health(camera=True)
            val=latest['signal'];wall=latest['signal_wall']
            if not val or wall==last_ts or time.monotonic()-wall>1.2:continue
            last_ts=wall
            if val.get('gate')!=gate:continue
            state=val.get('state');conf=float(val.get('confidence') or 0)
            image_stamp=val.get('sim_stamp_ns')
            # Require advancing CAMERA frame stamps. Wall-clock reception of a
            # cached green message must never count as fresh evidence.
            if not isinstance(image_stamp,int) or image_stamp<=last_image_stamp:
                continue
            last_image_stamp=image_stamp
            if state!=last_state:
                log('TRAFFIC_CAMERA_STATE',gate=gate,state=state,
                    score=round(conf,3),stamp_ns=image_stamp)
                last_state=state
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

    # AIC_LANE_FOLLOW_V5_4: read only Gazebo paint (visual-only, invisible to lidar).
    # Ignore zebra stripes for paint collision purposes; official crossings are
    # handled elsewhere by the real-camera traffic gates.
    world_sdf = Path.home()/'aic_ws/src/smart_community_gazebo/worlds/smart_community_v2.sdf'
    if not world_sdf.is_file():
        world_sdf=share/'worlds/smart_community_v2.sdf'
    lane_boxes=[]
    if args.mode=='patrol':
        root_sdf=ET.parse(world_sdf).getroot()
        world_node=root_sdf.find('world')
        if world_node is None or world_node.get('name')!='smart_community_semifinal_v8':
            raise RuntimeError('Lane-check SDF world mismatch; refusing patrol')
        if abs(abs(float(raw[0]['yaw']))-math.pi)>.001:
            raise RuntimeError('Lane-check expects START world yaw=pi')
        for mod in world_node.findall('model'):
            nm=mod.get('name','')
            if not nm.startswith(('line_outer_','h_','v_')):continue
            pose_txt=mod.findtext('pose')
            size_txt=mod.findtext('./link/visual/geometry/box/size')
            if not pose_txt or not size_txt:
                raise RuntimeError('Lane-check marking malformed '+nm)
            xx,yy,*_=map(float,pose_txt.split())
            sx,sy,*_=map(float,size_txt.split())
            # START yaw=pi: transform from world SDF into local map.
            xx=float(raw[0]['x'])-xx;yy=float(raw[0]['y'])-yy
            lane_boxes.append((nm,(xx-sx/2,yy-sy/2,xx+sx/2,yy+sy/2)))
        if len(lane_boxes)<15:
            raise RuntimeError('Lane-check missing paint markers; refusing patrol')

    def lane_paint_near(x,y,yaw,buffer=.015):
        """Conservative actual oriented body+wheel overlap with painted lines."""
        hull=poly_at(x,y,yaw)
        contacts=[]
        for nm,(x0,y0,x1,y1) in lane_boxes:
            rect=[(x0-buffer,y0-buffer),(x1+buffer,y0-buffer),
                  (x1+buffer,y1+buffer),(x0-buffer,y1+buffer)]
            if overlapping(hull,rect):contacts.append(nm)
        return contacts

    # AIC_V56_GUARDED_CENTERLINE_HANDOFF_20261013
    def centerline_follow_path(a,seq):
        """Dense FollowPath with a *guarded* join from an actual Nav2 stop.

        Nav2's goal checker accepts <=8cm XY errors. Prior code required
        <=4.5cm centerline deviation to start the NEXT leg, so a successful
        stop could deterministically fail here. Do NOT change the regular
        5.5cm lane-tracking limit. For a small safe initial offset, generate
        a smooth path that joins the nominal lane before the first via point;
        audit the full footprint against map and painted lines first.
        """
        b=seq[-1]
        x,y,yaw=pos()
        dx=float(b['x'])-float(a['x']);dy=float(b['y'])-float(a['y'])
        length=math.hypot(dx,dy)
        if not UNIFIED_MIN_LENGTH_M<=length<=3.0:
            abort('Unified FollowPath unexpected leg length')
        heading=math.atan2(dy,dx)
        if abs(normalize(yaw-heading))>.30:
            abort('Unified FollowPath start heading mismatch (>0.30 rad)')
        ux=dx/length;uy=dy/length;nx=-uy;ny=ux
        start_s=(x-a['x'])*ux+(y-a['y'])*uy
        signed_lat=(x-a['x'])*nx+(y-a['y'])*ny
        dline=abs(signed_lat)
        start_err=math.hypot(x-a['x'],y-a['y'])
        if dline>.085 or start_err>.11:
            abort(f'Unified FollowPath start outside guarded join (lateral={dline:.3f}, origin={start_err:.3f})')
        if lane_paint_near(x,y,yaw):
            abort('Unified FollowPath START paint <1.5cm')
        if clearance(grid,x,y,yaw)!='FREE':
            abort('Unified FollowPath START occupied/unknown')
        if start_s<-.11 or start_s>max(.16,.11):
            abort('Unified FollowPath unexpected longitudinal starting offset')

        traj=NavPath();traj.header.frame_id='map'
        traj.header.stamp=nav.get_clock().now().to_msg()
        def add(xx,yy,hh):
            po=PoseStamped();po.header=traj.header
            po.pose.position.x=float(xx);po.pose.position.y=float(yy)
            po.pose.orientation.z=math.sin(hh/2)
            po.pose.orientation.w=math.cos(hh/2)
            traj.poses.append(po)
        add(x,y,yaw)

        join=None
        first_s=max(0.,start_s)+.04
        # Only deviate from nominal centerline in a short, pre-audited JOIN.
        # Never teleport, fake a start pose, or disable the painted-line guard.
        if dline>.025:
            # In the middle gate B_RIGHT_VIEW is only 10cm after staging;
            # another signal via occurs 24cm after staging. A ~6cm starting
            # offset cannot be safely corrected in only 7cm. A near via may
            # be passed within 5cm while the join continues, but the join
            # MUST finish BEFORE the next more distant gate via.
            vias=sorted(((p['x']-a['x'])*ux+(p['y']-a['y'])*uy)
                          for p in seq)
            next_via=next((s for s in vias if s>start_s+.16),None)
            if next_via is None:
                abort('Unified FollowPath JOIN_REFUSED: no safe distant via point')
            end_s=min(start_s+max(.24,3.4*dline),next_via-.03)
            if end_s-start_s<.14 or end_s>=length-.03:
                abort('Unified FollowPath JOIN_REFUSED: insufficient safe distance before via/endpoint')
            span=end_s-start_s
            for via_s in vias:
                if start_s+.015<via_s<end_s:
                    q=(via_s-start_s)/span
                    near_error=abs(signed_lat*(1.-3.*q*q+2.*q*q*q))
                    if near_error>.050:
                        abort(f'Unified FollowPath JOIN_REFUSED: near via error {near_error:.3f}m >.050m')
            n=max(12,math.ceil(span/.0125))
            for j in range(1,n+1):
                t=j/n
                ss=start_s+span*t
                smooth=1.-3.*t*t+2.*t*t*t
                lat=signed_lat*smooth
                xx=a['x']+ux*ss+nx*lat
                yy=a['y']+uy*ss+ny*lat
                slope=signed_lat*(-6*t+6*t*t)/span
                hh=heading+math.atan(slope)
                # Preflight of every joining footprint AND 1.5cm transverse
                # position tolerance; existing 1.5cm white-line buffer remains.
                for side in (-.015,0.,.015):
                    testx=xx+nx*side;testy=yy+ny*side
                    if clearance(grid,testx,testy,hh)!='FREE' or lane_paint_near(testx,testy,hh):
                        abort(f'Unified FollowPath JOIN_REFUSED: clearance/paint at {(round(testx,3),round(testy,3))}')
                add(xx,yy,hh)
            end_xy=(a['x']+ux*end_s,a['y']+uy*end_s)
            join={'start':{'x':x,'y':y},
                  'end':{'x':end_xy[0],'y':end_xy[1]},
                  'end_s':end_s,'ux':ux,'uy':uy,'nominal_max':.085}
            first_s=end_s
            log('LANE_START_JOIN_PREFLIGHT_PASS',from_point=a['name'],
                to_point=b['name'],initial_lateral_m=round(dline,4),
                merge_length_m=round(span,4),endpoint=end_xy,
                note='Only joining section uses dynamic path reference; paint/map checks stay on')
        else:
            if first_s>=length-.04:
                abort('Unified FollowPath starts too close to endpoint')
        sections=[]
        for p in seq:
            ss=(p['x']-a['x'])*ux+(p['y']-a['y'])*uy
            if ss>first_s+1e-6:
                sections.append(ss)
        if not sections or sections[-1]<length-.001:
            abort('Unified FollowPath invalid ordered via points')
        prev_s=first_s
        for end_s in sections:
            count=max(1,math.ceil((end_s-prev_s)/.025))
            for k in range(1,count+1):
                ss=prev_s+(end_s-prev_s)*k/count
                add(a['x']+ux*ss,a['y']+uy*ss,heading)
            prev_s=end_s
        return traj,join

    def finish_lane_corner_turn(a,b):
        """Real Nav2 Spin after lane FollowPath completes, never during the straight.

        Preflight uses actual stopped TF, not nominal official checkpoint;
        +/-2cm 2D perturbation and 1-degree increments around the entire sweep.
        Motion aborts on runtime paint, map footprint, localization or drift.
        """
        nonlocal moving
        x,y,yaw=pos()
        lane_heading=math.atan2(b['y']-a['y'],b['x']-a['x'])
        if abs(normalize(yaw-lane_heading))>.18:
            abort('V55_TURN_BLOCKED: road-heading mismatch after FollowPath')
        endpoint_error=math.hypot(x-b['x'],y-b['y'])
        if endpoint_error>.09:
            abort(f'V55_TURN_BLOCKED: endpoint {endpoint_error:.3f}m >.09m')
        turn_delta=normalize(b['yaw']-yaw)
        if abs(turn_delta)<.10:
            log('V55_TURN_NOT_NEEDED',checkpoint=b['name'])
            return
        if not .9<abs(turn_delta)<1.8:
            abort('V55_TURN_BLOCKED: unexpected turn yaw '+str(round(turn_delta,3)))
        offsets=((0,0),(.02,0),(-.02,0),(0,.02),(0,-.02),
                 (.014,.014),(.014,-.014),(-.014,.014),(-.014,-.014))
        for j in range(91):
            check_yaw=yaw+turn_delta*j/90
            for ox,oy in offsets:
                marks=lane_paint_near(x+ox,y+oy,check_yaw)
                c=clearance(grid,x+ox,y+oy,check_yaw)
                if marks or c!='FREE':
                    abort(f'V55_TURN_BLOCKED: swept footprint at angle {check_yaw:.3f} '
                          f'offset={(ox,oy)}, paint={marks}, map={c}')
        # Failing to find the Spin server must stop here, not wait forever.
        from rclpy.action import ActionClient
        from nav2_msgs.action import Spin
        spin_client=ActionClient(nav,Spin,'spin')
        available=spin_client.wait_for_server(timeout_sec=4.0)
        spin_client.destroy()
        if not available:abort('V55_TURN_BLOCKED: Nav2 /spin action server absent')
        health(camera=args.mode=='patrol')
        log('V55_LANE_SPIN_START',checkpoint=b['name'],
            delta_rad=round(turn_delta,4),actual_stop=[x,y,yaw],
            endpoint_error_m=round(endpoint_error,4),sweep_guard_m=.02)
        pivot=(x,y)
        nav.spin(spin_dist=turn_delta,time_allowance=22)
        moving=True
        started=sim_seconds();wall_start=time.monotonic();max_drift=0.
        while not nav.isTaskComplete():
            spin(.035);health(camera=args.mode=='patrol')
            if sim_seconds()-started>26 or time.monotonic()-wall_start>75:
                abort('V55_TURN_ABORT: Nav2 Spin timed out')
            try:cx,cy,cyaw=pos()
            except Exception as e:abort('V55_TURN_ABORT: lost TF '+str(e))
            drift=math.hypot(cx-pivot[0],cy-pivot[1]);max_drift=max(max_drift,drift)
            if drift>.035:abort(f'V55_TURN_ABORT: pivot drift {drift:.3f}m >.035m')
            marks=lane_paint_near(cx,cy,cyaw)
            if marks:abort('V55_TURN_ABORT: paint '+','.join(marks))
            if clearance(grid,cx,cy,cyaw)!='FREE':
                abort('V55_TURN_ABORT: body intersects static obstacle')
        moving=False
        if nav.getResult()!=TaskResult.SUCCEEDED:
            abort('V55_TURN_ABORT: Nav2 Spin did not succeed')
        spin(.15)
        cx,cy,cyaw=pos()
        if (math.hypot(cx-pivot[0],cy-pivot[1])>.035 or
                abs(normalize(cyaw-b['yaw']))>.18 or
                lane_paint_near(cx,cy,cyaw) or
                clearance(grid,cx,cy,cyaw)!='FREE'):
            abort('V55_TURN_ABORT: unsafe final spin pose')
        log('V55_LANE_SPIN_DONE',checkpoint=b['name'],
            actual=[cx,cy,cyaw],max_pivot_drift_m=round(max_drift,4))

    def travel(seq,current,gate=None):
        nonlocal moving
        lane_leg = use_unified_follow(current,seq)
        lane_join=None
        goals=[current]+seq
        for a,b in zip(goals,goals[1:]):
            counts,bad=segment_check(grid,a,b)
            _,side_bad=lateral_margin_check(grid,a,b,margin=.04)
            if bad or side_bad:
                abort(f'Preflight segment {a["name"]}->{b["name"]} grid collision/margin: {(bad+side_bad)[0]}')
        health(camera=args.mode=='patrol')
        log('NAV_START',from_point=current['name'],goals=[p['name'] for p in seq],gate=gate)
        if lane_leg:
            lane_path,lane_join=centerline_follow_path(current,seq)
            log('LANE_FOLLOW_PATH_START',from_point=current['name'],to_point=seq[-1]['name'],
                path_poses=len(lane_path.poses),max_runtime_centerline_error_m=.055,
                note='Unified straight FollowPath; via points retained; real obstacle -> stop; no replanning')
            nav.followPath(lane_path)
        elif len(seq)==1:nav.goToPose(goal(seq[0]))
        else:nav.goThroughPoses([goal(p) for p in seq])
        moving=True;start_sim=sim_seconds();start_wall=time.monotonic()
        # For the first safe distance after GREEN staging, red/yellow before
        # actual entry triggers stop. After commitment do NOT abruptly brake in
        # the crosswalk; check the traffic gate before entry instead.
        gate_limit=.12 if gate else 0
        # AIC_PRECOMMIT_RAW_SIGNAL_20261013
        gate_green_wall=latest['signal_wall'] if gate else 0.0
        gate_image_ns=((latest['signal'] or {}).get('sim_stamp_ns',-1) if gate else -1)
        while not nav.isTaskComplete():
            spin(.04);health(camera=args.mode=='patrol')
            if sim_seconds()-start_sim>args.leg_timeout or time.monotonic()-start_wall>1800:
                abort('Nav2 leg timeout (sim clock or wall limit)')
            try:
                d,(x,y,yaw),segment=track(goals)
            except Exception as e:abort('Lost TF: '+str(e))
            # Narrow lamp-post corridor: 5cm lateral error already collides
            # with an occupied pixel in the user's ORIGINAL map.
            # The added via point splits the original narrow lamp-post corridor.
            # BOTH sublegs keep the original strict 3.5cm deviation stop.
            narrow=bool(segment and (segment[0],segment[1]) in (
                ('MID_LIGHT_WAIT','LEG12_GUIDE'), ('LEG12_GUIDE','BOTTOM_TURN')))
            bound=min(args.centerline_max,.035) if narrow else args.centerline_max
            if lane_leg:bound=min(bound,.055)  # <=5.5cm lane error, not a lane certificate
            # During the short, preflighted handoff from a Nav2 checkpoint
            # use the actual merge line as reference. Enforce the ORIGINAL
            # nominal-lane upper cap (.085m) and footprint+paint checks;
            # after the join, the regular 5.5cm centerline limit applies.
            if lane_leg and lane_join is not None:
                s=(x-current['x'])*lane_join['ux']+(y-current['y'])*lane_join['uy']
                if s<=lane_join['end_s']+.015:
                    if d>lane_join['nominal_max']:
                        abort(f'LANE_JOIN_ABORT: nominal-lane deviation {d:.3f}m >.085m')
                    join_err=centerline_error(x,y,lane_join['start'],lane_join['end'])
                    if join_err>.045:
                        abort(f'LANE_JOIN_ABORT: joining-path deviation {join_err:.3f}m >.045m')
                    d=join_err
            if d>bound:
                abort(f'Centerline deviation {d:.3f}m (limit {bound:.3f}, segment={segment}); safety stop')
            if lane_leg:
                contacts=lane_paint_near(x,y,yaw)
                if contacts:
                    abort('Paint-line clearance <1.5cm at actual pose: '+','.join(contacts))
            # Runtime footprint occupancy prevents passing known obstacles; Nav2
            # dynamic collision monitor provides an independent laser guard.
            cl=clearance(grid,x,y,yaw)
            if cl!='FREE':abort(f'Oriented robot footprint intersects {cl} grid at {(round(x,3),round(y,3))}')
            if gate:
                traveled=math.hypot(x-current['x'],y-current['y'])
                if traveled<gate_limit:
                    v=latest['signal'] or {}
                    now=time.monotonic()
                    age=now-latest['signal_wall'] if latest['signal_wall'] else 1e9
                    state=v.get('instant_state',v.get('state','UNKNOWN'))
                    conf=float(v.get('instant_confidence',v.get('confidence') or 0))
                    stamp=v.get('sim_stamp_ns')

                    if v.get('gate')!=gate:
                        abort(f'Traffic {gate} camera target mismatch: {v.get("gate")}')

                    if state in ('RED','YELLOW') and conf>=.7 and age<.8:
                        abort(f'Traffic {gate} observed {state} before commitment: STOP')

                    if (state=='GREEN' and conf>=.7 and age<.6 and
                            isinstance(stamp,int) and stamp>gate_image_ns):
                        gate_green_wall=now
                        gate_image_ns=stamp

                    if now-gate_green_wall>.80 or age>.8:
                        abort(f'Traffic {gate} no fresh green before commitment '
                              f'(raw={state},stable={v.get("state")},'
                              f'age={age:.2f}s,green_age={now-gate_green_wall:.2f}s)')
        moving=False
        result=nav.getResult()
        if result!=TaskResult.SUCCEEDED:abort('Nav2 failed: '+str(result))
        if lane_leg:
            # End of straight leg: only NOW rotate at real stopped position.
            finish_lane_corner_turn(current,seq[-1])
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
        # AIC_CAMERA_BOOTSTRAP_WAIT_20261013
        if args.mode=='patrol':
            wait_start=time.monotonic()
            while not latest['camera_wall'] or time.monotonic()-latest['camera_wall']>2:
                spin(.1)
                if time.monotonic()-wait_start>25:
                    raw=nav.get_publishers_info_by_topic('/camera/image_raw')
                    abort(f'CAMERA_BOOTSTRAP_TIMEOUT: /camera/image_raw publishers={len(raw)}')
            log('CAMERA_STREAM_READY',age_s=round(time.monotonic()-latest['camera_wall'],3))
        health(camera=args.mode=='patrol')
        t=time.monotonic()
        while True:
            spin(.1)
            try:x,y,yaw=pos();break
            except Exception:
                if time.monotonic()-t>20:abort('No map->base_footprint TF')
        # AIC_TOP_STAGE_RECOVERY_20261013
        # Recovery is deliberately limited to first traffic-light staging area.
        # Never replace physical position with a fake START / initial pose.
        if args.resume_top_stage:
            stage=stops[0]
            if stage['name']!='TOP_STAGING':
                abort('RESUME_REFUSED: first target no longer TOP_STAGING')
            dist=math.hypot(x-stage['x'],y-stage['y'])
            angle=abs(normalize(yaw-stage['yaw']))
            v=latest['motion_linear'];omega=latest['motion_angular']
            if dist>.09 or angle>.16:
                abort(f'RESUME_REFUSED: not at TOP_STAGING (distance={dist:.3f}, yaw_error={angle:.3f})')
            if v is None or omega is None or v>.025 or omega>.05:
                abort(f'RESUME_REFUSED: robot must be stationary (v={v}, omega={omega})')
            if lane_paint_near(x,y,yaw):
                abort('RESUME_REFUSED: body intersects road paint')
            cl=clearance(grid,x,y,yaw)
            if cl!='FREE':
                abort('RESUME_REFUSED: actual footprint intersects '+str(cl))
            # Explicitly do not claim the START->TOP_STAGING leg has been replayed.
            log('READY_RESUME_TOP_STAGING',actual=[x,y,yaw],distance_m=round(dist,4),
                note='PARTIAL RUN: start leg not rerun; top traffic gate still mandatory',out=str(out))
            prev=stage
            i=0  # resume at TOP_STAGING, keep mandatory light gate
        else:
            if math.hypot(x,y)>.12 or abs(normalize(yaw))>.25:
                abort(f'Robot not at map START {(x,y,yaw)}; use guarded recovery ONLY at TOP_STAGING')
            log('READY',start=[x,y,yaw],out=str(out))
            prev={'name':'START','x':x,'y':y,'yaw':yaw}
            i=0
        name_index={p['name']:i for i,p in enumerate(stops)}
        while i<len(stops):
            p=stops[i]
            if p['name']=='TOP_STAGING' or p['name']=='MID_STAGING':
                if args.resume_top_stage and p['name']=='TOP_STAGING' and i==0:
                    log('TOP_STAGE_ALREADY_REACHED',note='First traffic gate remains mandatory')
                else:
                    prev=travel([p],prev)
                i+=1
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
                if gate=='mid':
                    # Optimization validated by read-only NavFn via-point probe.
                    # This is a NON-OFFICIAL guide point, only in the Nav2 goal
                    # sequence. Original checkpoint identities and order remain.
                    wait_point, turn_point = via[-2], via[-1]
                    if (wait_point['name']!='MID_LIGHT_WAIT' or
                            turn_point['name']!='BOTTOM_TURN' or
                            abs(wait_point['x']-turn_point['x'])>0.003 or
                            not wait_point['y']+0.20 < 2.480 < turn_point['y']-0.20):
                        abort('LEG12_GUIDE geometry unexpected; refusing patrol')
                    guide=dict(wait_point)
                    guide.update(name='LEG12_GUIDE', y=2.480)
                    via=via[:-1]+[guide,via[-1]]
                    log('LEG12_GUIDE_INSERTED',map=[guide['x'],guide['y']],
                        note='Nav2 through-pose only; no change to official route')
                prev=travel(via,prev,gate=gate)
                for checkpoint in via:
                    if checkpoint['name']=='LEG12_GUIDE':
                        log('NAVIGATION_HELPER_PASSED',name='LEG12_GUIDE')
                    else:
                        log('CHECKPOINT_PASSED',name=checkpoint['name'],via=True)
                # The inserted LEG12_GUIDE is NOT an official stop.
                # Advance only by original checkpoints, otherwise PARKING_ENTRY is skipped.
                i+=len(expected_names)
                continue
            prev=travel([p],prev)
            if p['name'] in STOPS_AT:prev=camera_look_and_inspect(p,prev)
            i+=1
        log('FULL_PATROL_NAV_FINISHED',official_points=len(raw),
            partial_run=bool(args.resume_top_stage),
            disclaimer='No claim of lane, traffic, vision accuracy or dynamic collision certification')
        if args.resume_top_stage:
            print('V5_RESUMED_ROUTE_NAV_FINISHED: TOP_STAGING to FINISH; START leg not replayed')
        else:
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
