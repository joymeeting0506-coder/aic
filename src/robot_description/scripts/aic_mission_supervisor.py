#!/usr/bin/env python3
"""AIC SIM one-process supervisor: live SLAM coverage -> map -> AMCL/Nav2 -> patrol.

Attaches to an existing SINGLE Gazebo+SLAM simulation. Deliberately refuses to
spawn Gazebo twice or hand off when coverage/save/AMCL readiness is uncertain.
No automatic patrol without explicit crossing-rule acknowledgement.
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from aic_handoff_core import load_manifest, patrol_arguments, HandoffError


class MissionError(RuntimeError):pass


def call(cmd,timeout=12,check=True):
    res=subprocess.run(cmd,capture_output=True,text=True,timeout=timeout)
    if check and res.returncode:
        raise MissionError(f"Command failed ({res.returncode}): {' '.join(map(str,cmd))}\n"+
                           (res.stdout+'\n'+res.stderr)[-1600:])
    return res.stdout


def nodes():
    return set(call(['ros2','node','list'],timeout=12).splitlines())


def topic_publishers(topic):
    data=call(['ros2','topic','info',topic,'-v'],timeout=12)
    return int(re.search(r'Publisher count:\s*(\d+)',data).group(1))


def map_slam_publisher():
    out=call(['ros2','topic','info','/map','-v'],timeout=12)
    n=int(re.search(r'Publisher count:\s*(\d+)',out).group(1))
    return n==1 and 'Node name: slam_toolbox' in out


def lifecycle_state(name):
    out=call(['ros2','lifecycle','get',name],timeout=13,check=False)
    return out.lower()


def wait_for(predicate,seconds,label,check_process=None):
    deadline=time.monotonic()+seconds
    while time.monotonic()<deadline:
        if check_process is not None and check_process.poll() is not None:
            raise MissionError(label+f': process exited unexpectedly rc={check_process.returncode}')
        try:
            if predicate():return
        except (MissionError,subprocess.TimeoutExpired,ValueError,AttributeError):pass
        time.sleep(2)
    raise MissionError('TIMEOUT: '+label)


def start(cmd):
    print('START:', ' '.join(map(str,cmd)),flush=True)
    return subprocess.Popen(cmd,start_new_session=True)


def stop(proc):
    if proc is None or proc.poll() is not None:return
    try:os.killpg(proc.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:proc.wait(timeout=12)
    except subprocess.TimeoutExpired:
        try:os.killpg(proc.pid,signal.SIGTERM)
        except ProcessLookupError:pass
        try:proc.wait(timeout=6)
        except subprocess.TimeoutExpired:
            try:os.killpg(proc.pid,signal.SIGKILL)
            except ProcessLookupError:pass


def preflight_sim():
    ns=nodes()
    required={'/slam_toolbox','/frame_remap','/robot_state_publisher'}
    if not required<=ns:raise MissionError('Missing Gazebo/SLAM bridge nodes: '+str(required-ns))
    incompatible={'/amcl','/map_server','/bt_navigator','/controller_server',
                  '/aic_coverage_mapping','/aic_handoff_initial_pose'}
    if ns & incompatible:raise MissionError('Competing navigation/coverage nodes: '+str(ns&incompatible))
    if not map_slam_publisher():raise MissionError('Expected exactly one SLAM /map publisher')
    if topic_publishers('/cmd_vel')!=0:
        raise MissionError('Another /cmd_vel controller is already running')
    print('PREFLIGHT_OK: existing Gazebo+SLAM only, /cmd_vel controller free',flush=True)


def shutdown_slam():
    print('HANDOFF: shutting down SLAM lifecycle while preserving Gazebo/bridge',flush=True)
    out=call(['ros2','lifecycle','set','/slam_toolbox','shutdown'],timeout=25,check=False)
    print(out.strip(),flush=True)
    # A late background SLAM /map publisher or its map->odom TF is not acceptable.
    def gone():
        ns=nodes()
        if '/slam_toolbox' in ns:
            return 'finalized' in lifecycle_state('/slam_toolbox') and not map_slam_publisher()
        return True
    wait_for(gone,25,'slam_toolbox publisher shutdown')
    if map_slam_publisher():raise MissionError('SLAM /map still active; refuse Nav2 startup')
    if '/frame_remap' not in nodes():raise MissionError('Gazebo bridge lost; refuse Nav2 startup')


def static_patrol_preflight(map_yaml):
    cmd=['ros2','run','robot_description','aic_v5_patrol.py','--mode','preview',
         '--map-yaml',str(map_yaml)]
    print('NAV_PREFLIGHT:', ' '.join(cmd),flush=True)
    report=call(cmd,timeout=35)
    bad=re.search(r'ORIENTED_FOOTPRINT_REVIEW_SEGMENTS\s+(\d+)',report)
    if not bad:
        raise MissionError('Patrol preview has no recognizable footprint audit; fail closed')
    if int(bad.group(1)):
        raise MissionError(f'Patrol preview rejected {bad.group(1)} occupied/unknown road segments')
    if 'V5_PREVIEW_OK' not in report:
        raise MissionError('Patrol preview did not report success')
    print('PATROL_STATIC_PREFLIGHT_OK (visual-line/real-motion proof NOT implied)',flush=True)


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--drive',action='store_true',help='Actuate auto-coverage; OFF by default')
    p.add_argument('--resume-legacy-route',action='store_true',help='Only if old 17-point route JUST ran in SAME session')
    p.add_argument('--patrol',action='store_true',help='Run existing V5 competition patrol after handoff')
    p.add_argument('--ack-official-crossing',action='store_true',help='Explicitly accept organizer crossing interpretation')
    p.add_argument('--workspace',default=str(Path.home()/'aic_ws'))
    p.add_argument('--mapping-timeout',type=int,default=1500)
    p.add_argument('--preview-seconds',type=int,default=20)
    p.add_argument('--nav-timeout',type=int,default=150)
    args=p.parse_args(argv)
    if args.patrol and not args.drive:raise MissionError('Cannot patrol in non-driving preview')
    if args.patrol and not args.ack_official_crossing:
        raise MissionError('--patrol requires --ack-official-crossing; do not bypass line rule gate')
    if args.mapping_timeout < 60 or args.nav_timeout < 40:
        raise MissionError('Unsupported timeout')
    ws=Path(args.workspace).expanduser().resolve(strict=True)
    if not (ws/'src/robot_description/CMakeLists.txt').is_file():
        raise MissionError('This is not the expected workspace')
    # Fresh Gazebo/SLAM startup may take 30-90s on VMware; never race startup.
    wait_for(lambda: (preflight_sim() or True),120,'live Gazebo+SLAM preflight')
    out=ws/'aic_results/autonomous_mapping';out.mkdir(parents=True,exist_ok=True)
    stamp=datetime.now().strftime('%Y%m%d_%H%M%S')+'_'+str(os.getpid())
    handoff=out/('handoff_'+stamp+'.json')
    ready=out/('amcl_ready_'+stamp+'.json')
    processes=[]
    try:
        # One mapping publisher; supervisor only starts the COVERAGE CONTROLLER.
        graph=ws/'src/robot_description/config/aic_sim_road_graph.json'
        if not graph.is_file():
            raise MissionError(f'Road graph missing: {graph}')
        preview_status=out/('preview_'+stamp+'.json')
        cmd=['ros2','run','robot_description','aic_coverage_mapping.py',
            '--ros-args','-p',f'graph_path:={graph}',
            '-p',f'preview_status_path:={preview_status}',
            '-p',f'drive:={str(args.drive).lower()}',
            '-p',f'resume_legacy_route:={str(args.resume_legacy_route).lower()}',
            '-p',f'handoff_path:={handoff}','-p','use_sim_time:=true']
        mapping=start(cmd);processes.append(mapping)
        if not args.drive:
            deadline=time.monotonic()+args.preview_seconds
            while time.monotonic()<deadline:
                if mapping.poll() is not None:
                    raise MissionError('Preview controller exited early')
                if preview_status.is_file():
                    break
                time.sleep(.5)
            if not preview_status.is_file():
                raise MissionError('PREVIEW_NOT_READY: no verified map/scan/TF/A* plan before timeout; check /clock and HOLD warnings')
            report=json.loads(preview_status.read_text(encoding='utf-8'))
            if not (report.get('schema')=='aic_coverage_preview_v1' and
                    report.get('ready') is True and report.get('path_cells',0)>0 and
                    0<=report.get('target_index',-1)<report.get('samples',0)):
                raise MissionError('PREVIEW_INVALID: planner readiness report failed validation')
            stop(mapping)
            if mapping.returncode not in (0,-signal.SIGINT,128+signal.SIGINT):
                raise MissionError(f'Preview controller failed to exit cleanly: rc={mapping.returncode}')
            print(f'PREVIEW_FINISHED: A* plan ready, target={report["target_index"]+1}/{report["samples"]}, '
                  f'path_cells={report["path_cells"]}; stopped cleanly, no motion/Nav2',flush=True)
            return 0
        try:
            rc=mapping.wait(timeout=args.mapping_timeout)
        except subprocess.TimeoutExpired as exc:
            raise MissionError('Coverage timeout; abort and keep old map') from exc
        if rc!=0:
            raise MissionError('Coverage exited nonzero; no Nav2 handoff')
        if not handoff.is_file():
            raise MissionError('Coverage exited without atomic handoff manifest')
        data=load_manifest(handoff,expected_root=out)
        print('MAP_SAVED_AND_COVERAGE_VALIDATED:',data['map_yaml'],flush=True)
        # Static new-map footprint audit should happen BEFORE removing SLAM.
        # If it fails, remain stopped and retain current SLAM for review.
        if args.patrol:static_patrol_preflight(data['map_yaml'])
        shutdown_slam()
        # Preserve SINGLE Gazebo and bridge. This launch starts only AMCL, map_server, Nav2.
        navcmd=['ros2','launch','robot_description','aic_nav_after_slam.launch.py',
                f"map_yaml:={data['map_yaml']}",f'handoff_path:={handoff}',
                f'ready_path:={ready}']
        nav=start(navcmd);processes.append(nav)
        required=['/map_server','/amcl','/planner_server','/controller_server',
                  '/bt_navigator','/collision_monitor']
        def nav_ready():
            if not ready.is_file():return False
            a=json.loads(ready.read_text(encoding='utf-8'))
            if a.get('ready') is not True or a.get('map_yaml') != data['map_yaml']:
                return False
            for name in required:
                if 'active' not in lifecycle_state(name):return False
            # Critical: the saved map must be the ONLY publisher of /map.
            if topic_publishers('/map')!=1:return False
            return True
        wait_for(nav_ready,args.nav_timeout,'AMCL and Nav2 active on NEW saved map',check_process=nav)
        print('NAV2_HANDOFF_READY: Gazebo remains running; AMCL localized to new map',flush=True)
        if not args.patrol:
            print('NAV2_READY_HOLD: no patrol motion; to patrol, run supervisor with --patrol --ack-official-crossing',flush=True)
            while nav.poll() is None:time.sleep(1)
            raise MissionError('Nav2 process ended unexpectedly')
        vision=start(['ros2','run','robot_description','aic_v5_vision.py']);processes.append(vision)
        time.sleep(3)
        if vision.poll() is not None:
            raise MissionError('Vision node quit before patrol; fail closed')
        cmd=patrol_arguments(True,args.ack_official_crossing,data['map_yaml'])
        patrol=start(cmd);processes.append(patrol)
        # If vision or Nav2 die, immediately stop patrol instead of waiting blindly.
        while patrol.poll() is None:
            if vision.poll() is not None or nav.poll() is not None:
                raise MissionError('Vision/Nav2 exited during patrol; stopping motion')
            time.sleep(1)
        if patrol.returncode:
            raise MissionError(f'Patrol failed with exit status {patrol.returncode}')
        print('AIC_MISSION_COMPLETE: mapping -> Nav2 -> patrol passed launcher exit gates',flush=True)
        return 0
    finally:
        for proc in reversed(processes):stop(proc)
        # Never kill Gazebo or existing bridge/robot_state_publisher.
        # On failed handoff, leave the current SLAM session intact where possible.


if __name__=='__main__':
    try:sys.exit(main())
    except KeyboardInterrupt:
        print('USER_ABORTED: child controllers stopped; saved maps retained',file=sys.stderr)
        sys.exit(130)
    except (MissionError,HandoffError,FileNotFoundError,subprocess.TimeoutExpired) as e:
        print('MISSION_BLOCKED: '+str(e),file=sys.stderr)
        sys.exit(2)
