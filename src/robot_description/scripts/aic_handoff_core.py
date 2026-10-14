#!/usr/bin/env python3
"""Pure standard-library checks for AIC SLAM -> Nav2 handoff (fail closed)."""
import hashlib
import json
import math
from pathlib import Path


class HandoffError(RuntimeError):
    pass


def load_manifest(path, expected_root=None):
    p=Path(path).expanduser().resolve(strict=True)
    data=json.loads(p.read_text(encoding='utf-8'))
    if data.get('schema')!='aic_slam_to_nav2_v2' or data.get('complete') is not True:
        raise HandoffError('Incomplete or wrong-schema handoff')
    if data.get('map_frame')!='map' or data.get('robot_frame')!='base_footprint':
        raise HandoffError('Map/robot frame mismatch')
    if not isinstance(data.get('visited'),int) or not isinstance(data.get('total'),int):
        raise HandoffError('No coverage counts')
    if data['total']<5 or data['visited']!=data['total']:
        raise HandoffError('Road coverage not complete')
    y=Path(data['map_yaml']).expanduser().resolve(strict=True)
    img=Path(data['map_pgm']).expanduser().resolve(strict=True)
    if expected_root is not None:
        root=Path(expected_root).resolve(strict=True)
        if not y.is_relative_to(root) or not img.is_relative_to(root) or not p.is_relative_to(root):
            raise HandoffError('Output path escapes handoff directory')
    if y.suffix != '.yaml' or img.suffix != '.pgm' or y.stat().st_size<60 or img.stat().st_size<100:
        raise HandoffError('Unusable map files')
    if hashlib.sha256(img.read_bytes()).hexdigest()!=data.get('map_sha256'):
        raise HandoffError('Map checksum mismatch')
    try:
        import yaml
    except ImportError as exc:
        raise HandoffError('PyYAML unavailable in Python environment') from exc
    meta=yaml.safe_load(y.read_text(encoding='utf-8'))
    if Path(str(meta.get('image',''))).name != img.name:
        raise HandoffError('YAML image does not reference the expected PGM')
    if not (0.01<=float(meta.get('resolution',0))<=0.10):
        raise HandoffError('Map resolution outside supported range')
    pose=data.get('pose_map',{})
    try:
        x=float(pose['x']);ypos=float(pose['y']);yaw=float(pose['yaw'])
    except (TypeError, KeyError, ValueError) as exc:
        raise HandoffError('Missing map-frame end pose') from exc
    if not all(map(math.isfinite,[x,ypos,yaw])):
        raise HandoffError('Nonfinite end pose')
    if math.hypot(x,ypos)>.09 or abs(math.atan2(math.sin(yaw),math.cos(yaw)))>.14:
        raise HandoffError('Robot is not back at START; patrol unsafe')
    return data


def patrol_arguments(enable_patrol=False, ack=False, map_yaml=None):
    if not enable_patrol:return None
    if not ack:raise HandoffError('Patrol requires explicit --ack-official-crossing')
    if not map_yaml:raise HandoffError('Missing NEW map path')
    return ['ros2','run','robot_description','aic_v5_patrol.py','--mode','patrol',
            '--ack-official-crossing','--map-yaml',str(map_yaml)]
