#!/usr/bin/env python3
"""Pure-Python route geometry. Read-only; does not require ROS 2.

The saved 2-D SLAM map's origin is at START; START yaw=pi.  Mapping is
computed from the official route rather than hardcoding a translation.
"""
import json
import math
from pathlib import Path
import yaml

ROBOT_RADIUS = 0.24
# Staging = robot *center* BEFORE painted zebra. Not a certified stop line.
# Top zebra edge x=2.43; front half-length ~0.16; staging center x=2.84
# gives a nominal 0.25m body-front clearance (ignoring pose uncertainty).
TOP_STAGE_WORLD = (2.84, 3.886, math.pi)
# Middle zebra nearest edge y=1.67; staging center y=2.10 has ~0.27m
# body-front clearance; actual stop line and pose error MUST be confirmed.
MID_STAGE_WORLD = (1.994, 2.10, -math.pi/2)


def normalize(a):
    return math.atan2(math.sin(a), math.cos(a))


def load_points(path):
    obj = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    pts = obj['route']
    if len(pts) != 17 or pts[0]['name'] != 'START':
        raise ValueError('Official route must have 17 points and begin at START')
    for p in pts:
        for k in ('name', 'x', 'y', 'yaw'):
            if k not in p:
                raise ValueError(f'missing {k} in {p}')
    return pts


def to_local(p, start):
    angle = float(start['yaw']); c=math.cos(angle); s=math.sin(angle)
    dx=float(p['x'])-float(start['x']);dy=float(p['y'])-float(start['y'])
    return {'name':p.get('name',''), 'x':c*dx+s*dy,
            'y':-s*dx+c*dy,'yaw':normalize(float(p['yaw'])-angle),
            'action':p.get('action','')}


def planned_stops(raw):
    start=raw[0]
    points={p['name']:to_local(p,start) for p in raw}
    points['TOP_STAGING']=to_local({'name':'TOP_STAGING',
                    'x':TOP_STAGE_WORLD[0],'y':TOP_STAGE_WORLD[1],
                    'yaw':TOP_STAGE_WORLD[2]},start)
    points['MID_STAGING']=to_local({'name':'MID_STAGING',
                    'x':MID_STAGE_WORLD[0],'y':MID_STAGE_WORLD[1],
                    'yaw':MID_STAGE_WORLD[2]},start)
    # Never command the original 19cm-near-crosswalk waiting markers as a
    # *red-light stop*. Green must be positively observed BEFORE passing.
    names=['TOP_STAGING','TOP_LIGHT_WAIT','TOP_AFTER_LIGHT','LEFT_CORNER',
           'A_LEFT_VIEW','MIDDLE_LEFT','A_BOTTOM_B_TOP_VIEW',
           'CENTER_TURN','MID_STAGING','B_RIGHT_VIEW','MID_LIGHT_WAIT','BOTTOM_TURN','PARKING_ENTRY',
           'PLATE_1','PLATE_2','PLATE_3','RIGHT_TOP','FINISH']
    return [points[n] for n in names]


def centerline_error(x,y,a,b):
    dx=b['x']-a['x'];dy=b['y']-a['y']
    t=max(0.,min(1.,((x-a['x'])*dx+(y-a['y'])*dy)/(dx*dx+dy*dy))) if dx*dx+dy*dy>1.e-8 else 0.
    return math.hypot(x-a['x']-t*dx, y-a['y']-t*dy)


def main():
    import argparse
    ap=argparse.ArgumentParser()
    ap.add_argument('official_route_yaml')
    args=ap.parse_args()
    raw=load_points(args.official_route_yaml)
    print('ORIGINAL_OFFICIAL_ROUTE_POINTS',len(raw))
    print('EXECUTION_STOPS (includes 2 additional conservative light staging points)')
    for i,p in enumerate(planned_stops(raw)):
        print(f"{i:02d} {p['name']:<24} map=({p['x']:+.3f},{p['y']:+.3f},{p['yaw']:+.3f})")
    print('NOT_CERTIFIED: ground truth stop lines and lane boundary rules still require field check')

if __name__=='__main__': main()
