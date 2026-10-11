#!/usr/bin/env python3
"""Static VISUAL stripe screening, not a legal interpretation of competition rules.

The organizers' official diagram shows an unbroken parking separator AND a route
crossing it on departure and return. Those intersections are reported as
OFFICIAL_DIAGRAM_AMBIGUITY, NOT as a fabricated missing gap or an automatically
legal crossing. Other continuous lane-line overlaps remain warnings/blockers
for a targeted test segment. Crosswalks are not counted as solid lane boundaries.

This is solely a conservative 2D axis-aligned box screening. It cannot certify
stop-lines, a planned Nav2 path, or actual trajectory. It changes no world/map.
"""
import argparse
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import yaml


def segment_box(a, b, rect):
    """Liang–Barsky segment-AABB intersection."""
    x0, y0 = a
    x1, y1 = b
    xmin, ymin, xmax, ymax = rect
    dx, dy = x1 - x0, y1 - y0
    lo, hi = 0., 1.
    for p, q in ((-dx, x0 - xmin), (dx, xmax - x0),
                 (-dy, y0 - ymin), (dy, ymax - y0)):
        if abs(p) < 1e-12:
            if q < 0:
                return False
        else:
            t = q / p
            if p < 0:
                lo = max(lo, t)
            else:
                hi = min(hi, t)
            if lo > hi:
                return False
    return True


def read_stripes(sdf_file):
    root = ET.parse(sdf_file).getroot()
    marks = []
    for model in root.findall('./world/model'):
        name = model.get('name', '')
        if not name.startswith(('v_', 'h_', 'line_')):
            continue
        pose = [float(x) for x in (model.findtext('pose') or '').split()]
        box = model.find('./link/visual/geometry/box/size')
        if len(pose) < 6 or box is None:
            raise ValueError(f'No stripe visual geometry: {name}')
        sx, sy, _ = [float(x) for x in box.text.split()]
        if abs(pose[5]) > 1e-7:
            raise ValueError(f'Rotated stripe unsupported: {name}')
        x, y = pose[:2]
        marks.append((name, (x-sx/2, y-sy/2, x+sx/2, y+sy/2)))
    return marks


def overlaps_segment(a, b, rect, radius):
    x0,y0,x1,y1=rect
    conservative=(x0-radius,y0-radius,x1+radius,y1+radius)
    return segment_box(a,b,conservative), segment_box(a,b,rect)


def official_ambiguity(a_name,b_name,stripe):
    return stripe == 'v_parking_left' and (
        (a_name == 'START' and b_name == 'TOP_LIGHT_WAIT') or
        (a_name == 'RIGHT_TOP' and b_name == 'FINISH') or
        (a_name == 'START' and b_name in {'SMOKE_GOAL','FIRST_LIGHT_STAGING'})
    )


def screen_segment(a, b, markings, radius=0.24):
    """Returns (blocking_hits, official_crossing_ambiguities)."""
    pa=(float(a['x']),float(a['y']))
    pb=(float(b['x']),float(b['y']))
    blocking=[]; exceptions=[]
    for name,rect in markings:
        expanded, center = overlaps_segment(pa,pb,rect,radius)
        if not expanded:
            continue
        hit=(name, 'CENTERLINE' if center else 'FOOTPRINT_NEAR')
        if official_ambiguity(a['name'],b['name'],name):
            exceptions.append(hit)
        else:
            blocking.append(hit)
    return blocking, exceptions


def inspect(route_file, sdf_file, mode='all', distance=0.65, radius=0.24):
    route=yaml.safe_load(Path(route_file).read_text(encoding='utf-8'))['route']
    if len(route)!=17 or route[0]['name']!='START':
        raise ValueError('Expected 17 official route points, START first')
    markings=read_stripes(sdf_file)
    if mode in ('smoke','stage'):
        a,b=route[:2]
        length=math.hypot(float(b['x'])-float(a['x']),float(b['y'])-float(a['y']))
        if distance <= 0 or distance > length:
            raise ValueError('Requested stage/smoke distance outside first leg')
        t=distance/length
        goal={'name':'SMOKE_GOAL' if mode=='smoke' else 'FIRST_LIGHT_STAGING',
              'x':float(a['x'])+t*(float(b['x'])-float(a['x'])),
              'y':float(a['y'])+t*(float(b['y'])-float(a['y']))}
        pairs=[(a,goal)]
    else:
        pairs=list(zip(route[:-1],route[1:]))
    nblocking=0; nambiguous=0
    for i,(a,b) in enumerate(pairs,1):
        blocking,ambiguous=screen_segment(a,b,markings,radius)
        for mark,reason in ambiguous:
            nambiguous+=1
            print(f'OFFICIAL_DIAGRAM_AMBIGUITY {a["name"]}->{b["name"]}: '
                  f'{mark} {reason}; crossing appears in organizer diagram, '
                  'organizer interpretation NOT confirmed')
        for mark,reason in blocking:
            nblocking+=1
            print(f'OTHER_LANE_OVERLAP {a["name"]}->{b["name"]}: {mark} {reason}')
    print(f'LANE_SCREEN mode={mode} blockers={nblocking} '
          f'official_ambiguities={nambiguous} radius={radius:.3f}m')
    print('SCREEN_NOT_CERTIFICATION: actual Nav2 trajectory, legal lane/stop-line '
          'status and dynamic clearance require review / Gazebo testing')
    return nblocking == 0


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('route_yaml', type=Path)
    ap.add_argument('world_sdf', type=Path)
    ap.add_argument('--mode', choices=('all','smoke','stage'),default='all')
    ap.add_argument('--distance',type=float,default=.65)
    ap.add_argument('--radius',type=float,default=.24)
    args=ap.parse_args()
    try:
        return 0 if inspect(args.route_yaml,args.world_sdf,mode=args.mode,
                            distance=args.distance,radius=args.radius) else 1
    except (OSError,KeyError,ValueError,ET.ParseError) as exc:
        print(f'LANE_SCREEN_ERROR: {exc}',file=sys.stderr)
        return 2


if __name__=='__main__':
    sys.exit(main())
