#!/usr/bin/env python3
"""Conservative oriented-footprint geometry audit on original SLAM map.

This is an occupancy-map filter, NOT proof of dynamic safety or lane compliance.
The actual robot.xacro has 0.32x0.24 body and wheels at +/-0.165.
Polygon overapproximates body + wheels with ~1cm safety margin. No map edits.
"""
import argparse
from collections import Counter
import math
from pathlib import Path
import yaml
from aic_v5_route import load_points, planned_stops, normalize

# Approximate convex envelope of chassis+two wheels in base_link (X forward, Y left).
# Intentional 1cm margin on body and wheel extremities. Fits narrow lane
# better than false circular r=.24 while still enclosing original collision parts.
FOOTPRINT = [(-.17,-.13),(-.17,.13),(-.015,.19),(.105,.19),
             (.17,.13),(.17,-.13),(.105,-.19),(-.015,-.19)]


def read_pgm(path):
    data=Path(path).read_bytes();i=0;tokens=[]
    while len(tokens)<4:
        while i<len(data) and data[i] in b' \r\n\t':i+=1
        if data[i:i+1]==b'#':i=data.index(b'\n',i)+1;continue
        j=i
        while i<len(data) and data[i] not in b' \n\t\r#':i+=1
        tokens.append(data[j:i].decode('ascii'))
    kind,w,h,maxv=tokens
    if kind!='P5' or int(maxv)!=255:raise ValueError('Expected 8bit P5 PGM')
    if data[i:i+2]==b'\r\n':i+=2
    elif data[i:i+1] in (b' ',b'\n',b'\r',b'\t'):i+=1
    else:raise ValueError('Bad PGM header')
    pixels=data[i:]
    if len(pixels)!=int(w)*int(h):raise ValueError('Bad PGM pixels')
    return int(w),int(h),pixels


def load_map(path):
    path=Path(path);meta=yaml.safe_load(path.read_text())
    w,h,pix=read_pgm(path.parent/meta['image'])
    return meta,w,h,pix


def poly_at(x,y,yaw):
    c,s=math.cos(yaw),math.sin(yaw)
    return [(x+c*a-s*b,y+s*a+c*b) for a,b in FOOTPRINT]


def overlapping(poly, square):
    # Separating axis theorem for convex footprint against axis-aligned pixel.
    for shape in (poly,square):
        for i,p in enumerate(shape):
            q=shape[(i+1)%len(shape)];axis=(-(q[1]-p[1]),q[0]-p[0])
            lo1,hi1=min(u*axis[0]+v*axis[1] for u,v in poly),max(u*axis[0]+v*axis[1] for u,v in poly)
            lo2,hi2=min(u*axis[0]+v*axis[1] for u,v in square),max(u*axis[0]+v*axis[1] for u,v in square)
            if hi1 < lo2-1.e-9 or hi2 < lo1-1.e-9:return False
    return True


def clearance(grid,x,y,yaw):
    meta,w,h,pixels=grid;res=float(meta['resolution']);ox,oy,theta=meta['origin']
    if abs(theta)>1e-6:raise ValueError('V5 map grid assumes map origin yaw=0')
    p=poly_at(x,y,yaw)
    cs=[(a-ox)/res for a,b in p];rs=[(b-oy)/res for a,b in p]
    for row in range(math.floor(min(rs)),math.floor(max(rs))+1):
        for col in range(math.floor(min(cs)),math.floor(max(cs))+1):
            if row<0 or row>=h or col<0 or col>=w:return 'EDGE'
            box=[(ox+col*res,oy+row*res),(ox+(col+1)*res,oy+row*res),
                 (ox+(col+1)*res,oy+(row+1)*res),(ox+col*res,oy+(row+1)*res)]
            if not overlapping(p,box):continue
            pixel=pixels[(h-1-row)*w+col];prob=(1-pixel/255) if not meta.get('negate',0) else pixel/255
            if prob>=float(meta.get('occupied_thresh',.65)):return 'OCCUPIED'
            if prob>float(meta.get('free_thresh',.196)):return 'UNKNOWN'
    return 'FREE'


def segment_check(grid,a,b,step=.025):
    length=math.hypot(b['x']-a['x'],b['y']-a['y']);n=max(1,math.ceil(length/step))
    # During translation use direction of movement, not a yaw interpolated over
    # the whole segment. Arrival yaw rotation is separately swept below.
    motion_heading=math.atan2(b['y']-a['y'],b['x']-a['x']) if length>.005 else a['yaw']
    observations=Counter();bad=[]
    for j in range(n+1):
        t=j/n;x=a['x']+(b['x']-a['x'])*t;y=a['y']+(b['y']-a['y'])*t
        verdict=clearance(grid,x,y,motion_heading)
        observations[verdict]+=1
        if verdict!='FREE':bad.append((round(x,3),round(y,3),round(motion_heading,3),verdict))
    # Check at arrival when rotating from travel yaw to requested waypoint yaw,
    # samples of 10 degrees. This doesn't represent a guaranteed Nav2 motion.
    delta=normalize(b['yaw']-motion_heading)
    if abs(delta)>.01:
        for j in range(1,math.ceil(abs(delta)/math.radians(10))+1):
            yaw=motion_heading+delta*j/math.ceil(abs(delta)/math.radians(10))
            verdict=clearance(grid,b['x'],b['y'],yaw); observations[verdict]+=1
            if verdict!='FREE':bad.append((round(b['x'],3),round(b['y'],3),round(yaw,3),verdict))
    return observations,bad


def lateral_margin_check(grid, a, b, margin=.04, step=.025):
    """Check ±margin offsets from the centerline for linear motion.

    Independent of painted lane rules: this is occupancy-map robustness only.
    Rotation at the endpoint is checked by segment_check, not by this function.
    """
    length=math.hypot(b['x']-a['x'],b['y']-a['y'])
    if length<.005:return (Counter(),[])
    n=max(1,math.ceil(length/step))
    heading=math.atan2(b['y']-a['y'],b['x']-a['x'])
    nx=-math.sin(heading);ny=math.cos(heading)
    counts=Counter();bad=[]
    for side in (-1,1):
        for j in range(n+1):
            t=j/n
            x=a['x']+(b['x']-a['x'])*t+nx*side*margin
            y=a['y']+(b['y']-a['y'])*t+ny*side*margin
            verdict=clearance(grid,x,y,heading)
            counts[verdict]+=1
            if verdict!='FREE':bad.append((round(x,3),round(y,3),side,verdict))
    return counts,bad


def report(route_yaml,map_yaml):
    stops=planned_stops(load_points(route_yaml));grid=load_map(map_yaml)
    previous=dict(stops[0]);previous['name']='START';previous['x']=0.;previous['y']=0.;previous['yaw']=0.
    blocked=[]
    for goal in stops:
        counts,bad=segment_check(grid,previous,goal)
        # At least 4cm lateral margin on both sides for ideal-line map geometry.
        # Does NOT guarantee actual dynamic clearance or lane-line legality.
        margin_counts,margin_bad=lateral_margin_check(grid,previous,goal,margin=.04)
        if bad or margin_bad:blocked.append((previous['name'],goal['name'],(bad+margin_bad)[:3]))
        print('%-23s -> %-23s %s %s'%(previous['name'],goal['name'],dict(counts),'REVIEW_REQUIRED' if bad else 'FOOTPRINT_FREE_GRID_ONLY'))
        previous=goal
    print('ORIENTED_FOOTPRINT_REVIEW_SEGMENTS',len(blocked),'OF',len(stops))
    for a,b,h in blocked:print('REVIEW',a,'->',b,h)
    print('MAP_ONLY: this does not prove Nav2 can follow without crossing road lines or colliding')
    return blocked


if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('route_yaml');ap.add_argument('map_yaml');a=ap.parse_args()
    report(a.route_yaml,a.map_yaml)
