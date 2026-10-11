#!/usr/bin/env python3
"""Conservative static occupancy screening of a FIRST-leg test trajectory.

Operates on the saved map only; it does not infer permitted line crossings,
stop-line locations or guarantee that Nav2 follows a straight line.
"""
import argparse
import math
from pathlib import Path
import sys
import yaml
from route_preflight import read_pgm


def screen_first_leg(route_file, map_yaml, distance, radius=.24, step=.025):
    route=yaml.safe_load(Path(route_file).read_text(encoding='utf-8'))['route']
    a,b=route[:2]
    length=math.hypot(b['x']-a['x'],b['y']-a['y'])
    if distance<=0 or distance>=length:
        raise ValueError('Target must be within first route leg')
    meta=yaml.safe_load(Path(map_yaml).read_text(encoding='utf-8'))
    w,h,mx,pixels=read_pgm(Path(map_yaml).parent / meta['image'])
    ox,oy,ang=map(float,meta['origin'])
    res=float(meta['resolution']);free=float(meta.get('free_thresh',.196))
    occupied=float(meta.get('occupied_thresh',.65))
    negate=int(meta.get('negate',0))
    # footprint sampling is deliberately conservative (pixel-cell overapproximation)
    pix_rad=int(math.ceil(radius/res))
    neig=[(dx,dy) for dx in range(-pix_rad,pix_rad+1)
          for dy in range(-pix_rad,pix_rad+1)
          if math.hypot(dx*res,dy*res)<=radius+res*.5]
    yaw0=float(a['yaw']); c=math.cos(yaw0);s=math.sin(yaw0)
    ca=math.cos(ang);sa=math.sin(ang)
    counts={}
    samples=max(1,math.ceil(distance/step))
    for j in range(samples+1):
        t=(j/samples)*(distance/length)
        wx=a['x']+t*(b['x']-a['x']);wy=a['y']+t*(b['y']-a['y'])
        mdx,mdy=wx-a['x'],wy-a['y']
        x=c*mdx+s*mdy; y=-s*mdx+c*mdy
        dx=x-ox;dy=y-oy
        col=int(math.floor((ca*dx+sa*dy)/res))
        row=h-1-int(math.floor((-sa*dx+ca*dy)/res))
        verdict='FREE'
        for ix,iy in neig:
            cc,rr=col+ix,row+iy
            if not (0<=cc<w and 0<=rr<h):
                verdict='EDGE';break
            val=pixels[rr*w+cc]/mx
            p=val if negate else 1-val
            if p>occupied:
                verdict='OCCUPIED';break
            if p>=free and verdict=='FREE':
                verdict='UNKNOWN'
        counts[verdict]=counts.get(verdict,0)+1
    ok=set(counts)<={'FREE'}
    print(f'STATIC_FIRST_LEG distance={distance:.3f}m radius={radius:.2f}m '
          f'counts={counts} verdict={"FREE_GRID_ONLY" if ok else "BLOCK"}')
    print('NOT_CERTIFIED: map/scan cannot detect painted road markings or '
          'verify stop-line/real controller tracking')
    return ok


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('route_yaml',type=Path)
    ap.add_argument('map_yaml',type=Path)
    ap.add_argument('--distance',type=float,default=.65)
    ap.add_argument('--radius',type=float,default=.24)
    args=ap.parse_args()
    try:
        return 0 if screen_first_leg(args.route_yaml,args.map_yaml,
                                     args.distance,args.radius) else 1
    except Exception as exc:
        print(f'MAP_GATE_ERROR {exc}',file=sys.stderr)
        return 2


if __name__=='__main__':
    sys.exit(main())
