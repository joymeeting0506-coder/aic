#!/usr/bin/env python3
"""Offline screening of AIC route against a saved SLAM occupancy grid.

Not a competition lane-line/stop-line certification: painted lines are often
invisible to 2D LiDAR and therefore absent from occupancy grids.
"""
import argparse
import math
from pathlib import Path
import sys
import yaml


def read_pgm(path):
    raw = path.read_bytes()
    pos = 0
    tokens = []
    for _ in range(4):
        while pos < len(raw):
            if raw[pos] in b' \t\n\r':
                pos += 1
            elif raw[pos] == 35:  # header comment
                end = raw.find(b'\n', pos)
                pos = len(raw) if end < 0 else end + 1
            else:
                break
        start = pos
        while pos < len(raw) and raw[pos] not in b' \t\n\r#':
            pos += 1
        if start == pos:
            raise ValueError('Bad PGM header')
        tokens.append(raw[start:pos].decode('ascii'))
    mode, w, h, maxval = tokens
    w, h, maxval = int(w), int(h), int(maxval)
    if mode not in ('P5', 'P2') or not (1 <= maxval <= 65535):
        raise ValueError('Unsupported PGM format')
    if mode == 'P2':
        # ASCII PGM comments may appear among sample values.
        tail = raw[pos:].decode('ascii').splitlines()
        vals = []
        for line in tail:
            vals.extend(int(x) for x in line.split('#', 1)[0].split())
    else:
        # Raster starts after precisely one whitespace delimiter (CRLF allowed).
        if pos >= len(raw) or raw[pos] not in b' \t\n\r':
            raise ValueError('Missing PGM raster separator')
        pos += 2 if raw[pos:pos+2] == b'\r\n' else 1
        n = w * h
        if maxval < 256:
            vals = list(raw[pos:pos+n])
        else:
            block = raw[pos:pos+2*n]
            vals = [int.from_bytes(block[i:i+2], 'big') for i in range(0, len(block), 2)]
    if len(vals) != w * h:
        raise ValueError(f'PGM pixel count mismatch {len(vals)} != {w*h}')
    return w, h, maxval, vals


def world_to_map(route):
    anchor = route[0]
    x0, y0, yaw0 = [float(anchor[k]) for k in ('x', 'y', 'yaw')]
    c, s = math.cos(yaw0), math.sin(yaw0)
    out = []
    for p in route:
        dx, dy = float(p['x']) - x0, float(p['y']) - y0
        out.append((str(p['name']), c*dx+s*dy, -s*dx+c*dy))
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('route_yaml', type=Path)
    ap.add_argument('map_yaml', type=Path)
    ap.add_argument('--robot-radius', type=float, default=0.22)
    ap.add_argument('--extra-margin', type=float, default=0.02)
    ap.add_argument('--sample-step', type=float, default=0.025)
    args = ap.parse_args()

    route = yaml.safe_load(args.route_yaml.read_text(encoding='utf-8'))['route']
    if not route or route[0].get('name') != 'START':
        raise SystemExit('ERROR: expected START as first waypoint')
    info = yaml.safe_load(args.map_yaml.read_text(encoding='utf-8'))
    map_file = args.map_yaml.parent / info['image']
    w, h, maxval, pixels = read_pgm(map_file)
    res = float(info['resolution'])
    ox, oy, oyaw = map(float, info['origin'])
    ca, sa = math.cos(oyaw), math.sin(oyaw)
    negate = int(info.get('negate', 0))
    free_threshold = float(info.get('free_thresh', 0.196))
    occupied_threshold = float(info.get('occupied_thresh', 0.65))
    clearance = args.robot_radius + args.extra_margin
    radius_pixels = math.ceil(clearance/res)
    neighbors = [(di, dj) for di in range(-radius_pixels, radius_pixels+1)
                 for dj in range(-radius_pixels, radius_pixels+1)
                 if math.hypot(di*res, dj*res) <= clearance + res * 0.5]

    def grid(x, y):
        dx, dy = x-ox, y-oy
        # map pixel axes relative to rotated map origin, PGM first row at top
        gx, gy = ca*dx + sa*dy, -sa*dx + ca*dy
        return int(math.floor(gx/res)), h-1-int(math.floor(gy/res))

    def status(x, y):
        col, row = grid(x, y)
        if not (0 <= col < w and 0 <= row < h):
            return 'OUTSIDE'
        saw_unknown = False
        for dc, dr in neighbors:
            cc, rr = col+dc, row+dr
            if not (0 <= cc < w and 0 <= rr < h):
                return 'EDGE_CLEARANCE'
            value = pixels[rr*w+cc]/maxval
            probability = value if negate else 1.0-value
            if probability > occupied_threshold:
                return 'OCCUPIED_NEAR'
            if probability >= free_threshold:
                saw_unknown = True
        return 'UNKNOWN_NEAR' if saw_unknown else 'FREE_GRID'

    pts = world_to_map(route)
    print(f'MAP {w}x{h} @ {res}m origin={info["origin"]}; '
          f'robot clearance screening radius={clearance:.2f}m')
    if len(pts) != 17:
        print(f'WARN route has {len(pts)} points, expected 17')
    count = {}
    anomaly_count = 0
    for i, (name, x, y) in enumerate(pts):
        label = status(x, y)
        count[label] = count.get(label, 0)+1
        if label != 'FREE_GRID': anomaly_count += 1
        print(f'POINT {i:02d} {name:24.24} map=({x:+.3f},{y:+.3f}) {label}')
    for idx, (a, b) in enumerate(zip(pts[:-1], pts[1:]), 1):
        length = math.hypot(b[1]-a[1], b[2]-a[2])
        n = max(1, math.ceil(length/args.sample_step))
        found = {}
        for j in range(n+1):
            t = j/n
            state = status(a[1]+t*(b[1]-a[1]), a[2]+t*(b[2]-a[2]))
            if state != 'FREE_GRID':
                found[state] = found.get(state, 0)+1
        if found:
            anomaly_count += 1
            print(f'WARN SEGMENT {idx:02d} {a[0]} -> {b[0]} '
                  f'grid anomalies over {n+1} samples: {found}')
    print('STATIC_GRID_SCREENING_COMPLETE')
    print('NOT_COMPETITION_CERTIFIED: floor lane lines, stop lines, dynamic '
          'obstacles, and Nav2 planner shortcuts are NOT validated by this map.')
    return 1 if anomaly_count else 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f'PREFLIGHT_ERROR: {exc}', file=sys.stderr)
        sys.exit(2)
