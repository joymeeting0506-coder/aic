#!/usr/bin/env python3
"""Pure-Python geometry, occupancy and graph-constrained coverage planning.

ROS-free: importable in a normal Python 3 test. A road graph is a SAFETY
CONSTRAINT for the current Gazebo demo, NOT a detector of roads in unknown sites.
"""
import heapq
import math


class Grid:
    def __init__(self, width, height, resolution, ox, oy, yaw, data):
        self.w = int(width)
        self.h = int(height)
        self.res = float(resolution)
        self.ox, self.oy, self.yaw = float(ox), float(oy), float(yaw)
        self.data = list(data)
        if len(self.data) != self.w * self.h:
            raise ValueError('OccupancyGrid data length mismatch')

    def index(self, x, y):
        return int(y) * self.w + int(x)

    def inside(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def to_cell(self, wx, wy):
        dx, dy = wx - self.ox, wy - self.oy
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        x, y = c * dx + s * dy, -s * dx + c * dy
        return int(math.floor(x / self.res)), int(math.floor(y / self.res))

    def to_world(self, x, y):
        dx, dy = (x + .5) * self.res, (y + .5) * self.res
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return self.ox + c * dx - s * dy, self.oy + s * dx + c * dy


def pt_segment_dist(x, y, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    v = dx * dx + dy * dy
    t = min(1.0, max(0.0, ((x-ax)*dx + (y-ay)*dy)/v)) if v > 1e-12 else 0.0
    return math.hypot(x - (ax+t*dx), y - (ay+t*dy))


def polyline_dist(x, y, pts):
    if not pts:
        return float('inf')
    if len(pts) == 1:
        return math.hypot(x - pts[0][0], y - pts[0][1])
    return min(pt_segment_dist(x,y,*a,*b) for a,b in zip(pts,pts[1:]))


def road_samples(segments, step=.28):
    """Return [(x,y)] of spacing <= step; order independent of itinerary."""
    samples = []
    for a,b in segments:
        d = math.dist(a,b)
        n = max(1,math.ceil(d/step))
        for k in range(n+1):
            t=k/n
            p=(a[0]*(1-t)+b[0]*t, a[1]*(1-t)+b[1]*t)
            if not any(math.dist(p,q)<.07 for q in samples):
                samples.append(p)
    return samples


def masks(grid, segments, corridor_radius=.095, robot_radius=.22, free_ceiling=20):
    """Build conservative footprint-safe cells and road-centreline geofence.

    Non-free cells (including unknown and ambiguous) are obstacles for driving.
    This is intentionally fail-closed near occlusions and near map edges.
    """
    w,h,r=grid.w,grid.h,grid.res
    dr=math.ceil(robot_radius/r)
    offsets=[(dx,dy) for dx in range(-dr,dr+1) for dy in range(-dr,dr+1)
             if math.hypot(dx,dy)*r <= robot_radius+1e-9]
    unsafe=set()
    blocked={i for i,v in enumerate(grid.data) if v<0 or v>free_ceiling}
    for i in blocked:
        ix,iy=i%w,i//w
        for dx,dy in offsets:
            nx,ny=ix+dx,iy+dy
            if grid.inside(nx,ny):
                unsafe.add(grid.index(nx,ny))
    # Treat off-map cells as unsafe within robot radius as well.
    valid=set()
    for y in range(h):
        for x in range(w):
            i=grid.index(x,y)
            if i in unsafe or grid.data[i] < 0 or grid.data[i] > free_ceiling:
                continue
            if min(x+.5,y+.5,w-x-.5,h-y-.5)*r <= robot_radius:
                continue
            wx,wy=grid.to_world(x,y)
            if any(pt_segment_dist(wx,wy,*a,*b) <= corridor_radius for a,b in segments):
                valid.add(i)
    return valid


def closest_cell(grid, valid, point, radius=.18):
    x,y=grid.to_cell(*point)
    n=int(math.ceil(radius/grid.res))
    result=None; dmin=1e9
    for cy in range(y-n,y+n+1):
        for cx in range(x-n,x+n+1):
            if grid.inside(cx,cy) and grid.index(cx,cy) in valid:
                d=math.dist(point,grid.to_world(cx,cy))
                if d < dmin and d<=radius:
                    result=grid.index(cx,cy);dmin=d
    return result


def astar(grid, valid, start, goal):
    if start not in valid or goal not in valid:
        return None
    if start==goal:
        return [start]
    w=grid.w
    gx,gy=goal%w,goal//w
    dist={start:0.}
    prev={}
    heap=[(0.,start)]
    visited=set()
    while heap:
        _,u=heapq.heappop(heap)
        if u in visited:continue
        visited.add(u)
        if u==goal:
            result=[u]
            while u in prev:
                u=prev[u];result.append(u)
            result.reverse()
            return result
        x,y=u%w,u//w
        for dx,dy in ((0,1),(1,0),(-1,0),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1)):
            nx,ny=x+dx,y+dy
            if not grid.inside(nx,ny):continue
            v=grid.index(nx,ny)
            if v not in valid:continue
            if dx and dy:
                # No diagonal corner cutting, especially near line/obstacle boundaries.
                if grid.index(x+dx,y) not in valid or grid.index(x,y+dy) not in valid:
                    continue
            cost=math.hypot(dx,dy)
            new=dist[u]+cost
            if new<dist.get(v,1e9):
                dist[v]=new;prev[v]=u
                heapq.heappush(heap,(new+math.hypot(gx-nx,gy-ny),v))
    return None


def frontier_indices(grid,valid):
    """Find known safe road cells close to unknown patches. No entry into unknown."""
    found=[]
    offsets=[(dx,dy) for dx in range(-8,9) for dy in range(-8,9)
             if .2<=math.hypot(dx,dy)*grid.res<=.45]
    for i in valid:
        x,y=i%grid.w,i//grid.w
        if any(grid.inside(x+dx,y+dy) and grid.data[grid.index(x+dx,y+dy)]<0
               for dx,dy in offsets):
            found.append(i)
    return found


def path_distance(grid, path):
    return sum(math.dist(grid.to_world(a%grid.w,a//grid.w),
                         grid.to_world(b%grid.w,b//grid.w))
               for a,b in zip(path,path[1:]))


def select_target(grid, valid, pose, samples, visited, rejected, frontier_only=False):
    """Plan to nearest unvisited candidate; returns (sample_index,path) or None.

    A finite sampling lattice covers drivable roads even if 360-deg lidar has
    already labelled them FREE. This avoids frontier-only coverage blind spots.
    """
    start=closest_cell(grid,valid,pose,radius=.20)
    if start is None:return None
    frontiers=set(frontier_indices(grid,valid))
    ranked=[]
    for k,point in enumerate(samples):
        if k in visited or k in rejected:continue
        goal=closest_cell(grid,valid,point,radius=.10)
        if goal is None:continue
        if frontier_only and goal not in frontiers:continue
        d=math.dist(pose,point)
        # Visit nearby frontiers first, then other unswept road cells.
        priority=d - (.16 if goal in frontiers else 0.)
        ranked.append((priority,k,goal))
    ranked.sort()
    # In a maze, nearest geometric target may be inaccessible: test more.
    best=None; best_cost=1e9
    for _,k,goal in ranked[:100]:
        path=astar(grid,valid,start,goal)
        if path is not None:
            plen=path_distance(grid,path)
            cost=plen-(.16 if goal in frontiers else 0.)
            if cost < best_cost:
                best=(k,path);best_cost=cost
    return best
