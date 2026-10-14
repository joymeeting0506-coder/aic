#!/usr/bin/env python3
"""Create a separate Nav2 YAML from proven v3.1 without altering it.

Footprint is an oriented convex hull enclosing xacro body + wheels (+~1cm).
Nav2 MPPI CostCritic requires consider_footprint when using a polygon.
"""
import argparse
from pathlib import Path
import yaml
from aic_v5_audit import FOOTPRINT


def make(src,dst):
    a=yaml.safe_load(Path(src).read_text(encoding='utf-8'))
    if not isinstance(a,dict):raise ValueError('Invalid Nav2 params')
    foot='['+','.join('['+format(x,'.4f')+','+format(y,'.4f')+']' for x,y in FOOTPRINT)+']'
    for mapkind in ('local_costmap','global_costmap'):
        pars=a[mapkind][mapkind]['ros__parameters']
        pars.pop('robot_radius',None)
        pars['footprint']=foot
        # Since Nav2 uses inscribed radius from polygon, inflation still serves
        # obstacle buffer. It does NOT enforce road painting restrictions.
        pars['footprint_padding']=0.0  # polygon already has ~1cm hull margin
    ctr=a['controller_server']['ros__parameters']
    mppi=ctr['FollowPath']
    assert mppi['plugin']=='nav2_mppi_controller::MPPIController'
    mppi['CostCritic']['consider_footprint']=True
    # Conservative motion in 4.2m field; scene uses software lidar/camera.
    mppi['vx_max']=min(float(mppi.get('vx_max',.5)),.15)
    mppi['vx_min']=max(float(mppi.get('vx_min',-.35)),-.08)
    mppi['wz_max']=min(float(mppi.get('wz_max',1.9)),1.0)
    smoother=a['velocity_smoother']['ros__parameters']
    smoother['max_velocity'][0]=min(float(smoother['max_velocity'][0]),.15)
    smoother['min_velocity'][0]=max(float(smoother['min_velocity'][0]),-.08)
    a['planner_server']['ros__parameters']['GridBased']['allow_unknown']=False
    a['planner_server']['ros__parameters']['GridBased']['tolerance']=.1
    Path(dst).parent.mkdir(parents=True,exist_ok=True)
    Path(dst).write_text(yaml.safe_dump(a,allow_unicode=True,sort_keys=False),encoding='utf-8')
    print('V5_PARAMS_CREATED',dst,'convex_footprint=',foot)
    return a

if __name__=='__main__':
    ap=argparse.ArgumentParser();ap.add_argument('source');ap.add_argument('output');a=ap.parse_args()
    make(a.source,a.output)
