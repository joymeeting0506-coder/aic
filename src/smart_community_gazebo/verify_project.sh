#!/usr/bin/env bash
set -e
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

fail=0
check() {
  if [ -e "$ROOT/$1" ]; then
    printf '[OK] %s\n' "$1"
  else
    printf '[MISSING] %s\n' "$1"
    fail=1
  fi
}

check package.xml
check CMakeLists.txt
check launch_world.sh
check launch/sim.launch.py
check worlds/smart_community_v2.sdf
check models/traffic_light_red/model.sdf
check models/traffic_light_yellow/model.sdf
check models/traffic_light_green/model.sdf
check models/vehicle_1/model.sdf
check models/vehicle_2/model.sdf
check models/vehicle_3/model.sdf
check models/noncommunity_person_F1/model.sdf
check models/noncommunity_person_F2/model.sdf
check models/community_person_01/model.sdf
check models/community_person_16/model.sdf

if command -v gz >/dev/null 2>&1; then
  printf '[OK] Gazebo CLI: '
  gz sim --version | head -1
else
  printf '[WARN] gz command not found in current shell; source ROS Jazzy first.\n'
fi

if [ "$fail" -eq 0 ]; then
  echo 'Project file check passed.'
else
  echo 'Project file check failed.'
  exit 1
fi
