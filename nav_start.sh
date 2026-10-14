#!/usr/bin/env bash
set -e
workspace=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source /opt/ros/jazzy/setup.bash
source "$workspace/install/setup.bash"

act() {
  ros2 lifecycle set "$1" activate 2>/dev/null || {
    ros2 lifecycle set "$1" configure
    ros2 lifecycle set "$1" activate
  }
}
act /map_server
act /amcl
echo 'Fresh spawn at START only: map initial pose (0,0,0).'
ros2 run robot_description initial_pose.py --ros-args -p use_sim_time:=true
for name in planner_server controller_server smoother_server behavior_server bt_navigator velocity_smoother collision_monitor; do
  act "/$name"
done
ros2 lifecycle get /amcl
