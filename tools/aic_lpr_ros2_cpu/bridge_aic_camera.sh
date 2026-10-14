#!/usr/bin/env bash
# For nav_bringup.launch.py at upstream commit ec9f590, which has no camera bridge.
# Do not run this if an existing bridge already publishes /camera/image_raw.
set -eo pipefail
source /opt/ros/jazzy/setup.bash
exec ros2 run ros_gz_bridge parameter_bridge \
  'camera@sensor_msgs/msg/Image[gz.msgs.Image' \
  'camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo' \
  --ros-args -r __node:=aic_plate_camera_bridge \
  -r camera:=/camera/image_raw -r camera_info:=/camera/camera_info
