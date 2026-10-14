#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
AIC_WORKSPACE="${AIC_WORKSPACE:-$HOME/aic_ws}"
if [[ -f "$AIC_WORKSPACE/install/setup.bash" ]]; then
  source "$AIC_WORKSPACE/install/setup.bash"
fi
AIC_LPR_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -x "$AIC_LPR_DIR/.venv/bin/python" ]]; then
  echo 'Run bash install_ubuntu_cpu.sh first (after sourcing ROS Jazzy).'
  exit 1
fi
exec "$AIC_LPR_DIR/.venv/bin/python" "$AIC_LPR_DIR/ros2_plate_node.py" --ros-args \
  -p image_topic:=/camera/image_clean \
  -p result_topic:=/plate_recognition/result \
  -p device:=cpu -p rate_hz:=2.0 -p cpu_threads:=2 "$@"
