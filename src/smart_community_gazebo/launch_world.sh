#!/usr/bin/env bash
set -eo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [ -f /opt/ros/jazzy/setup.bash ]; then
  # ROS setup may reference unset variables; don't use `set -u`.
  source /opt/ros/jazzy/setup.bash
fi

export GZ_SIM_RESOURCE_PATH="$ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"
export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
export GZ_PARTITION="aic_smart_community_${USER}"

exec gz sim -r -v 2 --force-version 8   "$ROOT/worlds/smart_community_v2.sdf"   --render-engine ogre
