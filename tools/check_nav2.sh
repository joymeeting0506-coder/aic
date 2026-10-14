#!/usr/bin/env bash
set -Eeuo pipefail
set +u
source /opt/ros/jazzy/setup.bash
source "$HOME/aic_ws/install/setup.bash"
set -u
python3 "$HOME/aic_ws/tools/check_nav2_v3_1.py"
