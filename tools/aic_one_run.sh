#!/usr/bin/env bash
# AIC integrated launch: run only when Gazebo is already running and robot is stationary.
set -Eeuo pipefail
ROOT="$HOME/aic_ws"
cd "$ROOT"
set +u
source "$ROOT/install/setup.bash"
set -u
mkdir -p "$ROOT/aic_results"
VISION="$ROOT/install/robot_description/lib/robot_description/aic_v5_vision.py"
[[ -f "$VISION" ]] || { echo 'RUN_ABORT: vision executable missing; run colcon build first'; exit 1; }

# Previous test sessions left Python children behind after 'ros2 run' exited.
# Stop ONLY this exact competition vision script, never Gazebo/Nav2/controller.
mapfile -t OLD < <(ps -eo pid=,args= | awk '$2=="python3" && $3 ~ /\/aic_v5_vision\.py$/ {print $1}')
if ((${#OLD[@]})); then
    echo "Stopping old aic_v5_vision.py processes: ${OLD[*]}"
    kill -TERM "${OLD[@]}" 2>/dev/null || true
    sleep 4
fi
VISION_PID=''
cleanup() {
    if [[ -n "$VISION_PID" ]]; then
        kill -TERM "$VISION_PID" 2>/dev/null || true
        wait "$VISION_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT INT TERM
python3 "$VISION" --snapshot-interval 0 --ros-args -p use_sim_time:=true \
   >"$ROOT/aic_results/vision_integrated.log" 2>&1 &
VISION_PID=$!
for i in $(seq 1 15); do
    kill -0 "$VISION_PID" 2>/dev/null || { tail -n 30 "$ROOT/aic_results/vision_integrated.log"; echo 'VISION_DIED'; exit 1; }
    RESULT=$(ros2 topic info -v /aic/traffic/observation 2>/dev/null || true)
    COUNT=$(sed -n 's/^Publisher count: //p' <<<"$RESULT")
    if [[ "$COUNT" == 1 && "$RESULT" == *'Node name: aic_v5_camera_perception'* ]]; then break; fi
    sleep 1
done
if [[ "$COUNT" != '1' || "$RESULT" != *'Node name: aic_v5_camera_perception'* ]]; then
    echo 'VISION_GRAPH_FAIL: expected exactly one real camera publisher'
    printf '%s\n' "$RESULT"
    exit 1
fi
echo 'VISION_READY: one camera node, auto lamp ROI, checkpoint-only screenshots'
echo 'The patrol retains all red-light, footprint and navigation safety aborts.'
ros2 run robot_description aic_v5_patrol.py \
    --mode patrol --ack-official-crossing "$@" \
    2>&1 | tee "$ROOT/aic_results/patrol_integrated.log"
