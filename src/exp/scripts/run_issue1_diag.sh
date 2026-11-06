#!/usr/bin/env bash
# Run a controlled Issue-1 diagnostic bag and post-process it.
#
# Usage:
#   ./run_issue1_diag.sh [bag_prefix] [measurement_mode] [motion_mode] [robot_speed] [robot_duration] [extra roslaunch args...]
#
# Examples:
#   ./run_issue1_diag.sh issue1_static real_umap preset_goal 0.0 8.0
#   ./run_issue1_diag.sh issue1_move_real real_umap preset_goal 0.4 8.0
#   ./run_issue1_diag.sh issue1_move_perf perfect preset_goal 0.4 8.0 stamp_offset:=0.0
#
# Default geometry is intentionally "obstacle ahead of the robot" for the
# current Gazebo spawn yaw (+pi/2): obstacle_x=0, obstacle_y=2, obstacle_z=0.5.

set -euo pipefail

BAG_PREFIX="${1:-issue1_run}"
MEASUREMENT_MODE="${2:-real_umap}"
MOTION_MODE="${3:-preset_goal}"
ROBOT_SPEED="${4:-0.4}"
ROBOT_DURATION="${5:-8.0}"
shift 5 || true

EFFECTIVE_MOTION_MODE="$MOTION_MODE"
if [[ "$MOTION_MODE" == "preset_goal" ]]; then
  if python3 - <<PY
speed=float("$ROBOT_SPEED")
raise SystemExit(0 if speed <= 0.0 else 1)
PY
  then
    EFFECTIVE_MOTION_MODE="none"
    echo "robot_speed <= 0: overriding motion_mode preset_goal -> none for true static baseline"
  fi
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
SETUP_SCRIPT="$WS_ROOT/devel/setup.bash"
CONFIG="$SCRIPT_DIR/../configs/issue1.yaml"

if [[ ! -f "$SETUP_SCRIPT" ]]; then
  echo "ERROR: workspace not built — run catkin build first" >&2
  exit 3
fi
source "$SETUP_SCRIPT"

TOPICS="$(python3 -c "import yaml; print(' '.join(yaml.safe_load(open('$CONFIG'))['runner']['rosbag_topics']))")"
WARMUP_SEC="$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['runner']['warmup_sec'])")"

rm -f "${BAG_PREFIX}"*.bag

LAUNCH_PID=""
BAG_PID=""
cleanup() {
  if [[ -n "${BAG_PID:-}" ]] && kill -0 "$BAG_PID" 2>/dev/null; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    wait "$BAG_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT

echo "============================================"
echo "  Issue-1 Diagnostic"
echo "  Prefix:            $BAG_PREFIX"
echo "  Measurement mode:  $MEASUREMENT_MODE"
echo "  Motion mode:       $EFFECTIVE_MOTION_MODE"
echo "  Robot speed:       $ROBOT_SPEED"
echo "  Robot duration:    $ROBOT_DURATION"
echo "  Warmup:            ${WARMUP_SEC}s"
echo "============================================"

roslaunch sup_exp issue1_static_obstacle.launch \
  measurement_mode:="$MEASUREMENT_MODE" \
  motion_mode:="$EFFECTIVE_MOTION_MODE" \
  robot_speed:="$ROBOT_SPEED" \
  robot_duration:="$ROBOT_DURATION" \
  "$@" &
LAUNCH_PID=$!

sleep "$WARMUP_SEC"

BAG_PATH="${BAG_PREFIX}.bag"
# shellcheck disable=SC2086
rosbag record -O "$BAG_PATH" $TOPICS &
BAG_PID=$!

sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "ERROR: rosbag process died immediately." >&2
  exit 4
fi

TOTAL_DURATION=$(python3 - <<PY
warmup = float("$WARMUP_SEC")
motion = float("$ROBOT_DURATION")
print(max(8.0, motion + 4.0))
PY
)
echo "Recording for ${TOTAL_DURATION}s ..."
sleep "$TOTAL_DURATION"

kill -INT "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true
BAG_PID=""

kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

if [[ ! -f "$BAG_PATH" ]]; then
  echo "ERROR: Bag file not found: $BAG_PATH" >&2
  exit 5
fi

python3 "$SCRIPT_DIR/analyze_issue1_tracking_drift.py" \
  --bag "$BAG_PATH" \
  --output "$BAG_PREFIX"

echo "Outputs:"
echo "  ${BAG_PREFIX}.bag"
echo "  ${BAG_PREFIX}_summary.json"
echo "  ${BAG_PREFIX}_samples.csv"
echo "  ${BAG_PREFIX}_timeseries.png"
echo "  ${BAG_PREFIX}_errors.png"
echo "  ${BAG_PREFIX}_displacement.png"
