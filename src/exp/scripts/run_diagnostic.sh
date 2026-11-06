#!/usr/bin/env bash
# run_diagnostic.sh — One-shot frame diagnostic experiment.
#
# Usage:
#   ./run_diagnostic.sh [bag_prefix] [duration_sec]
#   Defaults: bag_prefix=diagnostic_run, duration=60
#
# Steps:
#   1. Start diagnostic_frames.launch in background
#   2. Wait for warmup
#   3. Start rosbag record in background (shell-managed, not in launch)
#   4. Sleep for experiment duration
#   5. Stop rosbag via SIGINT (flushes bag cleanly)
#   6. Stop roslaunch
#   7. Run analyze_frame_errors.py

set -euo pipefail

BAG_PREFIX="${1:-diagnostic_run}"
DURATION="${2:-60}"
shift 2 || shift 1 || true

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"

_SETUP_SCRIPT="$WS_ROOT/devel/setup.bash"
if [[ -f "$_SETUP_SCRIPT" ]]; then
  source "$_SETUP_SCRIPT"
else
  echo "ERROR: workspace not built — run: cd $WS_ROOT && catkin build" >&2
  exit 3
fi

# Read topics and warmup from yaml config
CONFIG="$SCRIPT_DIR/../configs/diagnostic.yaml"
TOPICS="$(python3 -c "import yaml; print(' '.join(yaml.safe_load(open('$CONFIG'))['runner']['rosbag_topics']))")"
WARMUP_SEC="$(python3 -c "import yaml; print(yaml.safe_load(open('$CONFIG'))['runner']['warmup_sec'])")"

# Remove any old stale bags with this prefix
rm -f "${BAG_PREFIX}"*.bag

# ---- EXIT trap ----
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
echo "  Frame Diagnostic Experiment"
echo "  Prefix:   $BAG_PREFIX"
echo "  Duration: ${DURATION}s"
echo "  Warmup:   ${WARMUP_SEC}s"
echo "  Topics:   $TOPICS"
echo "============================================"

# 1. Start launch in background (rosbag is NOT in the launch file)
echo ""
echo "[1/4] Launching experiment nodes ..."
roslaunch sup_exp diagnostic_frames.launch "$@" &
LAUNCH_PID=$!

# 2. Wait for warmup
echo "[2/4] Warmup ${WARMUP_SEC}s ..."
sleep "$WARMUP_SEC"

# 3. Start rosbag in background
BAG_PATH="${BAG_PREFIX}.bag"
echo "[3/4] Starting rosbag: $BAG_PATH"
# shellcheck disable=SC2086
rosbag record -O "$BAG_PATH" $TOPICS &
BAG_PID=$!

# Verify rosbag started
sleep 2
if ! kill -0 "$BAG_PID" 2>/dev/null; then
  echo "ERROR: rosbag process died immediately. Check ROS master connectivity." >&2
  exit 3
fi

echo "  Recording for ${DURATION}s ..."
sleep "$DURATION"

# 4. Stop rosbag FIRST with SIGINT (triggers flush + clean close)
echo "[4/4] Stopping rosbag (SIGINT) ..."
kill -INT "$BAG_PID" 2>/dev/null || true
wait "$BAG_PID" 2>/dev/null || true
BAG_PID=""
echo "  Rosbag stopped."

# 5. Stop launch
echo "Stopping launch (SIGINT) ..."
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""
echo "  Launch stopped."

# ---- Verify bag exists ----
if [[ ! -f "$BAG_PATH" ]]; then
  echo "ERROR: Bag file not found: $BAG_PATH" >&2
  echo "Checking for timestamp-stamped bags..."
  ls -lt "${BAG_PREFIX}"*.bag ~/.ros/"${BAG_PREFIX}"*.bag 2>/dev/null || true
  exit 4
fi

BAG_SIZE=$(du -h "$BAG_PATH" | cut -f1)
echo ""
echo "  Bag file: $BAG_PATH ($BAG_SIZE)"

# ---- Analysis ----
echo ""
echo "============================================"
echo "  Running analysis on $BAG_PATH"
echo "============================================"

OUTPUT_BASE="${BAG_PREFIX}"
python3 "${SCRIPT_DIR}/analyze_frame_errors.py" \
    --bag "$BAG_PATH" \
    --output "$OUTPUT_BASE"

echo ""
echo "============================================"
echo "  Done. Outputs:"
echo "    ${OUTPUT_BASE}_timeseries.csv"
echo "    ${OUTPUT_BASE}_summary.json"
echo "============================================"
