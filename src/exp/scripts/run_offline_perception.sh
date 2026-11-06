#!/usr/bin/env bash
# run_offline_perception.sh
# ========================
# Offline rosbag replay through the DOA perception pipeline at reduced speed.
#
# The master bag is played back at the specified --rate (default 0.5x) with
# --clock so that perception nodes experience 2x wall-clock time per frame,
# simulating better GPU compute on a real robot.
#
# Perception output topics and reference topics (odom, obstacles, tf) are
# recorded into a new raw.bag, which is then processed by process_bag.sh.
#
# Requires: fastsam conda environment (for SAM/CLIP Python nodes).
#
# Usage:
#   run_offline_perception.sh --method METHOD --bag MASTER.bag --out-dir DIR
#                             [--rate RATE]
#
#   --method   {ours_full|ours_no_clip|ours_no_sam|stage1_only|dodt|fapp|oracle_gt}
#   --bag      Path to the master rosbag (must contain camera, odom, tf data)
#   --out-dir  Output directory for the new raw.bag and processed results
#   --rate     Playback rate multiplier (default: 0.5)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PROCESS_BAG="$SCRIPT_DIR/process_bag.sh"

ALLOWED_METHODS=(ours_full ours_no_clip ours_no_sam stage1_only dodt fapp oracle_gt)

# ---------- helpers ----------
log() { printf '[offline] %s\n' "$*" >&2; }

in_list() {
  local needle="$1"; shift
  local x
  for x in "$@"; do [[ "$x" == "$needle" ]] && return 0; done
  return 1
}

usage() {
  cat <<'EOF'
Usage: run_offline_perception.sh --method METHOD --bag BAG --out-dir DIR [--rate RATE]

  --method   Perception method (ours_full, ours_no_clip, etc.)
  --bag      Path to the master rosbag.
  --out-dir  Output directory for the new raw.bag and processed results.
  --rate     Playback speed multiplier (default: 0.5).
             Use 0.5 for 2x wall-clock time per frame.
  -h|--help  Print this usage and exit 0.
EOF
}

# ---------- defaults ----------
METHOD=""
MASTER_BAG=""
OUT_DIR=""
RATE="0.5"

# ---------- arg parsing ----------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --method)  METHOD="$2"; shift 2 ;;
    --bag)     MASTER_BAG="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --rate)    RATE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) log "ERROR: unknown arg: $1"; usage >&2; exit 2 ;;
  esac
done

# ---------- validation ----------
if [[ -z "$METHOD" ]]; then log "ERROR: --method required"; usage >&2; exit 2; fi
if [[ -z "$MASTER_BAG" ]]; then log "ERROR: --bag required"; usage >&2; exit 2; fi
if [[ -z "$OUT_DIR" ]]; then
  OUT_DIR="/data/rosbags/offline_perception"
  log "INFO: --out-dir not specified, using default: $OUT_DIR"
fi

if ! in_list "$METHOD" "${ALLOWED_METHODS[@]}"; then
  log "ERROR: invalid --method '$METHOD' (allowed: ${ALLOWED_METHODS[*]})"
  exit 2
fi

if [[ ! -f "$MASTER_BAG" ]]; then
  log "ERROR: master bag not found: $MASTER_BAG"
  exit 2
fi

# Validate the rate is a positive number
if ! python3 -c "x=float('$RATE'); assert x > 0" 2>/dev/null; then
  log "ERROR: --rate must be a positive number, got '$RATE'"
  exit 2
fi

# Conda check (SAM/CLIP nodes need fastsam env)
if [[ "$CONDA_DEFAULT_ENV" != "fastsam" ]]; then
  if [[ "$METHOD" == "ours_full" || "$METHOD" == "ours_no_clip" || "$METHOD" == "ours_no_sam" ]]; then
    log "WARNING: conda env is '$CONDA_DEFAULT_ENV', not 'fastsam'."
    log "         SAM/CLIP nodes may fail. Activate with: conda activate fastsam"
  fi
fi

# ---------- setup ----------
ABS_MASTER="$(cd "$(dirname "$MASTER_BAG")" && pwd)/$(basename "$MASTER_BAG")"
mkdir -p "$OUT_DIR"
ABS_OUT_DIR="$(cd "$OUT_DIR" && pwd)"

log "method=$METHOD  rate=$RATE"
log "master bag: $ABS_MASTER"
log "output dir: $ABS_OUT_DIR"

# ---------- source workspace ----------
if [[ -f "$WS_ROOT/devel/setup.bash" ]]; then
  source "$WS_ROOT/devel/setup.bash"
elif [[ -f "$WS_ROOT/devel/setup.zsh" ]]; then
  source "$WS_ROOT/devel/setup.zsh"
else
  log "ERROR: workspace not built; $WS_ROOT/devel/setup.bash not found"
  exit 3
fi

# ---------- recording topics ----------
# Perception output topics + reference topics from master bag that
# process_bag.sh needs (odom, obstacles, tf).
RECORD_TOPICS=(
  /odom
  /obstacles
  /tf
  /tf_static
  /rematch/tracker_details
  /doa_obstacles
  /tracker/dynamic_obstacles
  /rematch_statistics_sam
  /rematch_statistics_clip
  /rematch_statistics_aggregator
)

# ---------- trap cleanup ----------
LAUNCH_PID=""
RECORD_PID=""
PLAY_PID=""

cleanup() {
  log "cleaning up..."
  if [[ -n "${RECORD_PID:-}" ]] && kill -0 "$RECORD_PID" 2>/dev/null; then
    log "stopping rosbag record (pid=$RECORD_PID)"
    kill -INT "$RECORD_PID" 2>/dev/null || true
    wait "$RECORD_PID" 2>/dev/null || true
  fi
  if [[ -n "${LAUNCH_PID:-}" ]] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    log "stopping roslaunch (pid=$LAUNCH_PID)"
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
  if [[ -n "${PLAY_PID:-}" ]] && kill -0 "$PLAY_PID" 2>/dev/null; then
    kill -INT "$PLAY_PID" 2>/dev/null || true
    wait "$PLAY_PID" 2>/dev/null || true
  fi
  # Kill any leftover rosbag/roslaunch processes
  pkill -INT -f "rosbag record" 2>/dev/null || true
  pkill -INT -f "rosbag play" 2>/dev/null || true
  log "cleanup done"
}
trap cleanup EXIT

# ---------- 1. Start perception pipeline ----------
log "starting perception_offline.launch (method=$METHOD)"
roslaunch sup_exp perception_offline.launch method:="$METHOD" &
LAUNCH_PID=$!

log "waiting 5s for perception nodes to initialize..."
sleep 5

# ---------- 2. Start rosbag record ----------
# rosbag record -O appends .bag automatically, so pass just the stem.
RECORD_BAG="$ABS_OUT_DIR/raw.bag"
RECORD_STEM="$ABS_OUT_DIR/raw"
log "starting rosbag record -> $RECORD_BAG"
# rosbag record needs to use sim time when /use_sim_time is true.
# --buffsize increases the internal buffer to avoid dropping messages
# under high throughput.
rosbag record -O "$RECORD_STEM" --buffsize 2048 "${RECORD_TOPICS[@]}" &
RECORD_PID=$!

# Give rosbag record a moment to subscribe to all topics before playback starts.
sleep 1

# ---------- 3. Play master bag ----------
log "playing master bag at ${RATE}x speed with --clock"
log "  (this may take a while; rosbag play prints progress to stdout)"

# rosbag play publishes messages with original timestamps. --clock publishes
# /clock at the playback rate.  With use_sim_time=true this drives the
# perception pipeline's sense of time, giving SAM/CLIP more wall-clock time.
rosbag play --clock --rate "$RATE" "$ABS_MASTER" &
PLAY_PID=$!

# Wait for playback to finish.
wait "$PLAY_PID" || true
PLAY_PID=""

# ---------- 4. Drain perception backlog ----------
log "playback finished; waiting 3s for perception pipeline to drain..."
sleep 3

# ---------- 5. Stop recording ----------
log "stopping rosbag record (pid=$RECORD_PID)"
kill -INT "$RECORD_PID" 2>/dev/null || true
wait "$RECORD_PID" 2>/dev/null || true
RECORD_PID=""

# ---------- 6. Stop perception launch ----------
log "stopping roslaunch (pid=$LAUNCH_PID)"
kill -INT "$LAUNCH_PID" 2>/dev/null || true
wait "$LAUNCH_PID" 2>/dev/null || true
LAUNCH_PID=""

# Wait for roscore to fully shut down before processing
sleep 2

# ---------- 7. Process the output bag ----------
if [[ ! -f "$RECORD_BAG" ]]; then
  log "ERROR: output bag not created: $RECORD_BAG"
  exit 4
fi

BAG_SIZE="$(du -h "$RECORD_BAG" | cut -f1)"
BAG_INFO="$(rosbag info "$RECORD_BAG" 2>/dev/null || echo "rosbag info failed")"
log "output bag: $RECORD_BAG  ($BAG_SIZE)"
echo "$BAG_INFO" >&2

log "running process_bag.sh on $ABS_OUT_DIR"
if ! bash "$PROCESS_BAG" "$ABS_OUT_DIR"; then
  log "WARNING: process_bag.sh returned non-zero; some steps may have failed"
fi

log "done. results in: $ABS_OUT_DIR"
ls -la "$ABS_OUT_DIR"/*.csv "$ABS_OUT_DIR"/*.json 2>/dev/null || true
